# 設計: Web UI

> 対象機能: [features/05-web-ui.md](../features/05-web-ui.md) | 作成日: 2026-08-12 | 更新: 2026-10-17 | ステータス: 実装済み
> 関連ゴール: [G1](../やりたいこと.md#g1-ロッカーボギーロボット自動走行) [G2](../やりたいこと.md#g2-自動でアームでものを回収)

## 構成

```
ブラウザ (バニラ JS / ES モジュール, ビルド無し)
  ├ /api/map (5 s) /api/map.png (ETag + 304)
  ├ /api/pose (0.5 s) /api/status (1 s) /api/vision (1 s)
  └ /api/vision.mjpg (MJPEG, 1 接続を張りっぱなし)
        │
   httplib::Server (127.0.0.1:8088, スレッドプール)
        │  ports.hpp の口だけを見る (ROS 型を出さない)
   ├ MapSource    ← MapListener  (/map TRANSIENT_LOCAL, /tf)   ┐ 同一 executor
   ├ VisionSource ← VisionListener (/detection_image, /detections) ┘ (別スレッド)
   └ NavSink      ← NavWorker    (/navigate_to_pose action)     … 専用スレッド
```

| 層 | ファイル | 責務 |
|---|---|---|
| HTTP | `http_app.cpp` `/ `vision_routes.cpp` / `http_support.cpp` | 経路登録、ETag/304、JSON、MJPEG、静的ファイル |
| 変換 | `map_cache.cpp` / `vision_cache.cpp` | メッセージ → ブラウザが貼れる画像と状態 JSON |
| ROS | `map_listener.cpp` / `vision_listener.cpp` / `nav_worker.cpp` | 購読・アクション・スレッド |
| 口 | `ports.hpp` | `MapSource` / `VisionSource` / `NavSink`（gtest はここを stub する） |

`ports.hpp` に ROS 型を出さないので、HTTP 面は ROS グラフ無しでテストできる（AGENTS.md R8）。

## 設計判断

- **地図は内容ハッシュで版管理する**。メッセージ連番だと、同じ地図を再送する publisher が
  連番を進めるだけで ETag が変わり、304 が永久に成立しない。crop（既知セルの bbox + 5% 余白）と
  PNG エンコードは版ごとに 1 回だけ計算する（Python 版は 1 ポーリングで 2 回 crop していた）。
- **PNG は `cv::imencode`、認識画像は JPEG（品質 80）**。認識画像は 3 Hz で流れ続けるので、
  可逆圧縮の PNG を毎フレーム送ると帯域もブラウザのデコードも無駄になる。JPEG への変換は
  購読コールバックで 1 回だけ行い、`/api/vision.jpg` と MJPEG は同じバイト列を共有する。
- **検出の対応付け**は `Detection2D.id`（クラス名）・`results[0].hypothesis.score`・
  `bbox.center.position` / `size_x` / `size_y`（画素）。1 応答 12 件・スコア降順に丸める。
- **MJPEG は chunked のまま押し出す**。ブラウザ側は `<img src>` を 1 本張るだけでよく、
  JS で毎フレーム描き直すより軽い。ただし**終わらない応答なので `load` イベントが完了しない**
  （タブが「読み込み中」のままになる）。フロントは `load` 後に `src` を張る。
- **監視 ON/OFF は購読の作成/破棄で行う**。`detection_node` は購読者が居ない間だけ推論を
  止める設計（`detection_node.cpp` の `AnyoneListening()`）なので、購読を外すことがそのまま
  CPU を空ける操作になる。作成/破棄は **executor スレッドのタイマー**（200 ms）で行い、
  HTTP スレッドは `enabled_` を立てるだけにする（executor の callback group 収集と競合させない）。
- **既定は ON**。認識モニターは「見たいときだけ見る」機能ではなく、遠隔でロボットの
  見え方を確認する主要な計器なので、起動直後から映る方を既定にする。CPU を空けたいときは
  UI の 1 ボタンで切る。
- **`Cache-Control` の使い分け**: 地図 PNG は `no-cache`（保存してよいが必ず再検証 = ETag で
  304）、認識画像は `no-store`（1 枚ごとに別の絵なので保存に意味が無い）。
- **依存**: cpp-httplib（`libcpp-httplib-dev`、CMake config が無いので pkg-config で引く）と
  nlohmann/json（`nlohmann-json3-dev`）。**nlohmann-json3-dev は rosdep キーを持たない**
  （`rosdep resolve nlohmann-json3-dev` / `nlohmann_json` ともに "no rosdep rule"）ため
  `package.xml` には書かない。書くと `rosdep install` がワークスペースごと落ちる。apt 直入れ前提。

## スレッド構成

| スレッド | 動くもの | 注意 |
|---|---|---|
| httplib のプール | 経路ハンドラ。MJPEG のフレーム待ちは条件変数で最大 1 秒 | executor を掴まない |
| ROS executor (SingleThreaded) | `/map`・`/tf`・`/detection_image`・`/detections` のコールバックと監視 ON/OFF タイマー | 画像の JPEG 化はここで 1 回だけ |
| NavWorker | action client と結果待ち | 専用スレッドなので executor と競合しない |

## 実装上の逸脱（Python 版との差）

| 項目 | Python 版 | C++ 版 | 影響 |
|---|---|---|---|
| 地図 PNG のバイト列 | PIL | `cv::imencode` | 画素は同じ、バイト列は別物。ETag は内容ハッシュなので 304 の成立に影響しない |
| 版ハッシュ | BLAKE2b(8) | FNV-1a 64 | ETag の文字列が変わる（キャッシュは 1 回作り直される） |
| 本文上限超過 | 400 + `{"error": ...}` | httplib が 413 | 1 MB 超の本文のみ。UI は送らない |
| Nav2 起動待ち | `nav2_simple_commander` | `bt_navigator/get_state` のポーリング + action server 待ち | 状態機械の文字列は同じ |

## 未検証（✗）

- ブラウザでの描画結果そのもの（この開発環境のヘッドレス Brave は静的なページでも
  描画結果を返さないため、スクリーンショットでの確認は **✗ 未検証**。配線は
  `id` の突き合わせ・`/api/*` の突き合わせ・ES モジュールの構文と解決で機械検証している）
- 認識モニター ON のときの Orange Pi 5 Max での CPU 使用率（開発機では 3 Hz 動作を実測）
- 長時間運用での MJPEG 接続の安定性（再接続はブラウザの `error` → 3 秒後に張り直しのみ）

## セキュリティ

`127.0.0.1` のみに bind する。外部公開する場合は認証・TLS と、静的ファイル配信の
パストラバーサル対策（`weakly_canonical` の前方一致）の再確認が要る。

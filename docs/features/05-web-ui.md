# 機能: Web UI

> 対象リポジトリ: `sosukigara/2026-Meister` | 作成日: 2026-08-12 | 更新: 2026-10-17 | ステータス: 実装済み
> 設計: [design/05-web-ui.md](../design/05-web-ui.md)
> 機能分解: [functions/05-web-ui.md](../functions/05-web-ui.md)
> 関連ゴール: [G1](../やりたいこと.md#g1-ロッカーボギーロボット自動走行) [G2](../やりたいこと.md#g2-自動でアームでものを回収)

## 概要

`meister_web_nav` パッケージがローカルホストの Web UI（`127.0.0.1:8088`）を提供する。
ブラウザ上にライブ地図と**認識モニター（物体検出の枠つきカメラ画像）**を表示し、
地図上のクリック/ドラッグで複数ウェイポイントを指定して Nav2 巡回を実行できる。

2026-10-17 に Python 実装から C++（`ament_cmake`）へ移行し、同じ日に認識モニターを追加した。

## 実装済み機能

- [ ] サーバ起動: `web_nav_server`（C++）が HTTP サーバ・ROS executor・ナビワーカーを起動する
- [ ] ライブ地図表示: `map_listener` が `/map`（TRANSIENT_LOCAL QoS）を購読し、OccupancyGrid を
      PNG に描く。`map_cache` が crop・PNG エンコード・版（ETag）を 1 箇所で持つ
- [ ] 認識モニター: `vision_listener` が `/detection_image`・`/detections` を購読し、
      MJPEG（`GET /api/vision.mjpg`）と状態 JSON（`GET /api/vision`）を配る。
      状態 JSON には fps・フレーム経過 ms・古さ・解像度・検出件数・検出一覧が入る
- [ ] 認識モニターの ON/OFF: `POST /api/vision` で購読を張り替える。OFF の間は
      `detection_node` の購読者が居なくなるため**推論そのものが止まる**（CPU を空ける）
- [ ] 姿勢表示: `GET /api/pose` が map 座標の x/y/yaw を返し、地図上にロボットと軌跡を描く
- [ ] REST API: 以下の経路を提供する
  - `GET /api/map` / `GET /api/map.png` — 地図メタデータと PNG（ETag + 304 対応）
  - `GET /api/pose` / `GET /api/status` — 姿勢と航行状態
  - `GET /api/vision` / `GET /api/vision.jpg` / `GET /api/vision.mjpg` / `POST /api/vision`
  - `POST /api/nav` / `POST /api/cancel`
- [ ] ウェイポイント巡回: `nav_worker` が専用スレッドで `/navigate_to_pose` action を
      **地点ごとに順に**実行する。`waypoint_follower` がスタックに無いため `follow_waypoints`
      は使わない。SLAM が map→odom を出すので AMCL は使わない
- [ ] フロントエンド: `webui/index.html` + `style.css` + ES モジュール（`webui/js/*.js`）。
      計器盤レイアウト（地図/姿勢/認識/航行の 4 計器）、地図のパン・ズーム・全体表示、
      目的地の順序と合計距離、イベント履歴、キーボード操作（F/Z/C/Enter/Esc/V）
- [ ] launch 組込み: `launch/web_nav.launch.py` で単体起動でき、`mapping_nav.launch.py` からも
      起動される（実行ファイル名を変えていないので launch は無変更）

## 計画中機能

- 検出結果からの把持対象の選択（[09-object-grasping.md](../design/09-object-grasping.md)）
- 認証・TLS（外部公開する場合のみ。現状は localhost 前提）

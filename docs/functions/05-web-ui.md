# 機能分解: Web UI

> 要件: [features/05-web-ui.md](../features/05-web-ui.md) | 設計: [design/05-web-ui.md](../design/05-web-ui.md) | 作成日: 2026-08-12 | 更新: 2026-10-17
> 関連ゴール: [G1](../やりたいこと.md#g1-ロッカーボギーロボット自動走行) [G2](../やりたいこと.md#g2-自動でアームでものを回収)

## 機能（このコンポーネントがすること）

- ライブ地図（`/map` → PNG）をブラウザに表示する
- 認識画像（`/detection_image` → MJPEG）と検出一覧（`/detections`）を表示する
- 認識モニターの購読を ON/OFF する（OFF の間は検出ノードの推論も止まる）
- 地図上のクリック/ドラッグでウェイポイント（ヨー角含む）を指定する
- 指定したウェイポイントを Nav2 へ送信し、巡回を開始/キャンセルする
- ナビゲーション状態（初期化中/待機中/送信中/移動中/到着/失敗/停止）を表示する
- サーバ（httplib）・ROS executor（地図 + 認識）・ナビワーカーの 3 系統で並行動作する

## HTTP 経路

| 経路 | メソッド | 返すもの |
|---|---|---|
| `/` `/index.html` `/app.js` `/style.css` | GET | UI 本体（`/.*` は `webui/` 配下の静的配信） |
| `/api/map` | GET | 地図メタデータ（`has_map` / `resolution` / `width` / `height` / `origin` / `image_url`） |
| `/api/map.png` | GET | 地図 PNG。`ETag` + `Cache-Control: no-cache`、一致時は 304（本文なし） |
| `/api/pose` | GET | `{"has_pose", x, y, yaw}` |
| `/api/status` | GET | 航行状態（`state` / `message` / `nav2_ready` / `waypoint_count` / `current_waypoint`） |
| `/api/vision` | GET | 認識モニター状態（`monitor` / `has_frame` / `stale` / `fps` / `frame_age_ms` / `width` / `height` / `frame_id` / `seq` / `detection_count` / `detection_age_ms` / `detections[]` / `publishers{}` / `image_url` / `stream_url`） |
| `/api/vision.jpg` | GET | 最新フレーム 1 枚（`image/jpeg`、`no-store`）。未着なら 404 |
| `/api/vision.mjpg` | GET | MJPEG（`multipart/x-mixed-replace`）。監視 OFF のときは 204 |
| `/api/vision` | POST | `{"enabled": bool}` で監視 ON/OFF。真偽値以外は 400 |
| `/api/nav` | POST | `{"waypoints":[{x,y,yaw?}, ...]}`。空・非配列・数値でない値は 400 |
| `/api/cancel` | POST | 巡回のキャンセル要求 |

## 画面（`webui/`）

| 部品 | ファイル | 表示するもの |
|---|---|---|
| 計器帯 | `js/instruments.js` | 地図 / 姿勢 / 認識 / 航行 の 4 計器（値・補足・色） |
| 地図 | `js/map_view.js` | 地図 PNG、パン・ズーム・全体表示、目的地マーカ（順序とヨー）、ロボットと軌跡、縮尺 |
| 目的地 | `js/waypoints.js` | 一覧（座標・向き・削除）、地点数、合計距離 |
| 認識モニター | `js/vision_panel.js` | MJPEG 表示、fps・遅延・解像度・検出件数、検出一覧、ON/OFF |
| 航行 | `js/nav_status.js` | 状態バッジと文言、地点の進捗 |
| イベント | `js/events.js` | 時刻つき履歴（80 件）とトースト |
| 取得 | `js/api.js` | JSON 取得の共通化と接続断の通知 |

## 関連リンク

- 要件: [features/05-web-ui.md](../features/05-web-ui.md)
- 設計: [design/05-web-ui.md](../design/05-web-ui.md)

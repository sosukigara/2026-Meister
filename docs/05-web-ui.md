# 機能: Web UI

> 対象リポジトリ: `sosukigara/2026-Meister` | 作成日: 2026-08-12 | ステータス: 実装済み

## 概要

`meister_web_nav` パッケージがローカルホストの Web UI（ポート 8088）を提供する。
ブラウザ上にライブ地図を表示し、地図上のクリック/ドラッグで複数ウェイポイントを指定して Nav2 巡回を実行できる。

## 実装済み機能

- [ ] サーバ起動: `web_nav_server.py` で HTTP サーバ + 地図リスナ executor + ナビワーカーの 3 スレッドを起動する
- [ ] ライブ地図表示: `map_listener.py` が `/map`（TRANSIENT_LOCAL QoS）を購読し、OccupancyGrid を PNG にレンダリングする
- [ ] REST API: `http_app.py` が以下の API を提供する
  - `GET /api/map` — 地図メタデータ
  - `GET /api/map.png` — 地図画像
  - `GET /api/status` — ナビゲーション状態
  - `POST /api/nav` — ウェイポイント送信（巡回開始）
  - `POST /api/cancel` — 巡回キャンセル
- [ ] ウェイポイント巡回: `nav_worker.py` が専用スレッドで BasicNavigator の `followWaypoints` を実行し、キャンセル・状態・フィードバックを処理する（SLAM が map→odom を提供するため AMCL をスキップし `localizer='robot_localization'` を使用）
- [ ] フロントエンド: `webui/index.html` + `app.js` + `style.css`（バニラ JS + Canvas）で地図描画・ウェイポイントマーカ（ドラッグでヨー角設定）・状態ポーリング・送信/キャンセル操作を提供する
- [ ] launch 組込み: `launch/web_nav.launch.py` で単体起動でき、`mapping_nav.launch.py` からも起動される

## 計画中機能

- （現時点で計画中の機能はありません）

## 仕様化メモ

- API の入出力仕様（リクエスト/レスポンス形式・エラーコード）を詳細仕様化する。
- 状態遷移（待機 / 巡回中 / キャンセル / 完了 / 失敗）を設計図化する。
- セキュリティ: 現状は localhost 前提。外部公開時の認証・TLS 方針を検討。

# 設計: Web UI

> 対象機能: [features/05-web-ui.md](../features/05-web-ui.md) | 作成日: 2026-08-12 | ステータス: 実装済み
> 関連ゴール: [G1](../やりたいこと.md#g1-ロッカーボギーロボット自動走行) [G2](../やりたいこと.md#g2-自動でアームでものを回収)

## 設計メモ

- API の入出力仕様（リクエスト/レスポンス形式・エラーコード）を詳細仕様化する。
- 状態遷移（待機 / 巡回中 / キャンセル / 完了 / 失敗）を設計図化する。
- セキュリティ: 現状は localhost 前提。外部公開時の認証・TLS 方針を検討。

## 設計図化対象

- [ ] REST API 仕様（エンドポイントごとのリクエスト/レスポンス/エラーコード）
- [ ] ナビゲーション状態遷移図（待機 → 巡回中 → 完了/失敗、キャンセル経路）
- [ ] スレッド構成図（HTTP サーバ / 地図リスナ executor / ナビワーカー）

## 関連ドキュメント

- 機能要件: [features/05-web-ui.md](../features/05-web-ui.md)
- 機能分解: [functions/05-web-ui.md](../functions/05-web-ui.md)
- 関連設計: [04-navigation.md](04-navigation.md)（巡回の実行元） / [09-object-grasping.md](09-object-grasping.md)（把持対象の選択 UI として拡張予定）

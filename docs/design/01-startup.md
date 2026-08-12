# 設計: 起動・実行環境

> 対象機能: [features/01-startup.md](../features/01-startup.md) | 作成日: 2026-08-12 | ステータス: 実装済み

## 設計メモ

- 起動シーケンスの詳細（各 launch の依存関係・起動順序）を設計図化する際の起点。
- `start_meister.sh` と `scripts/start_mapping_nav.sh` の差分整理（旧版の廃止 or 統合）を要検討。

## 設計図化対象

- [ ] 起動シーケンス図（start_meister.sh → launch 群の依存関係・起動順序）
- [ ] プロセス管理（cleanup で終了させるプロセスの一覧とシグナル設計）

## 関連ドキュメント

- 機能要件: [features/01-startup.md](../features/01-startup.md)
- 機能分解: [functions/01-startup.md](../functions/01-startup.md)
- 関連設計: [02-robot-model.md](02-robot-model.md)（起動対象のモデル定義） / [04-navigation.md](04-navigation.md)（起動対象のナビ）

# 設計: SLAM・地図作成

> 対象機能: [features/03-slam.md](../features/03-slam.md) | 作成日: 2026-08-12 | ステータス: 実装済み

## 設計メモ

- SLAM の質（地図再現精度・ループクローズの有効性）の確認方法と数値条件を詳細仕様化する。
- 地図作成時の操作手順（手動テレオペ vs 自動巡回）の確定が必要。

## 設計図化対象

- [ ] slam_toolbox 設定（mapper_params_online_async.yaml）の主要パラメータと意図
- [ ] 地図作成フロー図（起動 → 走行 → 保存 → 再利用）
- [ ] 地図品質の確認方法・受け入れ数値条件

## 関連ドキュメント

- 機能要件: [features/03-slam.md](../features/03-slam.md)
- 機能分解: [functions/03-slam.md](../functions/03-slam.md)
- 関連設計: [02-robot-model.md](02-robot-model.md)（`/scan` の供給元） / [04-navigation.md](04-navigation.md)（地図の利用元） / [10-auto-control.md](10-auto-control.md)（実機で LD-D500 を使用）

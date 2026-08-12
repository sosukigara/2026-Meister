# 設計: ナビゲーション

> 対象機能: [features/04-navigation.md](../features/04-navigation.md) | 作成日: 2026-08-12 | ステータス: 実装済み

## 設計メモ

- ナビゲーション精度（目標到達誤差・所要時間・障害物回避性能）の受け入れ基準を詳細仕様化する。
- プランナ（SmacPlanner2D 系）・コントローラ（DWB）のパラメータチューニングの意図を設計図に反映する。

## 設計図化対象

- [ ] Nav2 ノード構成図（プランナ / コントローラ / BT ナビゲータ / ライフサイクル管理）
- [ ] プランナ・コントローラパラメータのチューニング意図メモ
- [ ] ナビゲーション精度の受け入れ基準（到達誤差・所要時間・回避性能）

## 関連ドキュメント

- 機能要件: [features/04-navigation.md](../features/04-navigation.md)
- 機能分解: [functions/04-navigation.md](../functions/04-navigation.md)
- 関連設計: [03-slam.md](03-slam.md)（地図の供給元） / [05-web-ui.md](05-web-ui.md)（目標地点の入力元） / [10-auto-control.md](10-auto-control.md)（実機での自動移動に利用）

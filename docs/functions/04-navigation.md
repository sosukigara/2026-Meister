# 機能分解: ナビゲーション

> 要件: [features/04-navigation.md](../features/04-navigation.md) | 設計: [design/04-navigation.md](../design/04-navigation.md) | 作成日: 2026-08-12

## 機能（このコンポーネントがすること）

- SLAM で地図を作りながら同時にナビゲーションする（mapping_nav 統合起動）
- 保存済み地図上で AMCL 自己位置推定しながら移動する
- 目標地点への自律移動（`/navigate_to_pose`）を提供する
- 複数ウェイポイントの順次巡回（nav2_simple_commander）を提供する
- キーボードによる手動操作（teleop_twist_keyboard）を提供する
- GPU/CPU を自動検出し、URDF・LiDAR スペック・描画エンジンを切り替える
- 地図・コストマップ・経路・パーティクルを RViz で可視化する

## 関連リンク

- 要件: [features/04-navigation.md](../features/04-navigation.md)
- 設計: [design/04-navigation.md](../design/04-navigation.md)

# 機能分解: SLAM・地図作成

> 要件: [features/03-slam.md](../features/03-slam.md) | 設計: [design/03-slam.md](../design/03-slam.md) | 作成日: 2026-08-12

## 機能（このコンポーネントがすること）

- LiDAR（`/scan`）とオドメトリから**マップを作成**する（slam_toolbox 非同期 SLAM）
- 作成した地図を PGM/YAML として**保存**する（map_saver_cli）
- 保存済み地図（`maps/nav_world.pgm`、12 m × 12 m / 0.05 m/px）を提供する
- SDF の壁ジオメトリから直接占有グリッド地図を**生成**する（generate_map.py、SLAM 不要）
- 実機向けの SLAM パラメータを保持する

## 関連リンク

- 要件: [features/03-slam.md](../features/03-slam.md)
- 設計: [design/03-slam.md](../design/03-slam.md)

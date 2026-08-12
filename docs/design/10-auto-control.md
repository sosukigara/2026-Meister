# 設計: 自動制御（自動移動 → 完全自律）

> 対象機能: [features/10-auto-control.md](../features/10-auto-control.md) | 作成日: 2026-08-12 | ステータス: 計画中
> 関連ゴール: [G1](../やりたいこと.md#g1-ロッカーボギーロボット自動走行) [G2](../やりたいこと.md#g2-自動でアームでものを回収)

## 設計メモ

- **LiDAR**: LD-D500（2D LiDAR）。既存の `slam_toolbox` / Nav2 / `maps/` 資産がそのまま利用できる。
- **センサ統合**: オドメトリは 6輪のエンコーダ（搭載する場合）から算出。`/scan` と odom を既存の SLAM/Nav2 配管に接続する。
- **実機移植の要点**: `use_sim_time` の切り替え、トピック名・座標系（map/odom/base_link）の整合、ESP32 経由の cmd_vel 配線。
- 完全自律の実現には障害物回避の統合と、把持失敗時のリカバリ（再試行・次の対象へ）が必要。

## 設計図化対象

- [ ] 実機のセンサ・アクチュエータ配管図（LD-D500 / エンコーダ / ESP32 / cmd_vel）
- [ ] 既存シミュレーション資産の実機移植マッピング（トピック・launch・パラメータの対応表）
- [ ] 完全自律の運用フロー（巡回 → 物体検出 → 把持 → 回収 → リカバリ）
- [ ] 障害物回避と把持の優先制御（ナビ vs アームの調停）

## 関連ドキュメント

- 機能要件: [features/10-auto-control.md](../features/10-auto-control.md)
- 機能分解: [functions/10-auto-control.md](../functions/10-auto-control.md)
- 関連設計: [03-slam.md](03-slam.md)（地図作成） / [04-navigation.md](04-navigation.md)（自動走行） / [08-rocker-bogie.md](08-rocker-bogie.md)（走行系） / [09-object-grasping.md](09-object-grasping.md)（把持連携）

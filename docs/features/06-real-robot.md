# 機能: 実機対応

> 対象リポジトリ: `sosukigara/2026-Meister` | 作成日: 2026-08-12 | ステータス: 計画中
> 設計: [design/06-real-robot.md](../design/06-real-robot.md)

## 概要

シミュレーションで検証した SLAM / Nav2 スタックを実ロボットで動作させるための対応。launch ファイルの実装痕跡は存在するが、実機での動作検証は未実施。

## 実装済み機能

- [ ] 実機起動 launch: `real_robot.launch.py` で `use_sim_time: false` に設定し、robot_state_publisher + SLAM + Nav2 を起動できる（外部の LiDAR / odom / Micro-ROS ドライバを前提）

## 計画中機能

- [ ] 実機での SLAM 地図作成の検証
- [ ] 実機でのナビゲーション動作の検証
- [ ] 外部ドライバ（LiDAR・モータ・odom）との統合確認

# 機能分解: ロボットモデル・シミュレーション

> 要件: [features/02-robot-model.md](../features/02-robot-model.md) | 設計: [design/02-robot-model.md](../design/02-robot-model.md) | 作成日: 2026-08-12
> 関連ゴール: [G1](../やりたいこと.md#g1-ロッカーボギーロボット自動走行)

## 機能（このコンポーネントがすること）

- ロボットの物理モデル（URDF/xacro）を定義する
- LiDAR の `/scan` を出力する（360° / 10 Hz / 10 m）
- `/cmd_vel` を受けて車体を動かす（VelocityControl）
- odom + TF を 50 Hz で出力する（OdometryPublisher）
- Gazebo ↔ ROS 2 のトピック（`/scan` `/cmd_vel` `/odom` `/tf` `/clock`）を橋渡しする
- Nav2 スタックの全パラメータ（483 行）を保持する

## 関連リンク

- 要件: [features/02-robot-model.md](../features/02-robot-model.md)
- 設計: [design/02-robot-model.md](../design/02-robot-model.md)

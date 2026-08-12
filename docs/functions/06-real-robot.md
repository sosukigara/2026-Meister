# 機能分解: 実機対応

> 要件: [features/06-real-robot.md](../features/06-real-robot.md) | 設計: [design/06-real-robot.md](../design/06-real-robot.md) | 作成日: 2026-08-12
> 関連ゴール: [G1](../やりたいこと.md#g1-ロッカーボギーロボット自動走行)

## 機能（このコンポーネントがすること）

- 実機用 launch（`real_robot.launch.py`）で robot_state_publisher + SLAM + Nav2 を起動する
- `use_sim_time: false` で実機のシステム時刻を使う
- 外部ドライバ（LiDAR / odom / Micro-ROS）からのトピックを前提とする

## 関連リンク

- 要件: [features/06-real-robot.md](../features/06-real-robot.md)
- 設計: [design/06-real-robot.md](../design/06-real-robot.md)

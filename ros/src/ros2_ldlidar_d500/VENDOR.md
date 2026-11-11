# Vendored: ros2_ldlidar_d500

LD-D500 LiDAR 用 ROS 2 ドライバの vendored copy。
公式 (LDROBOT) に D500 用 ROS 2 ドライバはないため、サードパーティ製を取り込んでいる。

- 上流: https://github.com/SamuelSRI/ros2_ldlidar_d500
- 取り込み commit: `ca450aa035d16e9e9e3fe544b7acc6071e7be9af` (2026-07-31)
- ライセンス: MIT (`LICENSE` 参照)
- 上流の README は `README.md` のまま残してある

## 上流からの変更点

- `config/d500.yaml`: `port` を上流の実機固有 by-path から `/dev/ttyUSB1` に戻し
  (ESP32 ブリッジが `/dev/ttyUSB0` を使うため)、
  `frame_id` を `lidar_link` (URDF `umeonigiri_description/urdf/lidar.xacro` (real_robot.launch.py の my_robot_cpu.urdf も同名) と一致) にした。
  実機起動 (`launch.sh` 実機モード → `real_robot.launch.py`) では launch 引数
  `lidar_port` で上書きするため、この yaml は単体の `d500.launch.py` 用フォールバック。

## 更新手順

上流に更新があれば `src/ros2_ldlidar_d500/` に上書き copy し、このファイルの commit を更新する。
`d500_node.cpp` は 370 行単一ファイルのため R1 の分割対象にしない (外部コード)。

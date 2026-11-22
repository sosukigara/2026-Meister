# Vendored: umeonigiri_lidar (上流: ros2_ldlidar_d500)

LD-D500 LiDAR 用 ROS 2 ドライバの vendored copy。
公式 (LDROBOT) に D500 用 ROS 2 ドライバはないため、サードパーティ製を取り込んでいる。
このリポジトリではパッケージ名を `umeonigiri_lidar` に変更している
（上流のパッケージ名は `ros2_ldlidar_d500`）。

- 上流: https://github.com/SamuelSRI/ros2_ldlidar_d500
- 取り込み commit: `ca450aa035d16e9e9e3fe544b7acc6071e7be9af` (2026-07-31)
- ライセンス: MIT (`LICENSE` 参照)
- `README.md` は上流の内容を元に、コマンド例のパッケージ名だけローカル名に揃えている

## 上流からの変更点

- パッケージ名を `ros2_ldlidar_d500` から `umeonigiri_lidar` に変更し、`package.xml` /
  `CMakeLists.txt` / `launch/d500.launch.py` / `README.md` の参照を合わせた。
  実行ファイル名 `d500_node` は上流のまま。
- `config/d500.yaml`: `port` を上流の実機固有 by-path から `/dev/ttyUSB1` に戻し
  (ESP32 ブリッジが `/dev/ttyUSB0` を使うため)、
  `frame_id` を `lidar_link` (URDF `umeonigiri_description/urdf/lidar.xacro` (real_robot.launch.py の my_robot_cpu.urdf も同名) と一致) にした。
  実機起動 (`launch.sh` 実機モード → `real_robot.launch.py`) では launch 引数
  `lidar_port` で上書きするため、この yaml は単体の `d500.launch.py` 用フォールバック。

## 更新手順

上流に更新があれば `src/umeonigiri_lidar/` に上書き copy し、このファイルの commit を更新する。
上書き後はパッケージ名 (`package.xml` / `CMakeLists.txt` / `launch/d500.launch.py` /
`README.md`) を `umeonigiri_lidar` に戻すこと。
`d500_node.cpp` は 370 行単一ファイルのため R1 の分割対象にしない (外部コード)。

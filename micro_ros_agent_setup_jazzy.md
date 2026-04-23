# ROS 2 Jazzy 自律走行・シミュレーション環境構築手順

本ドキュメントでは、Raspberry Pi 5 (Ubuntu 24.04) での実機開発と、PC上での Gazebo シミュレーションの両方を1から完結させるためのセットアップ手順をまとめます。

## 1. 必須パッケージの一括インストール (共通)

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-rviz2 \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-interfaces
```

## 2. パッケージ構成の準備

ロボットのモデルや設定ファイルを管理するための「箱」となるパッケージを作成します。

```bash
# ワークスペースに移動
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# パッケージの作成
ros2 pkg create --build-type ament_cmake your_bot_description --dependencies urdf xacro

# 必要なディレクトリの作成
cd your_bot_description
mkdir -p urdf launch config worlds
```

## 3. シミュレーション環境の設定 (Gazebo Harmonic)

### 3.1 URDF (xacro) への記述
`urdf/` 内のファイルに、移動用プラグインとセンサー設定を記述します。

*   **移動用 (Planar Move):**
    ```xml
    <gazebo>
      <plugin name="planar_move" filename="libgazebo_ros_planar_move.so">
        <ros><remapping>cmd_vel:=cmd_vel</remapping></ros>
        <robot_base_frame>base_link</robot_base_frame>
        <publish_odom>true</publish_odom>
        <publish_odom_tf>true</publish_odom_tf>
      </plugin>
    </gazebo>
    ```
*   **センサー用 (LiDAR):**
    Jazzy（Gazebo Harmonic）では `gpu_lidar` センサーを使用します。
    ```xml
    <sensor name="lidar" type="gpu_lidar">
      <gz_frame_id>lidar_link</gz_frame_id>
      <topic>scan</topic>
      <update_rate>10</update_rate>
      <lidar>
        <scan><horizontal><samples>640</samples><resolution>1</resolution><min_angle>-3.14</min_angle><max_angle>3.14</max_angle></horizontal></scan>
        <range><min>0.1</min><max>12.0</max></range>
      </lidar>
    </sensor>
    ```

### 3.2 Gazebo ブリッジ設定 (`config/bridge.yaml`)
Gazebo と ROS 2 間の通信（LiDARデータ等）を中継します。

```yaml
- ros_topic_name: "/scan"
  gz_topic_name: "/world/world_name/model/your_bot/link/lidar_link/sensor/lidar/scan"
  ros_type: "sensor_msgs/msg/LaserScan"
  gz_type: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS
```

### 3.3 ビルド設定 (`CMakeLists.txt`)
作成したフォルダをシステムが認識できるよう、`CMakeLists.txt` の末尾（`ament_package()` の前）に以下を追記します。

```cmake
install(DIRECTORY urdf launch config worlds
  DESTINATION share/${PROJECT_NAME}
)
```

### 3.4 環境変数の設定
Gazebo が自分のロボットモデルを見つけられるようにします。`.bashrc` に追記しておくと便利です。

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ros2_ws/install/your_bot_description/share
```

## 4. ビルドと実行

### ① ビルドの実行
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### ② シミュレーションの開始
`ros2 launch your_bot_description spawn.launch.py` 等で起動し、`teleop_twist_keyboard` で横移動を確認します。

### ③ SLAM と Navigation2
```bash
# SLAM
ros2 launch slam_toolbox online_async_launch.py
# Navigation2 (シミュレーション時は use_sim_time:=true)
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

## 5. micro-ROS Agent のインストール (実機用)

```bash
mkdir -p ~/microros_ws/src
cd ~/microros_ws/src
git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git

cd ~/microros_ws
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
```

## 6. 全方向移動の重要パラメータ (params.yaml)

| 設定項目 | 指定値 | 備考 |
| :--- | :--- | :--- |
| `robot_model_type` | `nav2_amcl::HolonomicRobotModel` | AMCL で横移動を考慮 |
| `holonomic_robot` | `true` | 全方向移動を許可 |
| `max_vel_y` | `0.5` (例) | 横方向の最大速度 |

---

### 💡 最終チェック
ビルド時に `--symlink-install` を使うことで、URDFや設定ファイルを書き換えた際に再ビルドなしで反映されるようになります。LiDARのデータが `/scan` トピックに来ない場合は、`bridge.yaml` のトピック名が Gazebo 側と一致しているか確認してください。

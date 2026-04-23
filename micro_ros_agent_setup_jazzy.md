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
# ワークスペースに移動（例：~/ros2_ws）
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# パッケージの作成
ros2 pkg create --build-type ament_cmake your_bot_description --dependencies urdf xacro

# 必要なディレクトリの作成
cd your_bot_description
mkdir -p urdf launch config worlds
```

## 3. シミュレーション環境の設定 (Gazebo Harmonic)

### 3.1 URDF (xacro) へのプラグイン追加
`urdf/` ディレクトリ内に作成するロボットモデルファイルに、全方向移動を実現するための **Planar Move Plugin** を追記します。

```xml
<gazebo>
  <plugin name="planar_move" filename="libgazebo_ros_planar_move.so">
    <ros>
      <remapping>cmd_vel:=cmd_vel</remapping>
      <remapping>odom:=odom</remapping>
    </ros>
    <robot_base_frame>base_link</robot_base_frame>
    <publish_odom>true</publish_odom>
    <publish_odom_tf>true</publish_odom_tf>
    <update_rate>50</update_rate>
  </plugin>
</gazebo>
```

### 3.2 Gazebo ブリッジ設定 (`bridge.yaml`)
Gazebo Harmonic と ROS 2 間の通信を中継する設定ファイル（`config/bridge.yaml`）を作成します。

```yaml
# LiDARデータをGazeboからROS 2へ流す設定
- ros_topic_name: "/scan"
  gz_topic_name: "/world/world_name/model/your_bot/link/lidar_link/sensor/lidar/scan"
  ros_type: "sensor_msgs/msg/LaserScan"
  gz_type: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS
```

### 3.3 シミュレーション起動用 Launch ファイル
`launch/` ディレクトリに、Gazebo の起動とロボットのスポーン（登場）を行うスクリプトを作成します。

*   **Robot State Publisher:** URDFを読み込んで座標系（TF）を配信。
*   **Spawn Entity:** Gazebo の世界にロボットを配置。
*   **GZ Bridge:** 上記 `bridge.yaml` を読み込み通信を開始。

## 4. micro-ROS Agent のインストール (実機用)

実機の ESP32 と通信するためのエージェントを準備します。

```bash
mkdir -p ~/microros_ws/src
cd ~/microros_ws/src
git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git

cd ~/microros_ws
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build
```

## 5. 実行手順

### ① シミュレーションの開始
作成した Launch ファイルを実行し、Gazebo 上でロボットを確認します。
`teleop_twist_keyboard` で横移動（`j`, `l`キー）ができるかテストしてください。

### ② SLAM (地図作成) の起動
```bash
ros2 launch slam_toolbox online_async_launch.py
```

### ③ Navigation2 (自動走行) の起動
```bash
# use_sim_time を true に設定
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

## 6. 全方向移動の重要パラメータ (params.yaml)

`nav2_bringup` で使用する設定ファイルで以下の項目を必ずチェックしてください。

| 設定項目 | 指定値 | 備考 |
| :--- | :--- | :--- |
| `robot_model_type` | `nav2_amcl::HolonomicRobotModel` | AMCL で横移動を考慮 |
| `holonomic_robot` | `true` | 全方向移動のコマンド許可 |
| `max_vel_y` | `0.5` (例) | 横方向の最大速度 |

---

### 💡 アドバイス
「1から順番にコマンドを叩けば動く」状態にするためには、各ファイル（URDFやLaunch）の配置場所をドキュメント通りに守ることが重要です。シミュレーションで $v_y$ が動いたら、いよいよ実機のマイコン側に逆運動学を実装しましょう！

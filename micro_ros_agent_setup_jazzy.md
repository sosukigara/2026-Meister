# ROS 2 Jazzy 自律走行・シミュレーション環境構築手順

本ドキュメントでは、Raspberry Pi 5 (Ubuntu 24.04) での実機開発と、PC上での Gazebo シミュレーションの両方に対応したセットアップ手順をまとめます。

## 1. 必須パッケージの一括インストール (実機・シミュ共通)

ナビゲーションや地図作成に必要な基本セットに加え、Jazzy での標準シミュレータである **Gazebo Harmonic (GZ)** とのブリッジパッケージをインストールします。

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

## 2. micro-ROS Agent のインストール (実機用)

ESP32 などのマイコンと通信するために、エージェントをソースからビルドします。

```bash
# ワークスペースの作成
mkdir -p ~/microros_ws/src
cd ~/microros_ws/src

# リポジトリのクローン (jazzyブランチ)
git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git

# 依存関係の解決とビルド
cd ~/microros_ws
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
```

## 3. Gazebo で全方向移動を有効にする (シミュ用)

Gazebo 上で四輪メカナムなどの全方向移動（Holonomic）を再現するために、URDF (xacro) ファイルに **Planar Move Plugin** を追記します。

### URDF への記述例
```xml
<gazebo>
  <plugin name="planar_move" filename="libgazebo_ros_planar_move.so">
    <ros>
      <!-- トピックのリマッピング -->
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

## 4. 実行手順：シミュレーションから実機まで

### ① シミュレーションでの動作確認
1. Gazebo を起動し、ロボットモデルをスポーンさせます。
2. `teleop_twist_keyboard` を起動し、横移動（`j`, `l`キー）でロボットが真横に動くか確認します。

### ② 自己位置推定 (SLAM) の起動
```bash
# マッピング（地図作成）を開始
ros2 launch slam_toolbox online_async_launch.py
```

### ③ Navigation2 の起動
シミュレーション時は `use_sim_time:=true` を忘れずに付与します。
```bash
# 全方向移動設定を反映した params.yaml を指定して起動
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

## 5. 全方向移動の重要パラメータ (params.yaml)

Navigation2 で全方向移動を有効にするため、設定ファイル（`params.yaml`）の以下の項目を確認・変更してください。

| 設定項目 | 指定値 | 備考 |
| :--- | :--- | :--- |
| `robot_model_type` | `nav2_amcl::HolonomicRobotModel` | AMCL で横移動の可能性を考慮する |
| `holonomic_robot` | `true` | コントローラーに全方向への移動指令を許可する |
| `max_vel_y` | `0.5` (例) | 横方向の最大速度（デフォルトは 0.0） |

---

### 💡 アドバイス
シミュレーションで $v_y$（横方向速度）が正しく反映されたら、次は実機側の ESP32 でも同じ速度指令を受け取れるように逆運動学（Kinematics）の実装を進めましょう。
1つずつトピックが通るか確認しながら進めるのが確実です。

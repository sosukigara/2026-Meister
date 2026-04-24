# ROS 2 Jazzy 自律走行・シミュレーション環境構築手順

本ドキュメントでは、Raspberry Pi 5 (Ubuntu 24.04) での実機開発と、PC上での Gazebo シミュレーションの両方を1から完結させるためのセットアップ手順をまとめます。
現在、このリポジトリ（Meistar）自体が必要なファイルを含む構成になっています。

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

## 2. パッケージ構成（このリポジトリの構成）

本リポジトリは以下の構成になっており、必要なファイルは `src/meistar_description` にまとめられています。

- `Meistar/` (Workspace Root)
  - `src/meistar_description/`
    - `urdf/` : ロボットモデル（`robot.urdf.xacro`）
    - `launch/` : 起動スクリプト（`spawn.launch.py`）
    - `config/` : 設定ファイル（`bridge.yaml`, `params.yaml`）

## 3. シミュレーション環境の設定

### 3.1 URDF (xacro) への記述
`urdf/robot.urdf.xacro` に、移動用プラグインとセンサー設定が記述されています。

*   **移動用 (Planar Move):** `libgazebo_ros_planar_move.so` を使用。
*   **センサー用 (LiDAR):** Gazebo Harmonic 用の `gpu_lidar` センサーを使用。

### 3.2 Gazebo ブリッジ設定 (`config/bridge.yaml`)
Gazebo と ROS 2 間の通信を定義しています。`gz_topic_name` は Gazebo 側のワールド・モデル名と一致させる必要があります。

### 3.3 ビルド設定 (`CMakeLists.txt`)
`urdf`, `launch`, `config`, `worlds` フォルダが正しくインストールされるよう設定済みです。

### 3.4 環境変数の設定
Gazebo がモデルを見つけられるよう、以下の設定を `.bashrc` に追記してください。
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/Meistar/install/meistar_description/share
```

## 4. ビルドと実行

### ① ビルドの実行
```bash
cd ~/Meistar
colcon build --symlink-install
source install/setup.bash
```

> [!TIP]
> 新しくターミナルを開くたびに `source ~/Meistar/install/setup.bash` を実行してください。

### ② シミュレーションの開始
```bash
ros2 launch meistar_description spawn.launch.py
```
起動後、別ターミナルで `teleop_twist_keyboard` を動かして動作確認を行います。

### ③ SLAM と Navigation2
```bash
# SLAM (地図作成)
ros2 launch slam_toolbox online_async_launch.py
# Navigation2 (自動走行)
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

## 5. micro-ROS Agent のインストール (実機用)

実機開発時は別のワークスペースを作成して管理することを推奨します。

```bash
mkdir -p ~/microros_ws/src
cd ~/microros_ws/src
git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git

cd ~/microros_ws
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
```

## 6. 全方向移動の重要パラメータ (params.yaml)

`config/params.yaml` に定義されています。
- `robot_model_type`: `nav2_amcl::HolonomicRobotModel`
- `holonomic_robot`: `true`
- `max_vel_y`: `0.5`


---

### 💡 最終チェック
ビルド時に `--symlink-install` を使うことで、URDFや設定ファイルを書き換えた際に再ビルドなしで反映されるようになります。LiDARのデータが `/scan` トピックに来ない場合は、`bridge.yaml` のトピック名が Gazebo 側と一致しているか確認してください。

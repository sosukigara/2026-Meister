# Meistar ROS 2 Navigation Workspace

MeistarロボットのROS 2開発用ワークスペースです。Gazeboシミュレーション環境での自律移動（SLAM + Nav2）がセットアップされています。

## 🚀 クイックスタート

ビルドから起動までを自動で行うスクリプトを用意しています：

```bash
./scripts/start_mapping_nav.sh
```

## 📂 構成と編集のガイド

今回のリファクタリングにより、各機能がモジュール化されています。

### 1. ロボットモデルの編集 (URDF/Xacro)
`src/meistar_description/urdf/` 配下に分割されています。
- `chassis.xacro`: **ロボットの外見やサイズ、色**を変えたい時に編集します。
- `lidar.xacro`: **LiDARの性能、位置、スキャン範囲**を変えたい時に編集します。
- `gazebo.xacro`: **シミュレーター上の物理挙動（摩擦、速度制御プラグイン等）**を変えたい時に編集します。

### 2. 起動設定の編集 (Launch)
`src/ros2_autonomous_nav/launch/` 配下に分割されています。
- `robot_state_publisher.launch.py`: Xacroの読み込み設定。
- `simulation.launch.py`: Gazebo、スポーン位置、**Bridge（通信設定）**の変更。
- `slam.launch.py`: SLAMの起動タイミングやパラメータ。
- `navigation.launch.py`: Nav2の起動ノード管理。

### 3. パラメータの編集 (Config)
`src/ros2_autonomous_nav/config/` 配下。
- `mapping_nav_params.yaml`: Nav2（経路計画、回避設定など）の詳細設定。
- `mapper_params_online_async.yaml`: SLAM（地図作成）の精度や範囲の設定。

## 💡 開発のヒント（プロンプト節約用）

- **ビルドの注意**: XacroやLaunchを変更した後は `colcon build` が必要ですが、`scripts/start_mapping_nav.sh` を使えば自動で実行されます。
- **実機移行時の注意**: 実機では `use_sim_time` を `false` にする必要があります。現在のLaunchファイルは引数で切り替えられるように設計されています。
- **通信エラー**: もし `/scan` や `/odom` が表示されない場合は、`simulation.launch.py` 内の `ros_gz_bridge` のパス設定を確認してください。

詳細は [docs/agent.md](docs/agent.md) を参照してください。

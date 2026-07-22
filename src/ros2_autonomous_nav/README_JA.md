# 自律走行ナビゲーション (ros2-autonomous-nav) 日本語ガイド

このリポジトリは、ROS 2 Jazzy と Gazebo を使用した、差動二輪ロボットの自律走行（ナビゲーション）の学習・実行用パッケージです。

## 主な機能
- **自動ナビゲーション**: Nav2 スタック（AMCL, NavfnPlanner, DWB）を使用。
- **GPU/CPU 自動検知**: グラフィックボードの有無を判定し、LiDARの解像度や描画エンジンを自動で最適化します。
- **地図作成 (SLAM)**: `slam_toolbox` を使った地図作成が可能です。
- **ウェイポイント走行**: 複数の地点を順番に回るスクリプトが同梱されています。

## 必要条件
以下のコマンドで、必要な ROS 2 パッケージをインストールしてください。
```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-rviz2 \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-simple-commander \
    ros-jazzy-teleop-twist-keyboard
```

## ビルド手順
現在のディレクトリ（`/home/so/Meistar/ros2-autonomous-nav`）を ROS 2 ワークスペースとしてビルドします。

```bash
cd /home/so/Meistar
source /opt/ros/jazzy/setup.bash
# ワークスペース全体をビルド
colcon build --symlink-install
source install/setup.bash
```

## 実行方法

3つのターミナルを立ち上げ、それぞれでワークスペースを `source` してください。

### ターミナル 1: ロボット + Gazebo + Nav2 の起動
```bash
bash run.sh
```
※ 自動で GPU または CPU モードが選択されます。

### ターミナル 2: RViz2 (視覚化) の起動
```bash
bash rviz.sh
```
地図、コストマップ、AMCL のパーティクル、経路が表示されます。

### ターミナル 3: 目的地の送信
以下のいずれかの方法でロボットを動かします。

- **方法 A**: RViz の画面上部にある「2D Nav Goal」ボタンを押し、地図上をクリックする。
- **方法 B (コマンドライン)**: 指定した座標（x y yaw）へ移動させる。
  ```bash
  bash navigate.sh 3.5 3.5 1.57
  ```
- **方法 C (巡回走行)**: 迷路内のウェイポイントを順番に回ります。
  ```bash
  bash waypoints.sh
  ```

---

## 独自の地図を作る場合 (SLAM)
付属の地図ではなく、自分で地図を作りたい場合は以下の手順で行います。

1. **SLAMモード起動**: `bash slam.sh`
2. **RViz起動**: `bash rviz.sh`
3. **ロボットを操縦**: `bash drive.sh` (キーボードで操作)
4. **地図の保存**: 地図が完成したら `bash save_map.sh <map_name>`

---

## ファイル構成の概要
- `urdf/`: GPU用/CPU用それぞれのロボット定義
- `worlds/`: 壁に囲まれた迷路のシミュレーション世界
- `maps/`: 事前に作成された地図データ
- `config/`: Nav2 や SLAM の詳細パラメータ
- `scripts/`: ウェイポイント走行用などの Python スクリプト

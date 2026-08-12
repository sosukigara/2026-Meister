# 機能: ロボットモデル・シミュレーション

> 対象リポジトリ: `sosukigara/2026-Meister` | 作成日: 2026-08-12 | ステータス: 実装済み

## 概要

`meistar_description` パッケージでロボットの物理モデル（URDF/xacro）と Gazebo シミュレーション環境を定義する。
差動駆動ロボットを想定し、LiDAR センサ・速度制御・オドメトリは Gazebo プラグインで再現される。

## 実装済み機能

- [ ] ロボットモデル定義: `robot.urdf.xacro` でシャーシ・LiDAR・Gazebo プラグインを統合した URDF を提供する
- [ ] シャーシ定義: `chassis.xacro` でベースリンク/フットプリント・慣性・衝突判定・マテリアルを定義する
- [ ] LiDAR センサ: `lidar.xacro` で 360° 全方位 `gpu_lidar`（10 Hz, 640 サンプル, 検出距離 10 m）を定義し `/scan` をパブリッシュする
- [ ] 速度制御: `gazebo.xacro` の `VelocityControl` プラグインで `/cmd_vel` を受けてロボットを動かす
- [ ] オドメトリ: `gazebo.xacro` の `OdometryPublisher` プラグインで odom + TF を 50 Hz でパブリッシュする
- [ ] トピックブリッジ: `bridge.yaml` で Gazebo ↔ ROS 2 間の `/scan`, `/cmd_vel`, `/odom`, `/tf`, `/clock` を橋渡しする
- [ ] シミュレーションワールド: `worlds/` に Gazebo 環境（`my_custom_world.sdf`, `my_world.sdf`）を用意する
- [ ] 単体起動: `spawn.launch.py` で robot_state_publisher + Gazebo + スポーン + ブリッジを起動できる
- [ ] 統合起動: `all_system.launch.py` で Gazebo + SLAM + Nav2 を一括起動できる
- [ ] Nav2 設定: `config/nav2_params.yaml` に全 483 行の Nav2 スタック設定（AMCL / MPPI コントローラ / プランナ / BT ナビゲータ / スムーザ / ウェイポイントフォロワ / ルートサーバ / 衝突モニタ / ドッキングサーバ）を保持する

## 計画中機能

- （現時点で計画中の機能はありません）

## 仕様化メモ

- ロボット実機のスペック（ホイール径・トレッド・モータ仕様など）が確定したらモデルパラメータとの対応を設計図化する。
- `nav2_params.yaml` の各コントローラ/プランナ設定値の意図（経路品質のチューニング結果）を文書化する価値がある。

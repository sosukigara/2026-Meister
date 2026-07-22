# 次のステップ：実機プロジェクトへの移行ロードマップ

このドキュメントは、シミュレーションでの検証を終え、実機（ESP32 + シリアル通信 + LiDAR）での全方向移動ナビゲーションを実現するための指針です。

## 1. 現在のステータス
- **シミュレーション**: `ros2-autonomous-nav` パッケージにて、全方向移動（Holonomic）の Nav2 設定と物理プラグインの設定が完了。
- **Nav2パラメータ**: `max_vel: 1.0 m/s`, `controller_frequency: 20Hz` という高レスポンスな設定を適用済み。
- **ワークフロー**: SLAMによる地図生成（`start_mapping.sh`）→ 地図保存（`save_map.sh`）→ 自動航行（`start_nav.sh`）の一連の流れをスクリプト化。

## 2. 次に作成すべきもの（実機用）
実機プロジェクト（仮称：`meistar_real`）を立ち上げる際、以下のコンポーネントが必要です。

### A. Base Serial Bridge (ROS 2 ↔ ESP32)
シリアルポート（USB）を介して、PCとESP32を繋ぐ心臓部です。
- **Subscribe**: `/cmd_vel` (geometry_msgs/Twist)
    - 受信した `linear.x`, `linear.y`, `angular.z` を文字列またはバイナリに変換してESP32へ送信。
- **Publish**: `/odom` (nav_msgs/Odometry) & `/tf` (odom -> base_link)
    - ESP32から送られてくる座標（x, y）と角度（yaw）をROSメッセージに変換して配信。

### B. 実機用 URDF
シミュレーション用の URDF から Gazebo プラグインを除去し、実機の物理寸法を正確に反映させたもの。
- LiDARの取付位置（offset）を正確に記述することがSLAMの精度に直結します。

### C. 実機一括起動ローンチ
- LiDARドライバ、Serial Bridge、Robot State Publisher を一度に立ち上げるファイル。

## 3. 実機特有の注意事項
- **`use_sim_time`**: 実機では必ず `false` に設定すること。
- **シリアルプロトコル**: ESP32側と「どんな書式でデータを送るか（例：CSV形式 `vx,vy,wz\n`）」を握る必要があります。
- **キネマティクス**: 実機が「ロッカーボギー ＋ 全輪ステアリング」であるため、ESP32側（またはBridgeノード側）で `vx, vy, wz` から各車輪の角度と速度への変換を行う必要があります。

---
このロードマップに沿って進めることで、シミュレーションの成果をそのまま現実に移植できます。

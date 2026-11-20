# serial_bridge_node.cpp 分割（R1 フォローアップ）

## 目的

`ros/src/umeonigiri_serial_bridge/src/serial_bridge_node.cpp` が 395 行で R1 の
300 行目安を超えている。挙動を変えずに、出版の責務を分離して 300 行以下にする。

## 方針

`BridgeCore`（ROS 非依存）はそのまま。rclcpp に依存する出版だけを 2 クラスへ出し、
新しいソースは**ノード実行ファイルにだけ**追加する（CLI と gtest は rclcpp 非依存のまま）。

| 新規 | 責務 | 移すコード |
|---|---|---|
| `OdometryPublisher` | `/odom` と `odom -> base_footprint` TF、指令/IMU の直列化 | `StepAndPublishOdometry` と `odom_mutex_`、odom/tf publisher |
| `FeedbackPublisher` | `/esp32/state` と `/imu/data` | `PublishState` / `PublishImu`、state/imu publisher |

- `OdometryPublisher(rclcpp::Node&, const OdometryConfig&, bool publish_odom, bool publish_tf, std::string odom_frame_id, std::string base_frame_id)`
- `UpdateTwistCommand(double vx, double wz, double now_s)`
- `UpdateImuReport(const ImuReport&, double now_s)`
- `StepAndPublish(double now_s)`
- `FeedbackPublisher(rclcpp::Node&)`, `PublishState(const Feedback&)`, `PublishImu(const ImuReport&)`

ノードはタイマーから `odometry_->StepAndPublish(NowSeconds())` を呼び、RX ループは
`feedback_publisher_->Publish*` と `odometry_->UpdateImuReport` を呼ぶ。

## 制約

- トピック名・型・QoS depth・frame_id・共分散・出版レートを変えない。
- メンバー破棄順を守る（タイマー → 購読 → publisher → core の順で破棄される現行順を維持）。
- プロトコル・`BridgeCore`・CLI・テストの挙動を変えない。

## 検証

1. `colcon test --packages-select umeonigiri_serial_bridge` が 67 tests / 0 failures。
2. `./build.sh` が 6 packages 成功。
3. 実機なしの起動スモーク: `/odom` が 50 Hz、`/tf` の `odom -> base_footprint`、
   `/odom` の frame_id/covariance が従来どおり。`/esp32/state` と `/imu/data` が存在。
4. `wc -l serial_bridge_node.cpp` が 300 未満。

## 非ゴール

- 出版レート・共分散・トピック名の変更。
- `BridgeCore` の再設計。
- ハードウェア検証（従来どおり ✗ 未検証）。

## コミット

1. `docs(superpowers): add the serial bridge node split plan`
2. `refactor(serial_bridge): extract odometry and feedback publishers`
3. `docs: record the serial bridge node split`

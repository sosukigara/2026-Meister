#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "umeonigiri_serial_bridge/ackermann_imu_odometry.hpp"
#include "umeonigiri_serial_bridge/serial_io.hpp"

namespace umeonigiri_serial_bridge {

/// 指令速度・エンコーダFB・IMU ヨーの受信と出版を 1 つの mutex で直列化する。ROS 非依存の
/// `AckermannImuOdometry` を内包し、ノードはタイマーから StepAndPublish を呼ぶだけ。
class OdometryPublisher {
 public:
  OdometryPublisher(rclcpp::Node& node, const OdometryConfig& config, bool publish_odom,
                    bool publish_tf, std::string odom_frame_id,
                    std::string base_frame_id);

  void UpdateTwistCommand(double vx, double wz, double now_s);
  void UpdateWheelFeedback(const Feedback& fb, double now_s);
  void UpdateImuReport(const ImuReport& imu, double now_s);

  /// 1 周期分を積分して出版する。now_s は単調時計 [秒]。
  void StepAndPublish(double now_s);

 private:
  rclcpp::Node& node_;
  std::mutex mutex_;
  AckermannImuOdometry odometry_;
  bool publish_odom_;
  bool publish_tf_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
};

}  // namespace umeonigiri_serial_bridge

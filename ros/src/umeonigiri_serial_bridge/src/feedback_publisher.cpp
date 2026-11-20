#include "umeonigiri_serial_bridge/feedback_publisher.hpp"

#include <memory>

namespace umeonigiri_serial_bridge {

FeedbackPublisher::FeedbackPublisher(rclcpp::Node& node) : node_(node) {
  state_pub_ = node_.create_publisher<std_msgs::msg::Int16MultiArray>("/esp32/state", 10);
  imu_pub_ = node_.create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
}

void FeedbackPublisher::PublishState(const Feedback& feedback) {
  auto msg = std::make_unique<std_msgs::msg::Int16MultiArray>();
  msg->data.reserve(feedback.encoders.size() + 2);
  for (int16_t encoder : feedback.encoders) {
    msg->data.push_back(encoder);
  }
  msg->data.push_back(static_cast<int16_t>(feedback.state));
  msg->data.push_back(static_cast<int16_t>(feedback.error_flags));
  state_pub_->publish(std::move(msg));
}

void FeedbackPublisher::PublishImu(const ImuReport& imu) {
  auto msg = std::make_unique<sensor_msgs::msg::Imu>();
  msg->header.stamp = node_.now();
  msg->header.frame_id = "imu_link";
  const YawQuat q = YawTenthsToQuat(imu.yaw_tenths);
  msg->orientation.x = 0.0;
  msg->orientation.y = 0.0;
  msg->orientation.z = q.z;
  msg->orientation.w = q.w;
  // BNO055 yaw orientation covariance (non-zero diagonal required by SLAM/EKF)
  msg->orientation_covariance = {
      0.01, 0.0, 0.0,
      0.0, 0.01, 0.0,
      0.0, 0.0, 0.0015};
  // yaw 以外は送っていないので、角速度・加速度の先頭を -1（未知）で示す。
  msg->angular_velocity_covariance[0] = -1.0;
  msg->linear_acceleration_covariance[0] = -1.0;
  imu_pub_->publish(std::move(msg));
}

}  // namespace umeonigiri_serial_bridge

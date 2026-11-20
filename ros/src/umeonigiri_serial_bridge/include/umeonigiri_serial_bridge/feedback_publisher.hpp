#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include "umeonigiri_serial_bridge/serial_io.hpp"

namespace umeonigiri_serial_bridge {

/// `/esp32/state` と `/imu/data` の出版だけを持つ薄いクラス。
class FeedbackPublisher {
 public:
  explicit FeedbackPublisher(rclcpp::Node& node);

  void PublishState(const Feedback& feedback);
  void PublishImu(const ImuReport& imu);

 private:
  rclcpp::Node& node_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

}  // namespace umeonigiri_serial_bridge

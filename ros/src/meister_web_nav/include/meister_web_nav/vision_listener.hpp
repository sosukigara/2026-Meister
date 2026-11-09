// SPDX-License-Identifier: MIT
/// detection_image / detections を購読し、認識モニターの口を HTTP 面に出す。
///
/// 購読の作成と破棄は executor スレッド (タイマー) だけで行う。HTTP スレッドから
/// 直接作ると、executor が callback group を集めている最中と競合する。
#ifndef MEISTER_WEB_NAV_VISION_LISTENER_HPP_
#define MEISTER_WEB_NAV_VISION_LISTENER_HPP_

#include <atomic>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "meister_web_nav/ports.hpp"
#include "meister_web_nav/vision_cache.hpp"

namespace meister_web_nav {

class VisionListener : public rclcpp::Node, public VisionSource {
 public:
  VisionListener();

  nlohmann::json Status() override;
  std::optional<VisionFrame> Latest(long after_seq, double timeout_s) override;
  void SetEnabled(bool enabled) override;

 private:
  /// enabled_ と実際の購読状態を一致させる。タイマー (executor スレッド) から呼ぶ。
  void ApplyEnabled();

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
  rclcpp::TimerBase::SharedPtr reconcile_timer_;
  std::atomic<bool> enabled_{true};
  bool subscribed_ = false;
  VisionCache cache_;
};

}  // namespace meister_web_nav

#endif  // MEISTER_WEB_NAV_VISION_LISTENER_HPP_

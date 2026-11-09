// SPDX-License-Identifier: MIT
#include "meister_web_nav/vision_listener.hpp"

#include <chrono>

namespace meister_web_nav {
namespace {

constexpr char kNodeName[] = "web_nav_vision_listener";
constexpr char kImageTopic[] = "detection_image";
constexpr char kDetectionsTopic[] = "detections";
constexpr std::chrono::milliseconds kReconcilePeriod{200};

}  // namespace

VisionListener::VisionListener() : rclcpp::Node(kNodeName) {
  // 既定 ON。executor に載る前なので constructor で作ってよい。
  ApplyEnabled();
  reconcile_timer_ = create_wall_timer(kReconcilePeriod, [this] { ApplyEnabled(); });
}

void VisionListener::ApplyEnabled() {
  const bool want = enabled_.load();
  if (want == subscribed_) {
    return;
  }
  if (want) {
    // best_effort + depth 1: 表示に要るのは最新フレームだけ。取りこぼしを溜めると
    // 遅い描画に古い絵が混ざる。
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        kImageTopic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { cache_.SetFrame(*msg); });
    detections_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
        kDetectionsTopic, rclcpp::SensorDataQoS(),
        [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) {
          cache_.SetDetections(*msg);
        });
  } else {
    image_sub_.reset();
    detections_sub_.reset();
  }
  subscribed_ = want;
  RCLCPP_INFO(get_logger(), "認識モニター: %s", want ? "ON" : "OFF");
}

nlohmann::json VisionListener::Status() {
  nlohmann::json payload = cache_.Status();
  payload["monitor"] = enabled_.load();
  // 配信側が生きているかを見る。監視 OFF のときに「トピック自体は流れている」と
  // 分かるようにするため。
  payload["publishers"] = {{"detection_image", count_publishers(kImageTopic)},
                           {"detections", count_publishers(kDetectionsTopic)}};
  return payload;
}

std::optional<VisionFrame> VisionListener::Latest(long after_seq, double timeout_s) {
  return cache_.Latest(after_seq, timeout_s);
}

void VisionListener::SetEnabled(bool enabled) { enabled_.store(enabled); }

}  // namespace meister_web_nav

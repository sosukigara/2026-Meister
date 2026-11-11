// SPDX-License-Identifier: MIT
#include "meister_web_nav/vision_listener.hpp"

#include <chrono>

namespace meister_web_nav {
namespace {

constexpr char kNodeName[] = "umeonigiri_web_nav_vision_listener";
constexpr char kImageTopic[] = "detection_image";
constexpr char kDetectionsTopic[] = "detections";
constexpr std::chrono::milliseconds kReconcilePeriod{100};
constexpr std::chrono::seconds kIdleSuspendTimeout{10};

}  // namespace

VisionListener::VisionListener() : rclcpp::Node(kNodeName) {
  // 既定 ON。executor に載る前なので constructor で作ってよい。
  TouchClientActivity();
  ApplyEnabled();
  reconcile_timer_ = create_wall_timer(kReconcilePeriod, [this] { ApplyEnabled(); });
}

void VisionListener::TouchClientActivity() noexcept {
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          Clock::now().time_since_epoch())
                          .count();
  last_client_access_ns_.store(now_ns, std::memory_order_relaxed);
}

bool VisionListener::IsClientRecentlyActive() const noexcept {
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          Clock::now().time_since_epoch())
                          .count();
  const auto last_ns = last_client_access_ns_.load(std::memory_order_relaxed);
  const auto timeout_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(kIdleSuspendTimeout).count();
  return (now_ns - last_ns) <= timeout_ns;
}

void VisionListener::ApplyEnabled() {
  const bool want = enabled_.load(std::memory_order_relaxed) && IsClientRecentlyActive();
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
  RCLCPP_INFO(get_logger(), "認識モニター購読状態: %s (enabled=%d)",
              want ? "ACTIVE" : "SUSPENDED",
              static_cast<int>(enabled_.load(std::memory_order_relaxed)));
}

nlohmann::json VisionListener::Status() {
  TouchClientActivity();
  nlohmann::json payload = cache_.Status();
  payload["monitor"] = enabled_.load(std::memory_order_relaxed);
  // 配信側が生きているかを見る。監視 OFF のときに「トピック自体は流れている」と
  // 分かるようにするため。
  payload["publishers"] = {{"detection_image", count_publishers(kImageTopic)},
                           {"detections", count_publishers(kDetectionsTopic)}};
  return payload;
}

std::optional<VisionFrame> VisionListener::Latest(long after_seq, double timeout_s) {
  TouchClientActivity();
  return cache_.Latest(after_seq, timeout_s);
}

void VisionListener::SetEnabled(bool enabled) {
  TouchClientActivity();
  enabled_.store(enabled, std::memory_order_relaxed);
}

}  // namespace meister_web_nav

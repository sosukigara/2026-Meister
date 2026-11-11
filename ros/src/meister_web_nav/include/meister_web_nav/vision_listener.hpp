// SPDX-License-Identifier: MIT
/// detection_image / detections を購読し、認識モニターの口を HTTP 面に出す。
///
/// 購読の作成と破棄は executor スレッド (タイマー) だけで行う。HTTP スレッドから
/// 直接作ると、executor が callback group を集めている最中と競合する。
///
/// 最適化（ヘッドレス走行時のアイドル負荷削減）:
///   Web UI にブラウザが 1 台も接続していない状態（/api/vision* へのアクセスが
///   kIdleSuspendTimeout (10秒) 以上途絶えた状態）では、921 KB/frame の
///   detection_image 購読を自動的に一時休止（オンデマンド・スリープ）する。
///   これにより、ブラウザ非接続時は detection_node 側の DrawDetections() 描画処理、
///   921 KB 画像の DDS 転送、および VisionCache::SetFrame() の JPEG エンコード負荷が
///   完全にゼロになる。ブラウザが /api/vision* にアクセスした瞬間に購読は自動再開する。
#ifndef MEISTER_WEB_NAV_VISION_LISTENER_HPP_
#define MEISTER_WEB_NAV_VISION_LISTENER_HPP_

#include <atomic>
#include <chrono>
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
  using Clock = std::chrono::steady_clock;

  void TouchClientActivity() noexcept;
  bool IsClientRecentlyActive() const noexcept;
  void ApplyEnabled();

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
  rclcpp::TimerBase::SharedPtr reconcile_timer_;
  std::atomic<bool> enabled_{true};
  std::atomic<int64_t> last_client_access_ns_{0};
  bool subscribed_ = false;
  VisionCache cache_;
};

}  // namespace meister_web_nav

#endif  // MEISTER_WEB_NAV_VISION_LISTENER_HPP_

// SPDX-License-Identifier: MIT
/// 認識画像を「ブラウザがそのまま貼れる JPEG」に直して保持する。
///
/// detection_node は 3 Hz で描画済み画像を流す。ブラウザに毎回 PNG を渡すと
/// 1 フレーム数十 KB が積み上がるので、ここで JPEG に落としてから保持し、
/// 配信側 (/api/vision.jpg と MJPEG) は同じバイト列を共有する。
///
/// ROS グラフは要らない (購読は VisionListener)。node 無しで test できる。
#ifndef MEISTER_WEB_NAV_VISION_CACHE_HPP_
#define MEISTER_WEB_NAV_VISION_CACHE_HPP_

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "meister_web_nav/ports.hpp"

namespace meister_web_nav {

/// 表示用に丸めた 1 検出。座標は画像ピクセル。
struct VisionDetection {
  std::string label;
  double score;
  double x;
  double y;
  double width;
  double height;
};

/// 古いと見なすまでの無通信時間。detection_node の既定は 3 Hz なので、その 7 倍
/// 以上あけて「一時的な取りこぼし」と「配信が止まった」を分ける。
inline constexpr int kStaleAfterMs = 2500;

class VisionCache {
 public:
  /// stale_after_ms は test が短くするための口。既定は kStaleAfterMs。
  explicit VisionCache(int stale_after_ms = kStaleAfterMs);

  /// 復号できない画像 (未知の encoding / 壊れた step) は false を返して捨てる。
  /// 途中まで読めた画像を混ぜると、どのフレームか分からない絵が UI に残る。
  bool SetFrame(const sensor_msgs::msg::Image& msg);

  void SetDetections(const vision_msgs::msg::Detection2DArray& msg);

  nlohmann::json Status() const;

  std::optional<VisionFrame> Latest(long after_seq, double timeout_s);

 private:
  using Clock = std::chrono::steady_clock;

  VisionFrame FrameLocked() const;

  mutable std::mutex mutex_;
  std::condition_variable frame_cv_;

  bool has_frame_ = false;
  long seq_ = 0;
  std::string jpeg_;
  int width_ = 0;
  int height_ = 0;
  std::string frame_id_;
  Clock::time_point frame_at_{};
  /// fps は直近の到達間隔だけで見る。起動直後の 1 フレーム目を 0 と出したいので
  /// レートの移動平均は持たない。
  std::deque<Clock::time_point> arrivals_;

  bool has_detections_ = false;
  std::size_t detection_count_ = 0;
  Clock::time_point detections_at_{};
  std::vector<VisionDetection> detections_;
  const int stale_after_ms_;
};

}  // namespace meister_web_nav

#endif  // MEISTER_WEB_NAV_VISION_CACHE_HPP_

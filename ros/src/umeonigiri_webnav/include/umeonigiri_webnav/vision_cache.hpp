// SPDX-License-Identifier: MIT
/// 認識モニター用の画像・検出キャッシュ。
/// SetFrame では BGR のまま保持し、JPEG エンコードは Latest() で実際にブラウザから
/// 取得されたときだけ遅延実行（seq_ 単位でキャッシュ）することで、未視聴時の CPU を削減する。
///
/// ROS グラフは要らない (購読は VisionListener)。node 無しで test できる。
#ifndef UMEONIGIRI_WEBNAV_VISION_CACHE_HPP_
#define UMEONIGIRI_WEBNAV_VISION_CACHE_HPP_

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "umeonigiri_webnav/ports.hpp"

namespace umeonigiri_webnav {

/// 表示用に丸めた 1 検出。座標は画像ピクセル。
struct VisionDetection {
  std::string label;
  double score;
  double x;
  double y;
  double width;
  double height;
};

/// 古いと見なすまでの無通信時間。
inline constexpr int kStaleAfterMs = 2500;

class VisionCache {
 public:
  explicit VisionCache(int stale_after_ms = kStaleAfterMs);

  /// 復号できない画像 (未知の encoding / 壊れた step) は false を返して捨てる。
  bool SetFrame(const sensor_msgs::msg::Image& msg);

  void SetDetections(const vision_msgs::msg::Detection2DArray& msg);

  nlohmann::json Status() const;

  std::optional<VisionFrame> Latest(long after_seq, double timeout_s);

 private:
  using Clock = std::chrono::steady_clock;

  mutable std::mutex mutex_;
  std::condition_variable frame_cv_;

  bool has_frame_ = false;
  long seq_ = 0;
  cv::Mat latest_bgr_;
  long jpeg_seq_ = -1;
  std::string jpeg_;
  int width_ = 0;
  int height_ = 0;
  std::string frame_id_;
  Clock::time_point frame_at_{};
  std::deque<Clock::time_point> arrivals_;

  bool has_detections_ = false;
  std::size_t detection_count_ = 0;
  Clock::time_point detections_at_{};
  std::vector<VisionDetection> detections_;
  const int stale_after_ms_;
};

}  // namespace umeonigiri_webnav

#endif  // UMEONIGIRI_WEBNAV_VISION_CACHE_HPP_

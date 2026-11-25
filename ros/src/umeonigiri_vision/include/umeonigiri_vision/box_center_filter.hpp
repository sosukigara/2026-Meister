// SPDX-License-Identifier: MIT
/// 検出枠中心の時間平滑化。80x60 の低解像度では DFL の量子化で中心が
/// 数 px 揺れる。既存 TargetSelector の α-β と同じ発想を、 publishing する
/// 全検出へ適用する（アームの狙いを変えるのではなく、揺れだけを抑える）。
#ifndef MEISTER_VISION_BOX_CENTER_FILTER_HPP_
#define MEISTER_VISION_BOX_CENTER_FILTER_HPP_

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace umeonigiri_vision {

struct SmoothedBox {
  double x0 = 0.0;
  double y0 = 0.0;
  double x1 = 0.0;
  double y1 = 0.0;
  double score = 0.0;
  int class_id = 0;
};

class BoxCenterFilter {
 public:
  explicit BoxCenterFilter(double alpha = 0.25, double max_jump_px = 12.0)
      : alpha_(std::clamp(alpha, 0.05, 1.0)), max_jump_px_(max_jump_px) {}

  /// 同じクラスの直前中心が近ければ EMA、遠ければ今回値でリセットする。
  SmoothedBox Update(const SmoothedBox & measured) {
    SmoothedBox result = measured;
    const double cx = (measured.x0 + measured.x1) * 0.5;
    const double cy = (measured.y0 + measured.y1) * 0.5;
    auto it = centers_.find(measured.class_id);
    if (it == centers_.end() ||
        std::hypot(cx - it->second.first, cy - it->second.second) > max_jump_px_) {
      centers_[measured.class_id] = {cx, cy};
      return result;
    }
    const double sx = alpha_ * cx + (1.0 - alpha_) * it->second.first;
    const double sy = alpha_ * cy + (1.0 - alpha_) * it->second.second;
    const double shift_x = sx - cx;
    const double shift_y = sy - cy;
    result.x0 += shift_x;
    result.x1 += shift_x;
    result.y0 += shift_y;
    result.y1 += shift_y;
    it->second = {sx, sy};
    return result;
  }

  void Reset() { centers_.clear(); }

 private:
  double alpha_;
  double max_jump_px_;
  std::unordered_map<int, std::pair<double, double>> centers_;
};

}  // namespace umeonigiri_vision

#endif  // MEISTER_VISION_BOX_CENTER_FILTER_HPP_

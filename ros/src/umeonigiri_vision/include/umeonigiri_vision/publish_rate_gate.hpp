// SPDX-License-Identifier: MIT
/// /detections の配信レート上限。時刻を引数で受け、テストを決定的にする。
#ifndef MEISTER_VISION_PUBLISH_RATE_GATE_HPP_
#define MEISTER_VISION_PUBLISH_RATE_GATE_HPP_

namespace umeonigiri_vision {

class PublishRateGate {
 public:
  explicit PublishRateGate(double max_hz)
      : period_s_(max_hz > 0.0 ? 1.0 / max_hz : 0.0) {}

  bool Ready(double now_seconds) {
    if (period_s_ <= 0.0) {
      return true;
    }
    if (!has_last_ || now_seconds - last_seconds_ >= period_s_) {
      last_seconds_ = now_seconds;
      has_last_ = true;
      return true;
    }
    return false;
  }

 private:
  double period_s_;
  bool has_last_ = false;
  double last_seconds_ = 0.0;
};

}  // namespace umeonigiri_vision

#endif  // MEISTER_VISION_PUBLISH_RATE_GATE_HPP_

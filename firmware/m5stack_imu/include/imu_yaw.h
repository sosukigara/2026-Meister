#pragma once

#include <stdint.h>

#include "imu_sampling.h"

namespace meister_imu {

// ジャイロ角速度を時間積分して yaw [deg] を求める。
// 加速度から得た roll/pitch で傾き補正するが、地磁気が無いため絶対方位ではない。
class YawTracker {
 public:
  void Reset();                        // yaw と時刻のみリセット（bias は維持）
  void SetBias(const float gyro[3]);
  void BiasSample(const float gyro[3]);
  void FinishBias();
  void Update(const RawImu& raw);
  // ミリ秒で刻みを与えるテスト用オーバーロード。
  void Update(const RawImu& raw, uint32_t now_ms) {
    RawImu copy = raw;
    copy.timestamp_us = now_ms * 1000u;
    Update(copy);
  }

  float yaw_deg() const { return yaw_deg_; }
  float yaw_rate_deg() const { return yaw_rate_; }

 private:
  float bias_[3] = {0.0f, 0.0f, 0.0f};
  float bias_sum_[3] = {0.0f, 0.0f, 0.0f};
  float filtered_gyro_[3] = {0.0f, 0.0f, 0.0f};
  bool filtered_ready_ = false;
  uint32_t bias_samples_ = 0;
  float yaw_deg_ = 0.0f;
  float yaw_rate_ = 0.0f;
  uint32_t last_us_ = 0;
  bool has_last_ = false;
};

}  // namespace meister_imu

#pragma once

#include <stdint.h>

#include "imu_protocol.h"
#include "imu_sampling.h"

namespace meister_imu {

struct ViewLayout {
  int32_t w = 0;
  int32_t h = 0;
  int32_t compass_x = 0;
  int32_t compass_y = 0;
  int32_t compass_w = 0;
  int32_t compass_h = 0;
  int32_t cx = 0;
  int32_t cy = 0;
  int32_t radius = 0;
  int32_t roll_x = 0;
  int32_t roll_y = 0;
  int32_t pitch_x = 0;
  int32_t mini_w = 0;
  int32_t mini_h = 0;
  int32_t motion_x = 0;
  int32_t motion_y = 0;
  int32_t motion_w = 0;
  int32_t motion_h = 0;
  int32_t trend_x = 0;
  int32_t trend_y = 0;
  int32_t trend_w = 0;
};

class ImuView {
 public:
  void Begin();
  void Draw(const protocol::Sample& sample, const RawImu& raw,
            uint32_t tx_hz, bool tx_ok);

 private:
  ViewLayout layout_;
  float yaw_history_[32] = {};
  uint8_t yaw_index_ = 0;
  uint8_t yaw_count_ = 0;
};

}  // namespace meister_imu

#pragma once

#include <stdint.h>

#include "imu_protocol.h"

namespace meister_imu {

struct RawImu {
  float accel[3] = {0.0f, 0.0f, 0.0f};  // G
  float gyro[3] = {0.0f, 0.0f, 0.0f};   // deg/s
  uint32_t timestamp_ms = 0;
  bool present = false;  // M5.Imu.update() が新鮮な値を返したか
};

struct Attitude {
  float roll_deg = 0.0f;
  float pitch_deg = 0.0f;
};

void ConvertSample(const RawImu& raw, protocol::Sample* out);

// 画面表示用の簡易姿勢。軸の向きは個体差があるため ★要確認★。
Attitude EstimateAttitude(const RawImu& raw);

}  // namespace meister_imu

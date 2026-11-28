#include "imu_sampling.h"

#include <climits>
#include <cmath>

#include "imu_config.h"

namespace meister_imu {
namespace {

int16_t ToFixed(float value, float scale, bool* clamped) {
  if (!std::isfinite(value)) {
    *clamped = true;
    return 0;
  }
  const double scaled = static_cast<double>(value) * scale;
  if (scaled > INT16_MAX) {
    *clamped = true;
    return INT16_MAX;
  }
  if (scaled < INT16_MIN) {
    *clamped = true;
    return INT16_MIN;
  }
  return static_cast<int16_t>(std::lround(scaled));
}

}  // namespace

void ConvertSample(const RawImu& raw, protocol::Sample* out) {
  out->timestamp_ms = raw.timestamp_ms;

  bool clamped = false;
  for (size_t i = 0; i < 3; ++i) {
    if (raw.present) {
      out->accel[i] = ToFixed(raw.accel[i], config::kAccelScale, &clamped);
      out->gyro[i] = ToFixed(raw.gyro[i], config::kGyroScale, &clamped);
    } else {
      out->accel[i] = 0;
      out->gyro[i] = 0;
    }
  }

  out->status = (clamped ? protocol::kClamped : 0) |
                (raw.present ? 0 : protocol::kImuMissing);
}

Attitude EstimateAttitude(const RawImu& raw) {
  Attitude attitude;
  if (!raw.present || !std::isfinite(raw.accel[0]) ||
      !std::isfinite(raw.accel[1]) || !std::isfinite(raw.accel[2])) {
    return attitude;
  }

  const float ax = raw.accel[0];
  const float ay = raw.accel[1];
  const float az = raw.accel[2];
  attitude.roll_deg = std::atan2(ay, az) * config::kRadToDeg;
  attitude.pitch_deg =
      std::atan2(-ax, std::sqrt(ay * ay + az * az)) * config::kRadToDeg;
  return attitude;
}

}  // namespace meister_imu

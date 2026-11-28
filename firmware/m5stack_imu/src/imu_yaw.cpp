#include "imu_yaw.h"

#include <cmath>

#include "imu_config.h"

namespace meister_imu {
namespace {

constexpr uint32_t kMaxBiasSamples = 4096;

}  // namespace

void YawTracker::Reset() {
  yaw_deg_ = 0.0f;
  yaw_rate_ = 0.0f;
  last_us_ = 0;
  has_last_ = false;
}

void YawTracker::SetBias(const float gyro[3]) {
  for (size_t i = 0; i < 3; ++i) {
    bias_[i] = gyro[i];
  }
}

void YawTracker::BiasSample(const float gyro[3]) {
  if (bias_samples_ >= kMaxBiasSamples) {
    return;
  }
  for (size_t i = 0; i < 3; ++i) {
    bias_sum_[i] += gyro[i];
  }
  ++bias_samples_;
}

void YawTracker::FinishBias() {
  if (bias_samples_ == 0) {
    return;
  }
  for (size_t i = 0; i < 3; ++i) {
    bias_[i] = bias_sum_[i] / static_cast<float>(bias_samples_);
    bias_sum_[i] = 0.0f;
  }
  bias_samples_ = 0;
}

void YawTracker::Update(const RawImu& raw) {
  if (!raw.present) {
    return;
  }
  const uint32_t now_us = raw.timestamp_us;
  if (!has_last_) {
    last_us_ = now_us;
    has_last_ = true;
    return;
  }

  const uint32_t elapsed_us = now_us - last_us_;
  last_us_ = now_us;
  if (elapsed_us == 0) {
    return;
  }
  float dt = static_cast<float>(elapsed_us) / 1000000.0f;
  if (dt > config::kYawMaxDtS) {
    dt = config::kYawMaxDtS;
  }

  const float ax = raw.accel[0];
  const float ay = raw.accel[1];
  const float az = raw.accel[2];
  if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az) ||
      !std::isfinite(raw.gyro[0]) || !std::isfinite(raw.gyro[1]) ||
      !std::isfinite(raw.gyro[2])) {
    return;
  }

  const float roll = std::atan2(ay, az);
  const float pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
  float cos_pitch = std::cos(pitch);
  if (cos_pitch < config::kYawMinCosPitch) {
    cos_pitch = config::kYawMinCosPitch;
  }

  // 静止している間だけ零点をゆっくり追従させ、温度ドリフトを抑える。
  const float magnitude = std::sqrt(ax * ax + ay * ay + az * az);
  if (std::fabs(magnitude - 1.0f) < 0.05f &&
      std::fabs(raw.gyro[0]) < config::kYawStillGyroThreshold &&
      std::fabs(raw.gyro[1]) < config::kYawStillGyroThreshold &&
      std::fabs(raw.gyro[2]) < config::kYawStillGyroThreshold) {
    for (size_t i = 0; i < 3; ++i) {
      bias_[i] += config::kYawBiasAdaptRate * (raw.gyro[i] - bias_[i]);
    }
  }

  const float gx = raw.gyro[0] - bias_[0];
  const float gy = raw.gyro[1] - bias_[1];
  const float gz = raw.gyro[2] - bias_[2];
  yaw_rate_ = (gy * std::sin(roll) + gz * std::cos(roll)) / cos_pitch;
  yaw_deg_ += yaw_rate_ * dt;

  yaw_deg_ = std::fmod(yaw_deg_, 360.0f);
  if (yaw_deg_ < 0.0f) {
    yaw_deg_ += 360.0f;
  }
}

}  // namespace meister_imu

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

  for (size_t i = 0; i < 3; ++i) {
    if (!filtered_ready_) {
      filtered_accel_[i] = raw.accel[i];
    } else {
      filtered_accel_[i] += config::kYawAccelLowpassAlpha *
                            (raw.accel[i] - filtered_accel_[i]);
    }
  }
  const float ax = filtered_accel_[0];
  const float ay = filtered_accel_[1];
  const float az = filtered_accel_[2];
  if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az) ||
      !std::isfinite(raw.gyro[0]) || !std::isfinite(raw.gyro[1]) ||
      !std::isfinite(raw.gyro[2])) {
    return;
  }

  for (size_t i = 0; i < 3; ++i) {
    if (!filtered_ready_) {
      filtered_gyro_[i] = raw.gyro[i];
    } else {
      filtered_gyro_[i] += config::kYawGyroLowpassAlpha *
                           (raw.gyro[i] - filtered_gyro_[i]);
    }
  }
  filtered_ready_ = true;

  // 静止判定にはノイズ除去後の値を使う。
  const float magnitude = std::sqrt(ax * ax + ay * ay + az * az);
  const bool still =
      std::fabs(magnitude - 1.0f) < 0.05f &&
      std::fabs(filtered_gyro_[0]) < config::kYawStillGyroThreshold &&
      std::fabs(filtered_gyro_[1]) < config::kYawStillGyroThreshold &&
      std::fabs(filtered_gyro_[2]) < config::kYawStillGyroThreshold;

  if (still) {
    // 静止時は零点を追従させ、残留角速度を積分しない（ZUPT）。
    for (size_t i = 0; i < 3; ++i) {
      bias_[i] += config::kYawBiasAdaptRate * (filtered_gyro_[i] - bias_[i]);
    }
    yaw_rate_ = 0.0f;
    return;
  }

  // 直線加速度が混ざると傾き補正が壊れるため、重力とみなせる時だけ更新する。
  float roll = last_roll_;
  float pitch = last_pitch_;
  if (std::fabs(magnitude - 1.0f) <= config::kYawAccelValidTolerance) {
    roll = std::atan2(ay, az);
    pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
    last_roll_ = roll;
    last_pitch_ = pitch;
  }
  float cos_pitch = std::cos(pitch);
  if (cos_pitch < config::kYawMinCosPitch) {
    cos_pitch = config::kYawMinCosPitch;
  }

  const float gy = filtered_gyro_[1] - bias_[1];
  const float gz = filtered_gyro_[2] - bias_[2];
  yaw_rate_ = (gy * std::sin(roll) + gz * std::cos(roll)) / cos_pitch;
  yaw_deg_ += yaw_rate_ * dt;

  yaw_deg_ = std::fmod(yaw_deg_, 360.0f);
  if (yaw_deg_ < 0.0f) {
    yaw_deg_ += 360.0f;
  }
}

}  // namespace meister_imu

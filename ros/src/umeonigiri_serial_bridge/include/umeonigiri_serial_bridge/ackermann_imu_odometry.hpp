#pragma once

#include <algorithm>
#include <cmath>

#include "umeonigiri_serial_bridge/serial_io.hpp"

namespace umeonigiri_serial_bridge {

struct OdometryConfig {
  double wheelbase_m = 0.30;
  double max_steering_deg = 30.0;
  double imu_yaw_fusion_gain = 0.85;  // Innovation weight for BNO055 heading
  double cmd_timeout_s = 0.50;
  double pose_cov_xy = 0.0025;        // [m^2]
  double pose_cov_yaw = 0.0015;       // [rad^2]
  double twist_cov_vx = 0.0040;       // [(m/s)^2]
  double twist_cov_wz = 0.0010;       // [(rad/s)^2]
};

struct OdometryState {
  double x = 0.0;     // [m] in odom frame
  double y = 0.0;     // [m] in odom frame
  double yaw = 0.0;   // [rad] in [-pi, +pi]
  double vx = 0.0;    // [m/s] in base_footprint frame
  double wz = 0.0;    // [rad/s] in base_footprint frame
  double qz = 0.0;    // sin(yaw / 2)
  double qw = 1.0;    // cos(yaw / 2)
};

class AckermannImuOdometry {
 public:
  explicit AckermannImuOdometry(const OdometryConfig& cfg = OdometryConfig{})
      : cfg_(cfg) {}

  void UpdateTwistCommand(double vx, double wz, double now_s) noexcept {
    cmd_vx_ = vx;
    const double max_steer_rad = cfg_.max_steering_deg * (M_PI / 180.0);
    const double max_kappa = std::tan(max_steer_rad) / std::max(cfg_.wheelbase_m, 1e-3);
    if (std::fabs(vx) >= 0.01) {
      const double max_wz = std::fabs(vx) * max_kappa;
      cmd_wz_ = std::clamp(wz, -max_wz, +max_wz);
    } else {
      // Non-holonomic Ackermann cannot spin in place when vx == 0
      cmd_wz_ = 0.0;
    }
    last_cmd_time_s_ = now_s;
  }

  void UpdateImuReport(const ImuReport& imu, double now_s) noexcept {
    const YawQuat q = YawTenthsToQuat(imu.yaw_tenths);
    const double raw_yaw = 2.0 * std::atan2(q.z, q.w);

    if (!has_imu_) {
      imu_yaw_offset_ = WrapAngle(raw_yaw - state_.yaw);
      has_imu_ = true;
      last_imu_yaw_ = state_.yaw;
      last_imu_time_s_ = now_s;
      return;
    }

    const double new_imu_yaw = WrapAngle(raw_yaw - imu_yaw_offset_);
    if (last_imu_time_s_ > 0.0 && (now_s - last_imu_time_s_) > 1e-3 &&
        (now_s - last_imu_time_s_) < 0.5) {
      const double dyaw = WrapAngle(new_imu_yaw - last_imu_yaw_);
      imu_wz_ = 0.5 * imu_wz_ + 0.5 * (dyaw / (now_s - last_imu_time_s_));
    }
    last_imu_yaw_ = new_imu_yaw;
    last_imu_time_s_ = now_s;
  }

  const OdometryState& Step(double now_s) noexcept {
    if (last_step_time_s_ < 0.0) {
      last_step_time_s_ = now_s;
      return state_;
    }

    const double dt = now_s - last_step_time_s_;
    last_step_time_s_ = now_s;
    if (dt <= 0.0 || dt > 1.0) {
      return state_;
    }

    double vx = cmd_vx_;
    double wz = cmd_wz_;
    if (last_cmd_time_s_ < 0.0 || (now_s - last_cmd_time_s_) > cfg_.cmd_timeout_s) {
      vx = 0.0;
      wz = 0.0;
    }

    const double pred_yaw = WrapAngle(state_.yaw + wz * dt);
    double next_yaw = pred_yaw;
    if (has_imu_ && last_imu_time_s_ > 0.0 && (now_s - last_imu_time_s_) < 0.5) {
      const double innovation = WrapAngle(last_imu_yaw_ - pred_yaw);
      next_yaw = WrapAngle(pred_yaw + cfg_.imu_yaw_fusion_gain * innovation);
      wz = 0.3 * wz + 0.7 * imu_wz_;
    }

    // 旋回時の弦長と弧長の差を sinc(dyaw/2) で補正する厳密円弧積分。
    // 小角ではテイラー展開を使い 0 除算と桁落ちを避ける。
    const double dyaw = WrapAngle(next_yaw - state_.yaw);
    const double half = 0.5 * dyaw;
    const double half_sq = half * half;
    const double sinc_half =
        std::fabs(half) < 1e-4
            ? (1.0 - half_sq / 6.0 + (half_sq * half_sq) / 120.0)
            : (std::sin(half) / half);
    const double mid_yaw = state_.yaw + half;
    const double chord = vx * dt * sinc_half;

    state_.x += chord * std::cos(mid_yaw);
    state_.y += chord * std::sin(mid_yaw);
    state_.yaw = next_yaw;
    state_.vx = vx;
    state_.wz = wz;
    state_.qz = std::sin(0.5 * state_.yaw);
    state_.qw = std::cos(0.5 * state_.yaw);
    return state_;
  }

  const OdometryState& state() const noexcept { return state_; }
  const OdometryConfig& config() const noexcept { return cfg_; }

  static double WrapAngle(double rad) noexcept {
    // atan2(sin, cos) は超越関数を 3 回呼ぶため、IEEE 754 remainder で高速・高精度に正規化する。
    return std::remainder(rad, 2.0 * M_PI);
  }

 private:
  OdometryConfig cfg_;
  OdometryState state_{};
  bool has_imu_ = false;
  double imu_yaw_offset_ = 0.0;
  double last_imu_yaw_ = 0.0;
  double last_imu_time_s_ = -1.0;
  double imu_wz_ = 0.0;
  double cmd_vx_ = 0.0;
  double cmd_wz_ = 0.0;
  double last_cmd_time_s_ = -1.0;
  double last_step_time_s_ = -1.0;
};

}  // namespace umeonigiri_serial_bridge

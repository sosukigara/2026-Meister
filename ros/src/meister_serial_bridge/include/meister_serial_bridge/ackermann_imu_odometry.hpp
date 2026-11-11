#pragma once
// AckermannImuOdometry — 50 Hz SE(2) Complementary / Kalman Odometry Estimator
// for the Meister 6-Wheel Steering Ackermann Robot.
//
// Fuses Ackermann kinematic velocity/steering commands with BNO055 IMU heading
// (`FB_IMU` yaw_tenths & calibration status) to provide continuous, drift-free
// `odom -> base_footprint` TF and `/odom` messages without external IPC latency.

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "meister_serial_bridge/serial_io.hpp"

namespace meister_serial_bridge {

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

  void Reset() noexcept {
    state_ = OdometryState{};
    has_imu_ = false;
    imu_yaw_offset_ = 0.0;
    last_imu_yaw_ = 0.0;
    last_imu_time_s_ = -1.0;
    imu_wz_ = 0.0;
    cmd_vx_ = 0.0;
    cmd_wz_ = 0.0;
    last_cmd_time_s_ = -1.0;
    last_step_time_s_ = -1.0;
  }

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

    // Predict yaw via 2nd-order Runge-Kutta
    const double pred_yaw = WrapAngle(state_.yaw + wz * dt);
    double next_yaw = pred_yaw;
    if (has_imu_ && last_imu_time_s_ > 0.0 && (now_s - last_imu_time_s_) < 0.5) {
      const double innovation = WrapAngle(last_imu_yaw_ - pred_yaw);
      next_yaw = WrapAngle(pred_yaw + cfg_.imu_yaw_fusion_gain * innovation);
      wz = 0.3 * wz + 0.7 * imu_wz_;
    }

    const double mid_yaw = WrapAngle(state_.yaw + 0.5 * WrapAngle(next_yaw - state_.yaw));
    state_.x += vx * std::cos(mid_yaw) * dt;
    state_.y += vx * std::sin(mid_yaw) * dt;
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
    return std::atan2(std::sin(rad), std::cos(rad));
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

}  // namespace meister_serial_bridge

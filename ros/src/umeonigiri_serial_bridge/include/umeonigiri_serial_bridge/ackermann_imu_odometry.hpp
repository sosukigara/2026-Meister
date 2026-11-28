#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "umeonigiri_serial_bridge/serial_io.hpp"

namespace umeonigiri_serial_bridge {

struct OdometryConfig {
  double wheelbase_m = 0.60;             // 6輪前後ステア間距離の既定値（YAML/launchで上書き）
  double max_steering_deg = 30.0;
  double imu_yaw_fusion_gain = 0.85;     // レガシー絶対ヨー融合時のゲイン
  double imu_wz_fusion_gain = 0.70;      // 差分角速度融合ゲイン（磁気外乱対策の既定パス）
  bool imu_use_differential_yaw = false;  // true: BNO055絶対ヨーを直接引かず差分角速度のみ使用
  bool imu_require_gyro_calib = false;   // true: CALIB_STAT の gyro bits [5:4] >= imu_min_gyro_calib を要求
  uint8_t imu_min_gyro_calib = 1;
  double imu_max_jump_rad_per_s = 3.5;        // 磁気ジャンプ/再キャリブによる不連続跳躍の棄却閾値
  double imu_max_innovation_rad_per_s = 1.2;  // 運動学予測角速度との乖離棄却ゲート
  bool use_encoder_feedback = false;          // true: FB_STATE の実車輪速度を優先使用
  double encoder_units_to_mps = 0.001;        // encoders 1単位 -> m/s 変換係数（例: mm/s -> 0.001）
  double encoder_timeout_s = 0.25;
  double cmd_timeout_s = 0.50;
  double pose_cov_xy = 0.0025;           // [m^2]
  double pose_cov_yaw = 0.0015;          // [rad^2]
  double twist_cov_vx = 0.0040;          // [(m/s)^2]
  double twist_cov_wz = 0.0010;          // [(rad/s)^2]
};

struct OdometryState {
  double x = 0.0;     // [m] in odom frame
  double y = 0.0;     // [m] in odom frame
  double yaw = 0.0;   // [rad] in [-pi, +pi]
  double vx = 0.0;    // [m/s] in base_footprint frame
  double wz = 0.0;    // [rad/s] in base_footprint frame
  double qz = 0.0;    // sin(yaw / 2)
  double qw = 1.0;    // cos(yaw / 2)
  bool using_encoder_feedback = false;
  uint32_t imu_rejected_jumps = 0;
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
      // 非ホロノミック（中間2輪固定の前後4WS）は vx == 0 でその場旋回しない
      cmd_wz_ = 0.0;
    }
    last_cmd_time_s_ = now_s;
  }

  /// FB_STATE の 6 輪エンコーダ実測値から並進速度を更新する。
  /// use_encoder_feedback=false またはセンサエラー時は無視して指令値フォールバックを維持する。
  void UpdateWheelFeedback(const Feedback& fb, double now_s) noexcept {
    if (!cfg_.use_encoder_feedback) {
      return;
    }
    if ((fb.error_flags & meister::proto::kFbErrorSensor) != 0) {
      return;
    }
    if (fb.encoders.empty()) {
      return;
    }
    double sum = 0.0;
    for (int16_t v : fb.encoders) {
      sum += static_cast<double>(v);
    }
    enc_vx_ = (sum / static_cast<double>(fb.encoders.size())) * cfg_.encoder_units_to_mps;
    has_enc_ = true;
    last_enc_time_s_ = now_s;
  }

  void UpdateImuReport(const ImuReport& imu, double now_s) noexcept {
    if (cfg_.imu_require_gyro_calib) {
      const uint8_t gyro_calib = static_cast<uint8_t>((imu.calib >> 4) & 0x03U);
      if (gyro_calib < cfg_.imu_min_gyro_calib) {
        return;
      }
    }

    const YawQuat q = YawTenthsToQuat(imu.yaw_tenths);
    const double raw_yaw = 2.0 * std::atan2(q.z, q.w);

    if (!has_imu_) {
      imu_yaw_offset_ = WrapAngle(raw_yaw - state_.yaw);
      has_imu_ = true;
      last_imu_yaw_ = state_.yaw;
      last_raw_yaw_ = raw_yaw;
      last_imu_time_s_ = now_s;
      return;
    }

    const double dt = now_s - last_imu_time_s_;
    if (dt > 1e-3 && dt < 0.5) {
      const double dyaw = WrapAngle(raw_yaw - last_raw_yaw_);
      const double raw_wz = dyaw / dt;
      if (cfg_.imu_use_differential_yaw && std::fabs(raw_wz) > cfg_.imu_max_jump_rad_per_s) {
        // BNO055 NDOF の磁気外乱・自動再キャリブによる不連続ジャンプを棄却する。
        // 基準角だけ更新し、角速度推定やオドメトリ姿勢を壊さない。
        ++state_.imu_rejected_jumps;
        imu_yaw_offset_ = WrapAngle(raw_yaw - state_.yaw);
        last_raw_yaw_ = raw_yaw;
        last_imu_yaw_ = state_.yaw;
        last_imu_time_s_ = now_s;
        return;
      }
      imu_wz_ = 0.5 * imu_wz_ + 0.5 * raw_wz;
    }

    last_raw_yaw_ = raw_yaw;
    last_imu_yaw_ = WrapAngle(raw_yaw - imu_yaw_offset_);
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

    const bool enc_fresh = cfg_.use_encoder_feedback && has_enc_ &&
                           last_enc_time_s_ > 0.0 &&
                           (now_s - last_enc_time_s_) <= cfg_.encoder_timeout_s;
    const bool cmd_fresh =
        last_cmd_time_s_ >= 0.0 && (now_s - last_cmd_time_s_) <= cfg_.cmd_timeout_s;

    double vx = 0.0;
    double wz = 0.0;
    state_.using_encoder_feedback = enc_fresh;

    if (enc_fresh) {
      vx = enc_vx_;
      if (cmd_fresh && std::fabs(vx) >= 0.01) {
        const double max_steer_rad = cfg_.max_steering_deg * (M_PI / 180.0);
        const double max_kappa = std::tan(max_steer_rad) / std::max(cfg_.wheelbase_m, 1e-3);
        const double max_wz = std::fabs(vx) * max_kappa;
        wz = std::clamp(cmd_wz_, -max_wz, +max_wz);
      }
    } else if (cmd_fresh) {
      vx = cmd_vx_;
      wz = cmd_wz_;
    }

    const double pred_yaw = WrapAngle(state_.yaw + wz * dt);
    double next_yaw = pred_yaw;
    const bool imu_fresh =
        has_imu_ && last_imu_time_s_ > 0.0 && (now_s - last_imu_time_s_) < 0.5;

    if (imu_fresh) {
      if (cfg_.imu_use_differential_yaw) {
        // 磁気外乱を受けやすい絶対ヨーは直接引かず、イノベーションゲートを通過した
        // 短期角速度のみを融合する（長期ドリフトは 10Hz LiDAR 側で補正）。
        if (std::fabs(imu_wz_ - wz) <= cfg_.imu_max_innovation_rad_per_s) {
          const double alpha = std::clamp(cfg_.imu_wz_fusion_gain, 0.0, 1.0);
          wz = (1.0 - alpha) * wz + alpha * imu_wz_;
          next_yaw = WrapAngle(state_.yaw + wz * dt);
        }
      } else {
        const double innovation = WrapAngle(last_imu_yaw_ - pred_yaw);
        next_yaw = WrapAngle(pred_yaw + cfg_.imu_yaw_fusion_gain * innovation);
        wz = 0.3 * wz + 0.7 * imu_wz_;
      }
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
  double last_raw_yaw_ = 0.0;
  double last_imu_yaw_ = 0.0;
  double last_imu_time_s_ = -1.0;
  double imu_wz_ = 0.0;
  bool has_enc_ = false;
  double enc_vx_ = 0.0;
  double last_enc_time_s_ = -1.0;
  double cmd_vx_ = 0.0;
  double cmd_wz_ = 0.0;
  double last_cmd_time_s_ = -1.0;
  double last_step_time_s_ = -1.0;
};

}  // namespace umeonigiri_serial_bridge

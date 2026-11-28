#include "umeonigiri_serial_bridge/kinematics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#include "meister_protocol.h"

namespace umeonigiri_serial_bridge {
namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

/// 0.5 の丸めを Python の round() と揃える（偶数丸め）。
int RoundHalfToEven(double v) {
  return static_cast<int>(std::nearbyint(v));
}

struct WheelGeom {
  double x;
  double y;
  bool steerable;
};

}  // namespace

ActuatorCommand TwistToActuators(double vx, double wz, const TwistKinematics& kin) {
  if (kin.max_linear_vel_mps <= 0.0) {
    throw std::invalid_argument("max_linear_vel must be positive");
  }
  if (kin.max_steering_deg <= 0.0) {
    throw std::invalid_argument("max_steering_deg must be positive");
  }

  const int max_steer_tenths = RoundHalfToEven(kin.max_steering_deg * 10.0);

  int vel_permille = RoundHalfToEven(vx / kin.max_linear_vel_mps * 1000.0);
  vel_permille = std::clamp<int>(vel_permille, meister::proto::kMinVelocity,
                                 meister::proto::kMaxVelocity);
  if (kin.drive_invert) {
    vel_permille = -vel_permille;
  }

  int steer_tenths = 0;
  if (std::fabs(vx) >= kEpsilonVx) {
    steer_tenths = RoundHalfToEven(
        std::atan((kin.wheelbase_m * wz) / vx) * kRadToDeg * 10.0);
  } else if (wz > 0.0) {
    steer_tenths = max_steer_tenths;
  } else if (wz < 0.0) {
    steer_tenths = -max_steer_tenths;
  }
  steer_tenths = std::clamp(steer_tenths, -max_steer_tenths, max_steer_tenths);
  if (kin.steer_invert) {
    steer_tenths = -steer_tenths;
  }

  ActuatorCommand out;
  out.steering_tenths = static_cast<int16_t>(steer_tenths);
  out.velocity_permille = static_cast<int16_t>(vel_permille);
  return out;
}

SixWheelCommand TwistToSixWheelActuators(double vx, double wz, const TwistKinematics& kin) {
  if (kin.max_linear_vel_mps <= 0.0) {
    throw std::invalid_argument("max_linear_vel must be positive");
  }
  if (kin.max_steering_deg <= 0.0) {
    throw std::invalid_argument("max_steering_deg must be positive");
  }

  SixWheelCommand out{};
  const int max_steer_tenths = RoundHalfToEven(kin.max_steering_deg * 10.0);
  const double half_l = 0.5 * std::max(kin.wheelbase_m, 1e-3);
  const double half_w = 0.5 * std::max(kin.track_width_m, 0.0);

  // 順序: [0]FL, [1]FR, [2]ML(固定), [3]MR(固定), [4]RL, [5]RR
  const std::array<WheelGeom, 6> wheels = {{
      {+half_l, +half_w, true},
      {+half_l, -half_w, true},
      {0.0, +half_w, false},
      {0.0, -half_w, false},
      {-half_l, +half_w, true},
      {-half_l, -half_w, true},
  }};

  // ほぼ停止時の旋回要求: 駆動 0、前後 4 輪を逆位相最大舵角へ予角する
  if (std::fabs(vx) < kEpsilonVx) {
    if (std::fabs(wz) > 1e-6) {
      const int sign = wz > 0.0 ? 1 : -1;
      for (std::size_t i = 0; i < 6 && i < kSteerChannels; ++i) {
        if (!wheels[i].steerable) {
          out.steering_tenths[i] = 0;
          continue;
        }
        int angle = (wheels[i].x >= 0.0 ? sign : -sign) * max_steer_tenths;
        if (kin.steer_invert) {
          angle = -angle;
        }
        out.steering_tenths[i] = static_cast<int16_t>(angle);
      }
    }
    return out;
  }

  // 直進（曲率ほぼゼロ）: 全輪 0 度・等速
  if (std::fabs(wz) < 1e-6) {
    int vel = RoundHalfToEven(vx / kin.max_linear_vel_mps * 1000.0);
    vel = std::clamp<int>(vel, meister::proto::kMinVelocity, meister::proto::kMaxVelocity);
    if (kin.drive_invert) {
      vel = -vel;
    }
    out.velocity_permille.fill(static_cast<int16_t>(vel));
    return out;
  }

  // 瞬間旋回中心 ICR = (0, R), R = vx / wz
  // ICR が車体トレッド内に入ると内輪が反転するため最小旋回半径でガードする。
  double R = vx / wz;
  const double min_r = half_w + 0.05;
  if (std::fabs(R) < min_r) {
    R = (R >= 0.0) ? min_r : -min_r;
  }

  std::array<double, 6> wheel_v{};
  double peak_v = 0.0;

  for (std::size_t i = 0; i < 6; ++i) {
    const double dx = wheels[i].x;
    const double dy = R - wheels[i].y;  // ICR から車輪までの横距離（符号付き）

    // 前輪 (dx > 0) と後輪 (dx < 0) で自動的に逆位相・内外輪差（アッカーマン角）になる。
    // std::atan(dx / dy) は前進・後退・左右旋回すべてで [-90, +90] 度に連続収束する。
    if (wheels[i].steerable && i < kSteerChannels) {
      const double delta_rad = std::atan(dx / dy);
      int tenths = RoundHalfToEven(delta_rad * kRadToDeg * 10.0);
      tenths = std::clamp(tenths, -max_steer_tenths, max_steer_tenths);
      if (kin.steer_invert) {
        tenths = -tenths;
      }
      out.steering_tenths[i] = static_cast<int16_t>(tenths);
    }

    // 厳密転がり速度: v_i = vx * hypot(dx, dy) / |R|
    const double v_i = vx * (std::hypot(dx, dy) / std::fabs(R));
    wheel_v[i] = v_i;
    peak_v = std::max(peak_v, std::fabs(v_i));
  }

  // いずれかの外輪が上限を超える場合、全 6 輪を等比スケールして旋回曲率を維持する
  const double scale = (peak_v > kin.max_linear_vel_mps)
                           ? (kin.max_linear_vel_mps / peak_v)
                           : 1.0;

  for (std::size_t i = 0; i < 6 && i < kDriveChannels; ++i) {
    int permille = RoundHalfToEven((wheel_v[i] * scale) / kin.max_linear_vel_mps * 1000.0);
    permille = std::clamp<int>(permille, meister::proto::kMinVelocity, meister::proto::kMaxVelocity);
    if (kin.drive_invert) {
      permille = -permille;
    }
    out.velocity_permille[i] = static_cast<int16_t>(permille);
  }

  return out;
}

}  // namespace umeonigiri_serial_bridge

#include "meister_serial_bridge/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "meister_protocol.h"

namespace meister_serial_bridge {
namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

/// 0.5 の丸めを Python の round() と揃える。Python は偶数側へ丸めるので、
/// 0 から遠い側へ丸める std::round ではなく nearbyint（FE_TONEAREST）を使う。
int RoundHalfToEven(double v) {
  return static_cast<int>(std::nearbyint(v));
}

int ClampInt(int v, int lo, int hi) {
  return std::max(lo, std::min(hi, v));
}

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
  vel_permille = ClampInt(vel_permille, meister::proto::kMinVelocity,
                          meister::proto::kMaxVelocity);
  if (kin.drive_invert) {
    vel_permille = -vel_permille;
  }

  // 後退時 (vx < 0) の舵角反転バグ修正:
  // 旧実装の std::atan2(L * wz, vx) は vx < 0, wz == 0（直進後退）のとき +pi (+180度) を返し、
  // 舵角が最大の +300 (+30.0度) に張り付いて直進後退が不可能になっていた。
  // 自転車モデルの曲率 kappa = wz / vx に基づき、std::atan(L * wz / vx) を用いることで
  // 前進・後退の双方で [-90度, +90度] 内の連続かつ正確な舵角を算出する。
  int steer_tenths = 0;
  if (std::fabs(vx) >= kEpsilonVx) {
    steer_tenths = RoundHalfToEven(
        std::atan((kin.wheelbase_m * wz) / vx) * kRadToDeg * 10.0);
  } else if (wz > 0.0) {
    steer_tenths = max_steer_tenths;
  } else if (wz < 0.0) {
    steer_tenths = -max_steer_tenths;
  }
  steer_tenths = ClampInt(steer_tenths, -max_steer_tenths, max_steer_tenths);
  if (kin.steer_invert) {
    steer_tenths = -steer_tenths;
  }

  ActuatorCommand out;
  out.steering_tenths = static_cast<int16_t>(steer_tenths);
  out.velocity_permille = static_cast<int16_t>(vel_permille);
  return out;
}

}  // namespace meister_serial_bridge

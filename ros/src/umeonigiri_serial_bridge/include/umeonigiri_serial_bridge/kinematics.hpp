#pragma once

#include <array>
#include <cstdint>

#include "umeonigiri_serial_bridge/frames.hpp"

namespace umeonigiri_serial_bridge {

/// 自転車モデルおよび 6 輪前後 4WS アッカーマンモデルのパラメータ。
struct TwistKinematics {
  double wheelbase_m = 0.30;         // 前後ステア車軸間距離 [m]（実機は launch から 0.60 を渡す）
  double track_width_m = 0.50;       // 左右トレッド幅 [m]（y = ±0.25）
  double max_linear_vel_mps = 1.0;
  double max_steering_deg = 30.0;
  bool steer_invert = false;
  bool drive_invert = false;
  bool use_4ws_ackermann = false;    // true: 前後逆位相・内外輪差・6輪個別差速を算出
};

/// 6 輪共通の指令値（従来互換）。
struct ActuatorCommand {
  int16_t steering_tenths = 0;   ///< 舵角 (0.1 度)。正 = 左旋回
  int16_t velocity_permille = 0; ///< 速度 (千分率)。正 = 前進
};

/// 6 輪個別の指令値（順序: FL, FR, ML, MR, RL, RR）。
struct SixWheelCommand {
  SteeringArray steering_tenths{};
  VelocityArray velocity_permille{};
};

/// |vx| がこれ (m/s) 未満なら「ほぼ停止」とみなす。
constexpr double kEpsilonVx = 0.01;

/// Twist (vx, wz) を全輪共通の舵角・速度に変換する（従来互換）。
ActuatorCommand TwistToActuators(double vx, double wz, const TwistKinematics& kin);

/// Twist (vx, wz) を 6 輪（前後 4 輪ステア・中間 2 輪固定）の個別舵角・個別速度に変換する。
/// 前後逆位相かつアッカーマン内外輪差・左右差速を幾何学的に厳密解出し、
/// 最大輪速が上限を超える場合は全 6 輪を等比スケールして曲率を保つ。
SixWheelCommand TwistToSixWheelActuators(double vx, double wz, const TwistKinematics& kin);

}  // namespace umeonigiri_serial_bridge

#pragma once

#include <cstdint>

namespace meister_serial_bridge {

/// 自転車モデルのパラメータ。ROS パラメータの値をそのまま受ける。
struct TwistKinematics {
  double wheelbase_m = 0.30;
  double max_linear_vel_mps = 1.0;
  double max_steering_deg = 30.0;
  bool steer_invert = false;
  bool drive_invert = false;
};

/// 6 輪共通の指令値。舵角も駆動も全チャネルに同じ値を流す。
struct ActuatorCommand {
  int16_t steering_tenths = 0;   ///< 舵角 (0.1 度)。正 = 左旋回
  int16_t velocity_permille = 0; ///< 速度 (千分率)。正 = 前進
};

/// |vx| がこれ (m/s) 未満なら「ほぼ停止」とみなす。
///
/// アッカーマン系はその場旋回できないので、停止中の旋回指示は最大舵角と
/// 前進 0 に落とす。
constexpr double kEpsilonVx = 0.01;

/// Twist (vx, wz) を舵角と速度のコマンド値に変換する。
///
///   steering_tenths  = atan2(wheelbase * wz, vx) を度にして 10 倍
///   velocity_permille = vx / max_linear_vel * 1000
///
/// max_linear_vel と max_steering_deg が 0 以下のときは std::invalid_argument。
ActuatorCommand TwistToActuators(double vx, double wz, const TwistKinematics& kin);

}  // namespace meister_serial_bridge

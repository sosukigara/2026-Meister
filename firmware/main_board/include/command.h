/*
 * command.h — ロボットの指令モデル
 *
 * ロボットの「意思」を 1 つの型で表す。種別ごとのフレームは持たない。
 * 値は hal/generated_config.h（config/meister_robot.yaml から生成）を使う。
 *
 * Arduino 非依存。pio test -e native で検証できる。
 */
#pragma once

#include <stdint.h>

#include "hal/generated_config.h"

namespace meister {

/// 車体の目標速度。PC から送る唯一の基底指令。
///
/// vx / vy は mm/s、wz は 0.1 度/s で表現する（mm 単位だと 32 bit に
/// 収まらないので、送る側と受ける側で同じ縮尺を使う）。
struct TwistCommand {
  int16_t vx = 0;
  int16_t vy = 0;
  int16_t wz = 0;

  static constexpr int16_t kMinValue = -30000;
  static constexpr int16_t kMaxValue = 30000;
};

/// 1 輪の指令。角度はすべて 0.1 度、速度は千分率。
struct WheelCommand {
  int16_t velocity = 0;  // -1000..1000
  int16_t steering = 0;  // -900..900
  bool steerable = true;  // ボギー位置によって舵角が変わるか
};

/// ボギー軸の指令。位置表の添字ではなく角度を持つ。
struct RockerCommand {
  int16_t angle[2] = {0, 0};  // 前 / 後
};

/// アームの指令。角度はすべて 0.1 度。
///
/// アームは 4 軸: 肩 / 肘 / 手首 / グリッパー。BOM のサーボは 6 個だが、
/// 開閉①②はグリッパーの 1 軸で動かす。肘・手首・グリッパーは STS3215 バス。
/// 肩の DS サーボ 150kg は PWM。
struct ArmCommand {
  int16_t shoulder_pitch = 0;  // 肩（DS サーボ 150kg、PWM）
  int16_t elbow = 0;  // 肘（STS3215 バス）
  int16_t wrist = 0;  // 手首（STS3215 バス）
  int16_t gripper = 0;  // 開閉①②を 1 軸（STS3215 バス 2 台）
  bool valid = false;  // 指令が来たときだけ true
};

/// アームの軸数。protocol の arm_axial 数と一致させる。
static constexpr uint8_t kArmAxisCount = 4;

/// 機構に適用する 1 回分の指令。
struct RobotCommand {
  TwistCommand base;
  WheelCommand wheels[config::kNumDriveMotors];
  RockerCommand rocker;
  ArmCommand arm;
};

/// 値を範囲に収める。範囲外は 0 にせず端に丸める。
/// 範囲外を 0 にしてしまうと「停止した命令を出した」と誤解される。
inline int16_t Clamp(int16_t v, int16_t lo, int16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

inline TwistCommand ClampTwist(const TwistCommand& t) {
  return TwistCommand{
      Clamp(t.vx, TwistCommand::kMinValue, TwistCommand::kMaxValue),
      Clamp(t.vy, TwistCommand::kMinValue, TwistCommand::kMaxValue),
      Clamp(t.wz, TwistCommand::kMinValue, TwistCommand::kMaxValue),
  };
}

inline ArmCommand ClampArm(const ArmCommand& a) {
  ArmCommand out = a;
  out.shoulder_pitch = Clamp(a.shoulder_pitch, config::kMinShoulder, config::kMaxShoulder);
  out.elbow = Clamp(a.elbow, config::kMinJoint, config::kMaxJoint);
  out.wrist = Clamp(a.wrist, config::kMinJoint, config::kMaxJoint);
  out.gripper = Clamp(a.gripper, config::kMinGripper, config::kMaxGripper);
  return out;
}

inline RobotCommand ClampCommand(const RobotCommand& c) {
  RobotCommand out = c;
  out.base = ClampTwist(c.base);
  for (uint8_t i = 0; i < config::kNumDriveMotors; ++i) {
    out.wheels[i].velocity = Clamp(c.wheels[i].velocity, config::kMinVelocity, config::kMaxVelocity);
    out.wheels[i].steering = Clamp(c.wheels[i].steering, config::kMinSteering, config::kMaxSteering);
  }
  out.arm = ClampArm(c.arm);
  return out;
}

/// vy を扱えるか。6 輪ロッカーボギーには横方向の自由度がない。
///
/// false のまま前方速度だけ適用すると、PC 側のモデルと実挙動が食い違い、
/// Nav2 側が「なぜ進まないか」を診断できなくなる。指令側は
/// これをエラーコード付きで拒否する。
inline bool SupportsLateralMotion() { return false; }

}  // namespace meister

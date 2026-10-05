/*
 * arm.h — アーム（関節 4 軸 PWM サーボ）+ グリッパー
 *
 * グリッパーはサーボチャネルを参照で掴む。参照先のチャネルは
 * コンストラクタの初期化リストで先に作っている（メンバ宣言順でも保証される）。
 */
#pragma once

#include <stdint.h>

#include "hal/console_servo.h"
#include "hal/pwm_servo.h"
#include "hal/servo.h"
#include "meister_protocol.h"

namespace meister {

/// ドライバ選択: MSTE_SERVO_DRIVER_DEBUG 定義時はコンソール出力に切替
#if defined(MSTE_SERVO_DRIVER_DEBUG)
using ArmServoImpl = hal::ConsoleServoChannel;
using GripperServoImpl = hal::ConsoleServoChannel;
#else
using ArmServoImpl = hal::PwmServoChannel;
using GripperServoImpl = hal::PwmServoChannel;
#endif

/// グリッパー: 開/閉の 2 ポジション制御（サーボチャネルを内包）
class Gripper {
 public:
  explicit Gripper(hal::IServoChannel& servo) : servo_(servo) {}

  void setCommand(proto::GripperCommand cmd);

 private:
  static constexpr int16_t kOpenTenths = 100;  // 10.0°（開）— 実機に合わせて調整
  static constexpr int16_t kCloseTenths = 0;   // 0.0°（閉）
  hal::IServoChannel& servo_;
};

class Arm {
 public:
  Arm();

  void begin();
  void setAngles(const int16_t (&angles)[proto::kNumArmServos]);
  void setGripperCommand(proto::GripperCommand cmd);

 private:
  ArmServoImpl armServos_[proto::kNumArmServos];
  // gripper_ が参照するチャネル。先に構築される必要がある
  GripperServoImpl gripperServoChannel_;
  Gripper gripper_;
};

}  // namespace meister

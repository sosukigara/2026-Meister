// 静的グローバルは置かない（初期化順を翻訳単位に依存させないため）。
#pragma once

#include <stdint.h>

#include "hal/console_servo.h"
#include "hal/pwm_servo.h"
#include "meister_protocol.h"

namespace meister {

/// ドライバ選択: MSTE_SERVO_DRIVER_DEBUG 定義時はコンソール出力に切替
#if defined(MSTE_SERVO_DRIVER_DEBUG)
using SteerServoImpl = hal::ConsoleServoChannel;
#else
using SteerServoImpl = hal::PwmServoChannel;
#endif

class BaseChassis {
 public:
  BaseChassis();

  void begin();
  void setSteering(const int16_t (&angles)[proto::kNumSteeringServos]);

 private:
  SteerServoImpl steerServos_[proto::kNumSteeringServos];
};

}  // namespace meister

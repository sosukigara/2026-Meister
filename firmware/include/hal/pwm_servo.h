/*
 * pwm_servo.h — LEDC PWM でサーボを駆動するチャネル
 *
 * 50Hz の矩形波（パルス幅 minUs..maxUs）で角度を出す。
 */
#pragma once

#include <stdint.h>

#include "hal/servo.h"
#include "meister_config.h"

namespace meister {
namespace hal {

class PwmServoChannel : public IServoChannel {
 public:
  PwmServoChannel(uint8_t pin, int16_t rangeMinTenths, int16_t rangeMaxTenths,
                  uint16_t minUs = static_cast<uint16_t>(config::kServoMinPulseUs),
                  uint16_t maxUs = static_cast<uint16_t>(config::kServoMaxPulseUs));

  bool begin() override;
  bool setAngleTenths(int16_t tenths) override;
  void neutral() override;

 private:
  static uint32_t PulseUsToDuty(uint32_t us);
  uint8_t pin_;
  int16_t rangeMinTenths_;
  int16_t rangeMaxTenths_;
  uint16_t minUs_;
  uint16_t maxUs_;
  bool attached_ = false;
};

}  // namespace hal
}  // namespace meister

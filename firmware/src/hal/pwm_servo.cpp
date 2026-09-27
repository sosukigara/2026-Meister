#ifdef ARDUINO


#include <Arduino.h>
#include "hal/pwm_servo.h"

#include "hal/ledc_pwm.h"

namespace meister {
namespace hal {

PwmServoChannel::PwmServoChannel(uint8_t pin, int16_t rangeMinTenths,
                                 int16_t rangeMaxTenths, uint16_t minUs,
                                 uint16_t maxUs)
    : pin_(pin),
      rangeMinTenths_(rangeMinTenths),
      rangeMaxTenths_(rangeMaxTenths),
      minUs_(minUs),
      maxUs_(maxUs) {}

bool PwmServoChannel::begin() {
  attached_ = ledc::attach(pin_, config::kServoFreqHz, config::kServoResolution);
  if (attached_) {
    neutral();
  }
  return attached_;
}

bool PwmServoChannel::setAngleTenths(int16_t tenths) {
  if (!attached_) {
    return false;
  }
  if (tenths < rangeMinTenths_) {
    tenths = rangeMinTenths_;
  }
  if (tenths > rangeMaxTenths_) {
    tenths = rangeMaxTenths_;
  }
  // 角度 → パルス幅 [us] → デューティ比
  const int32_t span = static_cast<int32_t>(rangeMaxTenths_) - rangeMinTenths_;
  const uint32_t us = minUs_ + static_cast<uint32_t>(tenths - rangeMinTenths_) *
                                 (static_cast<uint32_t>(maxUs_) - minUs_) /
                                 static_cast<uint32_t>(span);
  ledc::write(pin_, PulseUsToDuty(us));
  return true;
}

void PwmServoChannel::neutral() {
  const int16_t mid = static_cast<int16_t>(rangeMinTenths_ +
                                           (static_cast<int32_t>(rangeMaxTenths_) - rangeMinTenths_) / 2);
  setAngleTenths(mid);
}

uint32_t PwmServoChannel::PulseUsToDuty(uint32_t us) {
  return us * (1u << config::kServoResolution) * config::kServoFreqHz / 1000000u;
}

}  // namespace hal
}  // namespace meister

#endif  // ARDUINO

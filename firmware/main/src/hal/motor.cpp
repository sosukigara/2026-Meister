#ifdef ARDUINO

#include "hal/motor.h"

#include <Arduino.h>

#include "hal/ledc_pwm.h"
#include "meister_config.h"
#include "meister_protocol.h"

namespace meister {
namespace hal {

bool MotorChannel::begin() {
  if (pwmPin_ != kNoPin &&
      !ledc::attach(pwmPin_, config::kMotorFreqHz, config::kMotorResolution)) {
    return false;
  }
  if (dirPin_ >= 0) {
    pinMode(dirPin_, OUTPUT);
    digitalWrite(dirPin_, LOW);
  }
  return true;
}

void MotorChannel::setVelocity(int16_t vel) {
  if (vel > proto::kMaxVelocity) {
    vel = proto::kMaxVelocity;
  }
  if (vel < proto::kMinVelocity) {
    vel = proto::kMinVelocity;
  }
  const bool forward = vel >= 0;
  const uint16_t absVel = forward ? static_cast<uint16_t>(vel)
                                  : static_cast<uint16_t>(-vel);
  const uint32_t maxDuty = (1u << config::kMotorResolution) - 1u;
  const uint32_t duty = absVel * maxDuty / static_cast<uint32_t>(proto::kMaxVelocity);
  if (dirPin_ >= 0) {
    digitalWrite(dirPin_, forward ? HIGH : LOW);
  }
  ledc::write(pwmPin_, duty);
}

}  // namespace hal
}  // namespace meister

#endif  // ARDUINO

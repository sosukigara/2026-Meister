#ifdef ARDUINO

#include "hal/console_servo.h"

#include <Arduino.h>

#include "meister_config.h"

namespace meister {
namespace hal {

ConsoleServoChannel::ConsoleServoChannel(uint8_t pin, int16_t rangeMinTenths,
                                         int16_t rangeMaxTenths)
    : rangeMinTenths_(rangeMinTenths), rangeMaxTenths_(rangeMaxTenths) {
  snprintf(name_, sizeof(name_), "servo%u", pin);
}

bool ConsoleServoChannel::begin() { return true; }

bool ConsoleServoChannel::setAngleTenths(int16_t tenths) {
  if (tenths < rangeMinTenths_) {
    tenths = rangeMinTenths_;
  }
  if (tenths > rangeMaxTenths_) {
    tenths = rangeMaxTenths_;
  }
  MSTE_LOG("[servo] %-12s angle = %+7.1f deg\n", name_, tenths / 10.0f);
  return true;
}

void ConsoleServoChannel::neutral() {
  const int16_t mid = static_cast<int16_t>(rangeMinTenths_ +
                                           (static_cast<int32_t>(rangeMaxTenths_) - rangeMinTenths_) / 2);
  setAngleTenths(mid);
}

}  // namespace hal
}  // namespace meister

#endif  // ARDUINO

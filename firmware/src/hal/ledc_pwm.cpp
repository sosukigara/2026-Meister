#ifdef ARDUINO

#include <Arduino.h>

#include "hal/ledc_pwm.h"

namespace meister {
namespace hal {
namespace ledc {

#if ESP_ARDUINO_VERSION_MAJOR < 3
int8_t sChannelOfPin[40];  // pin → LEDC チャネル（-1 = 未使用）
uint8_t sNextChannel = 0;
#endif

bool attach(uint8_t pin, uint32_t freq, uint8_t resolutionBits) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ::ledcAttach(pin, freq, resolutionBits);
#else
  if (pin >= 40 || sNextChannel >= 16) {
    return false;
  }
  const uint8_t ch = sNextChannel++;
  if (!::ledcSetup(ch, freq, resolutionBits)) {
    return false;
  }
  if (!::ledcAttachPin(pin, ch)) {
    return false;
  }
  sChannelOfPin[pin] = static_cast<int8_t>(ch);
  return true;
#endif
}

void write(uint8_t pin, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ::ledcWrite(pin, duty);
#else
  if (pin < 40 && sChannelOfPin[pin] >= 0) {
    ::ledcWrite(static_cast<uint8_t>(sChannelOfPin[pin]), duty);
  }
#endif
}

}  // namespace ledc
}  // namespace hal
}  // namespace meister

#endif  // ARDUINO

#ifdef ARDUINO

#include <Arduino.h>

#include "meister_config.h"

#include "hal/ledc_pwm.h"

namespace meister {
namespace hal {
namespace ledc {

// attach した数と失敗した数。失敗を黙って流さないために数える。
uint16_t attached = 0;
uint16_t failed = 0;

#if ESP_ARDUINO_VERSION_MAJOR < 3
int8_t sChannelOfPin[40];  // pin → LEDC チャネル（-1 = 未使用）
uint8_t sNextChannel = 0;
#endif

bool attach(uint8_t pin, uint32_t freq, uint8_t resolutionBits) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  const bool ok = ::ledcAttach(pin, freq, resolutionBits);
#else
  bool ok = false;
  if (pin < 40 && sNextChannel < 16) {
    const uint8_t ch = sNextChannel++;
    ok = ::ledcSetup(ch, freq, resolutionBits) && ::ledcAttachPin(pin, ch);
    if (ok) {
      sChannelOfPin[pin] = static_cast<int8_t>(ch);
    }
  }
#endif
  if (ok) {
    ++attached;
  } else {
    // ここで黙って返ると、そのピンのサーボ/モータだけが動かないまま気づけない。
    ++failed;
    MSTE_LOG("[ledc] attach FAILED pin=%u freq=%u\n", pin, freq);
  }
  return ok;
}

uint16_t attached_count() { return attached; }
uint16_t failed_count() { return failed; }

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

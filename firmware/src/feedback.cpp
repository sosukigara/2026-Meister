#ifdef ARDUINO

#include "feedback.h"

#include <Arduino.h>

#include "hal/proto_uart.h"
#include "meister_config.h"
#include "meister_protocol.h"

namespace meister {

void Feedback::tick(uint8_t errorFlags) {
  constexpr uint32_t kIntervalMs = config::kFeedbackIntervalMs;
  const uint32_t now = millis();
  if (now - lastSendMs_ < kIntervalMs) {
    return;
  }
  lastSendMs_ = now;

  // エンコーダ値はプレースホルダ（将来は実エンコーダから読む）
  int16_t encoders[proto::kNumDriveMotors];
  for (size_t i = 0; i < proto::kNumDriveMotors; ++i) {
    encoders[i] = static_cast<int16_t>(((now / 100) * 3 + i * 137) % 2000 - 1000);
  }

  uint8_t buf[proto::kMaxFrameSize];
  const size_t n = proto::EncodeState(buf, sizeof(buf), encoders, 0, errorFlags);
  hal::proto_uart::write(buf, n);
}

}  // namespace meister

#endif  // ARDUINO

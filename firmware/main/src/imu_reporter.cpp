#ifdef ARDUINO

#include "imu_reporter.h"

#include <Arduino.h>

#include "hal/imu_bno055.h"
#include "hal/proto_uart.h"
#include "meister_config.h"
#include "meister_protocol.h"

namespace meister {

void ImuReporter::begin() {
#if MSTE_ENABLE_IMU
  ok_ = hal::imu_bno055::begin(config::kImuSdaPin, config::kImuSclPin,
                               config::kImuAddr);
  MSTE_LOG("[imu] bno055 %s (sda=%u scl=%u addr=0x%02X)\n", ok_ ? "ok" : "NG",
           config::kImuSdaPin, config::kImuSclPin, config::kImuAddr);
#else
  ok_ = true;  // 無効時は sensor ビットを立てない
#endif
}

void ImuReporter::tick() {
#if MSTE_ENABLE_IMU
  if (!ok_) {
    return;
  }
  const uint32_t now = millis();
  if (now - lastSendMs_ < config::kImuIntervalMs) {
    return;
  }
  lastSendMs_ = now;

  int16_t yaw = 0;
  uint8_t calib = 0;
  if (!hal::imu_bno055::readYaw(&yaw, &calib)) {
    ok_ = false;  // ponytail: ラッチして黙る。復帰は次回 boot（再送スパムより単純）
    return;
  }

  uint8_t buf[proto::kMaxFrameSize];
  const size_t n = proto::EncodeImu(buf, sizeof(buf), yaw, calib);
  hal::proto_uart::write(buf, n);
#endif
}

bool ImuReporter::ok() const { return ok_; }

}  // namespace meister

#endif  // ARDUINO

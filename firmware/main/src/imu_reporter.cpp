#ifdef ARDUINO

#include "imu_reporter.h"

#include <Arduino.h>

#include "hal/imu_bno055.h"
#include "hal/proto_uart.h"
#include "meister_config.h"
#include "meister_protocol.h"

namespace meister {
namespace {

// 未接続のまま boot したときの再試行間隔 [ms]。通常時は通らない分岐。
constexpr uint32_t kRetryIntervalMs = 2000;

}  // namespace

void ImuReporter::begin() {
  // src/ は MSTE_* を直接見ない。機能ゲートは config::kImuEnabled が持つ。
  if (!config::kImuEnabled) {
    ok_ = true;  // 無効時は sensor ビットを立てない
    return;
  }
  ok_ = hal::imu_bno055::begin(config::kImuSdaPin, config::kImuSclPin,
                               config::kImuAddr);
  MSTE_LOG("[imu] bno055 %s (sda=%u scl=%u addr=0x%02X)\n", ok_ ? "ok" : "NG",
           config::kImuSdaPin, config::kImuSclPin, config::kImuAddr);
}

void ImuReporter::tick() {
  if (!config::kImuEnabled) {
    return;
  }
  const uint32_t now = millis();
  if (!ok_) {
    if (now - lastRetryMs_ >= kRetryIntervalMs) {
      lastRetryMs_ = now;
      begin();  // 後挿し救済。成功するまで FB_IMU は送らない。
    }
    return;
  }
  if (now - lastSendMs_ < config::kImuIntervalMs) {
    return;
  }
  lastSendMs_ = now;

  int16_t yaw = 0;
  uint8_t calib = 0;
  if (!hal::imu_bno055::readYaw(&yaw, &calib)) {
    ok_ = false;
    return;
  }

  uint8_t buf[proto::kMaxFrameSize];
  const size_t n = proto::EncodeImu(buf, sizeof(buf), yaw, calib);
  hal::proto_uart::write(buf, n);
}

bool ImuReporter::ok() const { return ok_; }

}  // namespace meister

#endif  // ARDUINO

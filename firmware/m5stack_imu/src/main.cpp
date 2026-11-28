#include <M5Unified.h>

#include <stddef.h>

#include "imu_config.h"
#include "imu_protocol.h"
#include "imu_sampling.h"
#include "imu_view.h"

namespace {

meister_imu::ImuView g_view;
uint32_t g_last_tx_ms = 0;
uint32_t g_last_draw_ms = 0;
uint32_t g_window_start_ms = 0;
uint32_t g_frames_in_window = 0;
uint32_t g_tx_hz = 0;
bool g_tx_ok = false;

const char* ImuTypeName(m5::imu_t type) {
  switch (type) {
    case m5::imu_bmi270: return "BMI270";
    case m5::imu_mpu6886: return "MPU6886";
    case m5::imu_mpu6050: return "MPU6050";
    case m5::imu_mpu9250: return "MPU9250";
    case m5::imu_sh200q: return "SH200Q";
    case m5::imu_unknown: return "unknown";
    case m5::imu_none: return "none";
    default: return "?";
  }
}

}  // namespace

void setup() {
  auto cfg = M5.config();
  cfg.internal_imu = true;
  cfg.external_imu = false;
  M5.begin(cfg);

  Serial.begin(meister_imu::config::kSerialBaud);
  g_view.Begin();

  // 起動時だけテキストを出す。定常送信はバイナリフレームのみ。
  Serial.printf("M5Stack_IMU ready: imu=%s frame=%u bytes interval=%u ms\n",
                ImuTypeName(M5.Imu.getType()), meister_imu::protocol::kFrameSize,
                meister_imu::config::kTxIntervalMs);
}

void loop() {
  const uint32_t now = millis();
  const bool updated = M5.Imu.update() != 0;
  const m5::imu_data_t data = M5.Imu.getImuData();

  meister_imu::RawImu raw;
  raw.accel[0] = data.accel.x;
  raw.accel[1] = data.accel.y;
  raw.accel[2] = data.accel.z;
  raw.gyro[0] = data.gyro.x;
  raw.gyro[1] = data.gyro.y;
  raw.gyro[2] = data.gyro.z;
  raw.timestamp_ms = data.usec / 1000;
  raw.present = updated;

  meister_imu::protocol::Sample sample;
  meister_imu::ConvertSample(raw, &sample);

  if (now - g_last_tx_ms >= meister_imu::config::kTxIntervalMs) {
    uint8_t frame[meister_imu::protocol::kFrameSize];
    const size_t n = meister_imu::protocol::EncodeFrame(frame, sizeof(frame), sample);
    g_tx_ok = (n == meister_imu::protocol::kFrameSize) &&
              (Serial.write(frame, n) == n);
    g_last_tx_ms = now;

    ++g_frames_in_window;
    const uint32_t elapsed = now - g_window_start_ms;
    if (elapsed >= 1000) {
      g_tx_hz = g_frames_in_window * 1000 / elapsed;
      g_window_start_ms = now;
      g_frames_in_window = 0;
    }
  }

  if (now - g_last_draw_ms >= meister_imu::config::kDrawIntervalMs) {
    g_view.Draw(sample, raw, g_tx_hz, g_tx_ok);
    g_last_draw_ms = now;
  }

  M5.update();
  delay(1);
}

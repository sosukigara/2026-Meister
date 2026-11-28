#include <M5Unified.h>

#include <math.h>
#include <stddef.h>

#include "imu_config.h"
#include "imu_protocol.h"
#include "imu_sampling.h"
#include "imu_view.h"
#include "imu_yaw.h"

namespace {

meister_imu::ImuView g_view;
meister_imu::YawTracker g_yaw;
meister_imu::RawImu g_latest;
meister_imu::protocol::Sample g_tx_sample;
portMUX_TYPE g_sample_mux = portMUX_INITIALIZER_UNLOCKED;

uint32_t g_boot_ms = 0;
bool g_yaw_calibrated = false;
uint32_t g_last_fresh_ms = 0;
uint32_t g_last_draw_ms = 0;
uint32_t g_window_start_ms = 0;
uint32_t g_frames_in_window = 0;
uint32_t g_tx_hz = 0;
volatile bool g_tx_ok = false;

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

// 送信を描画ループから分離し、画面更新が重くても 20 Hz を維持する。
void TxTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(meister_imu::config::kTxIntervalMs));

    meister_imu::protocol::Sample sample;
    portENTER_CRITICAL(&g_sample_mux);
    sample = g_tx_sample;
    portEXIT_CRITICAL(&g_sample_mux);

    uint8_t frame[meister_imu::protocol::kFrameSize];
    const size_t n = meister_imu::protocol::EncodeFrame(frame, sizeof(frame), sample);
    g_tx_ok = (n == meister_imu::protocol::kFrameSize) &&
              (Serial.write(frame, n) == n);

    ++g_frames_in_window;
    const uint32_t now = millis();
    const uint32_t elapsed = now - g_window_start_ms;
    if (elapsed >= 1000) {
      g_tx_hz = g_frames_in_window * 1000 / elapsed;
      g_window_start_ms = now;
      g_frames_in_window = 0;
    }
  }
}

}  // namespace

void setup() {
  auto cfg = M5.config();
  cfg.internal_imu = true;
  cfg.external_imu = false;
  M5.begin(cfg);

  Serial.begin(meister_imu::config::kSerialBaud);
  g_boot_ms = millis();

  // 起動時だけテキストを出す。定常送信はバイナリフレームのみ。
  Serial.printf(
      "M5Stack_IMU ready: imu=%s frame=%u bytes interval=%u ms display=%dx%d "
      "yaw_cal=%u ms\n",
      ImuTypeName(M5.Imu.getType()), meister_imu::protocol::kFrameSize,
      meister_imu::config::kTxIntervalMs, M5.Display.width(),
      M5.Display.height(), meister_imu::config::kYawCalibrationMs);
  g_view.Begin();

  xTaskCreatePinnedToCore(TxTask, "imu_tx", 4096, nullptr, 2, nullptr, 0);
}

void loop() {
  const uint32_t now = millis();
  if (M5.Imu.update() != 0) {
    const m5::imu_data_t data = M5.Imu.getImuData();
    g_latest.accel[0] = data.accel.x;
    g_latest.accel[1] = data.accel.y;
    g_latest.accel[2] = data.accel.z;
    g_latest.gyro[0] = data.gyro.x;
    g_latest.gyro[1] = data.gyro.y;
    g_latest.gyro[2] = data.gyro.z;
    g_latest.timestamp_ms = data.usec / 1000;
    g_latest.timestamp_us = data.usec;
    g_latest.present = true;
    g_last_fresh_ms = now;

    if (!g_yaw_calibrated) {
      g_yaw.BiasSample(g_latest.gyro);
      if (now - g_boot_ms >= meister_imu::config::kYawCalibrationMs) {
        g_yaw.FinishBias();
        g_yaw.Reset();
        g_yaw_calibrated = true;
      }
    } else {
      g_yaw.Update(g_latest);
    }
  }

  // 送信瞬間の update() 結果ではなく、直近の新鮮な値を送る。
  meister_imu::RawImu raw = g_latest;
  raw.present = g_last_fresh_ms != 0 &&
                (now - g_last_fresh_ms) <= meister_imu::config::kImuFreshTimeoutMs;

  meister_imu::protocol::Sample sample;
  meister_imu::ConvertSample(raw, &sample);
  sample.yaw_centideg = static_cast<uint16_t>(
      lroundf(g_yaw.yaw_deg() * 100.0f)) % 36000u;

  portENTER_CRITICAL(&g_sample_mux);
  g_tx_sample = sample;
  portEXIT_CRITICAL(&g_sample_mux);

  if (now - g_last_draw_ms >= meister_imu::config::kDrawIntervalMs) {
    g_view.Draw(sample, raw, g_tx_hz, g_tx_ok);
    g_last_draw_ms = now;
  }

  M5.update();
  delay(1);
}

#include "imu_view.h"

#include <M5GFX.h>
#include <M5Unified.h>

#include <stdio.h>

namespace meister_imu {
namespace {

constexpr int32_t kTopY = 56;
constexpr int32_t kLineH = 30;
constexpr int32_t kBarH = 14;
constexpr int32_t kLeftX = 12;

void DrawBar(int32_t x, int32_t y, int32_t w, int32_t h, float value,
             float range, uint32_t color) {
  if (w <= 0) {
    return;
  }
  const float clipped = value > range ? range : (value < -range ? -range : value);
  const int32_t length = static_cast<int32_t>(clipped / range * (w / 2));
  const int32_t center = x + w / 2;
  if (length >= 0) {
    M5.Display.fillRect(center, y, length, h, color);
  } else {
    M5.Display.fillRect(center + length, y, -length, h, color);
  }
  M5.Display.fillRect(center, y - 2, 1, h + 4, TFT_WHITE);
}

}  // namespace

void ImuView::Begin() {
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.drawString("M5Stack IMU (BMI270)", kLeftX, 12);
}

void ImuView::Draw(const protocol::Sample& sample, const RawImu& raw,
                   uint32_t tx_hz, bool tx_ok) {
  const int32_t w = M5.Display.width();
  const int32_t h = M5.Display.height();
  if (w <= 0 || h <= kTopY) {
    return;
  }

  const char* labels[6] = {"ACC X [G]", "ACC Y [G]", "ACC Z [G]",
                           "GYR X[dps]", "GYR Y[dps]", "GYR Z[dps]"};
  const float values[6] = {raw.accel[0], raw.accel[1], raw.accel[2],
                           raw.gyro[0], raw.gyro[1], raw.gyro[2]};
  const float ranges[6] = {2.0f, 2.0f, 2.0f, 250.0f, 250.0f, 250.0f};
  const uint32_t colors[6] = {TFT_RED, TFT_GREEN, TFT_BLUE,
                              TFT_YELLOW, TFT_CYAN, TFT_MAGENTA};

  const Attitude attitude = EstimateAttitude(raw);
  char line[96];

  M5.Display.startWrite();
  M5.Display.fillRect(0, kTopY, w, h - kTopY, TFT_BLACK);
  M5.Display.setTextSize(1);

  const int32_t bar_x = w / 2 + 40;
  const int32_t bar_w = w - bar_x - kLeftX;
  for (int i = 0; i < 6; ++i) {
    const int32_t y = kTopY + i * kLineH;
    snprintf(line, sizeof(line), "%-10s %+8.3f", labels[i], values[i]);
    M5.Display.setTextColor(colors[i], TFT_BLACK);
    M5.Display.drawString(line, kLeftX, y);
    DrawBar(bar_x, y + 3, bar_w, kBarH, values[i], ranges[i], colors[i]);
  }

  int32_t y = kTopY + 6 * kLineH + 8;
  snprintf(line, sizeof(line), "ROLL %+7.1f deg    PITCH %+7.1f deg",
           attitude.roll_deg, attitude.pitch_deg);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.drawString(line, kLeftX, y);

  y += kLineH;
  const char* flags = (sample.status & protocol::kImuMissing)
                          ? "NO-IMU"
                          : ((sample.status & protocol::kClamped) ? "CLAMP" : "OK");
  snprintf(line, sizeof(line), "TX %3u Hz  %-6s  %s", tx_hz, tx_ok ? "SENT" : "FAIL",
           flags);
  M5.Display.setTextColor(tx_ok ? TFT_GREEN : TFT_RED, TFT_BLACK);
  M5.Display.drawString(line, kLeftX, y);
  M5.Display.endWrite();
}

}  // namespace meister_imu

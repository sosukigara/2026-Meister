#include "imu_view.h"

#include <M5GFX.h>
#include <M5Unified.h>

#include <math.h>
#include <stdio.h>

namespace meister_imu {
namespace {

constexpr uint32_t kBg = 0x070C18;
constexpr uint32_t kCard = 0x111C30;
constexpr uint32_t kShade = 0x040912;
constexpr uint32_t kLine = 0x243352;
constexpr uint32_t kText = 0xF2F7FF;
constexpr uint32_t kMuted = 0x7F93B5;
constexpr uint32_t kAccent = 0x3FE0D0;
constexpr uint32_t kAccent2 = 0x4DA3FF;
constexpr uint32_t kAccentDim = 0x1D4A52;
constexpr uint32_t kGreen = 0x5CE6A5;
constexpr uint32_t kAmber = 0xFFC978;
constexpr uint32_t kRed = 0xFF6F8D;
constexpr uint32_t kTrack = 0x0D1628;
constexpr uint32_t kHeader = 0x0A1120;
constexpr float kPi = 3.14159265358979f;

ViewLayout ComputeLayout(int32_t w, int32_t h) {
  ViewLayout l;
  l.w = w;
  l.h = h;
  const int32_t margin = 14;
  const int32_t top = 66;
  const int32_t content_h = h - 54 - top;
  l.mini_h = 68;

  if (w >= h) {
    const int32_t left_w = w * 44 / 100 - margin;
    l.compass_x = margin;
    l.compass_y = top;
    l.compass_w = left_w;
    l.compass_h = content_h;
    const int32_t right_x = left_w + margin * 2;
    l.mini_w = (w - right_x - margin - 10) / 2;
    l.roll_x = right_x;
    l.pitch_x = right_x + (w - right_x - margin + 10) / 2;
    l.roll_y = top;
    l.motion_x = right_x;
    l.motion_y = top + 80;
    l.motion_w = w - right_x - margin;
    l.motion_h = content_h - 80;
  } else {
    const int32_t compass_h = content_h * 46 / 100;
    l.compass_x = margin;
    l.compass_y = top;
    l.compass_w = w - margin * 2;
    l.compass_h = compass_h;
    l.mini_w = (w - margin * 2 - 10) / 2;
    l.roll_x = margin;
    l.pitch_x = margin + (w - margin * 2 + 10) / 2;
    l.roll_y = top + compass_h + 10;
    l.motion_x = margin;
    l.motion_y = top + compass_h + 88;
    l.motion_w = w - margin * 2;
    l.motion_h = content_h - compass_h - 88;
  }

  l.cx = l.compass_x + l.compass_w / 2;
  l.cy = l.compass_y + l.compass_h / 2 + 8;
  const int32_t usable = l.compass_w < l.compass_h - 130 ? l.compass_w
                                                          : l.compass_h - 130;
  l.radius = usable / 2 - 10;
  l.trend_x = l.compass_x + 16;
  l.trend_y = l.compass_y + l.compass_h - 58;
  l.trend_w = l.compass_w - 32;
  return l;
}

void UseSmallFont() {
  M5.Display.setTextFont(&fonts::FreeMonoBold9pt7b);
  M5.Display.setTextSize(1);
}

void UseBigFont() {
  M5.Display.setTextFont(&fonts::FreeSansBold24pt7b);
  M5.Display.setTextSize(1);
}

void DrawPill(int32_t x, int32_t y, int32_t w, int32_t h, const char* text,
              uint32_t color) {
  M5.Display.fillRoundRect(x, y, w, h, h / 2, color);
  M5.Display.setTextColor(kBg, color);
  M5.Display.setTextDatum(TC_DATUM);
  M5.Display.drawString(text, x + w / 2, y + h / 2 - 5);
  M5.Display.setTextDatum(TL_DATUM);
}

void DrawHeader(int32_t w) {
  M5.Display.fillRect(0, 0, w, 54, kHeader);
  M5.Display.fillRect(0, 53, w, 1, kLine);
  UseSmallFont();
  M5.Display.setTextColor(kAccent, kHeader);
  M5.Display.setTextDatum(TL_DATUM);
  M5.Display.drawString("MEISTER", 16, 4);
  UseBigFont();
  M5.Display.setTextColor(kText, kHeader);
  M5.Display.drawString("IMU // TAB5", 16, 18);
  M5.Display.fillRect(16, 51, 118, 2, kAccent);
  DrawPill(w - 78, 16, 62, 22, "LIVE", kGreen);
}

void DrawStatic(const ViewLayout& l) {
  M5.Display.fillScreen(kBg);
  DrawHeader(l.w);

  M5.Display.fillRoundRect(l.compass_x + 3, l.compass_y + 4, l.compass_w,
                           l.compass_h, 16, kShade);
  M5.Display.fillRoundRect(l.roll_x + 2, l.roll_y + 3, l.mini_w, l.mini_h, 14,
                           kShade);
  M5.Display.fillRoundRect(l.pitch_x + 2, l.roll_y + 3, l.mini_w, l.mini_h, 14,
                           kShade);
  M5.Display.fillRoundRect(l.motion_x + 3, l.motion_y + 4, l.motion_w,
                           l.motion_h, 16, kShade);

  M5.Display.fillRoundRect(l.compass_x, l.compass_y, l.compass_w, l.compass_h,
                           16, kCard);
  M5.Display.drawRoundRect(l.compass_x, l.compass_y, l.compass_w, l.compass_h,
                           16, kLine);
  M5.Display.fillRoundRect(l.roll_x, l.roll_y, l.mini_w, l.mini_h, 14, kCard);
  M5.Display.drawRoundRect(l.roll_x, l.roll_y, l.mini_w, l.mini_h, 14, kLine);
  M5.Display.fillRoundRect(l.pitch_x, l.roll_y, l.mini_w, l.mini_h, 14, kCard);
  M5.Display.drawRoundRect(l.pitch_x, l.roll_y, l.mini_w, l.mini_h, 14, kLine);
  M5.Display.fillRoundRect(l.motion_x, l.motion_y, l.motion_w, l.motion_h, 16,
                           kCard);
  M5.Display.drawRoundRect(l.motion_x, l.motion_y, l.motion_w, l.motion_h, 16,
                           kLine);

  UseSmallFont();
  M5.Display.setTextColor(kMuted, kCard);
  M5.Display.setTextDatum(TL_DATUM);
  M5.Display.drawString("HEADING / YAW", l.compass_x + 16, l.compass_y + 12);
  M5.Display.drawString("ROLL", l.roll_x + 14, l.roll_y + 10);
  M5.Display.drawString("PITCH", l.pitch_x + 14, l.roll_y + 10);
  M5.Display.drawString("MOTION / 6-AXIS", l.motion_x + 14, l.motion_y + 10);

  const char* labels[6] = {"ACC X", "ACC Y", "ACC Z", "GYR X", "GYR Y", "GYR Z"};
  const int32_t row_h = l.motion_h > 90 ? (l.motion_h - 36) / 6 : 0;
  for (int i = 0; i < 6 && row_h > 0; ++i) {
    const int32_t ry = l.motion_y + 30 + i * row_h;
    M5.Display.drawString(labels[i], l.motion_x + 14, ry + row_h / 2 - 5);
  }

  const int32_t fy = l.h - 42;
  M5.Display.fillRect(0, fy, l.w, 42, kHeader);
  M5.Display.fillRect(0, fy, l.w, 1, kLine);
}

void DrawCompass(const ViewLayout& l, float yaw, float roll, float pitch,
                 const float* history, uint8_t count, uint8_t index) {
  if (l.radius < 30) {
    return;
  }
  M5.Display.fillCircle(l.cx, l.cy, l.radius - 2, kCard);
  M5.Display.drawArc(l.cx, l.cy, l.radius, l.radius, 0, 360, kLine);
  M5.Display.drawArc(l.cx, l.cy, l.radius - 6, l.radius - 6, 0, 360, kAccentDim);

  const char* cardinal[4] = {"N", "E", "S", "W"};
  for (int a = 0; a < 360; a += 10) {
    const float theta = static_cast<float>(a) * kPi / 180.0f - kPi / 2.0f;
    const int32_t tick = a % 90 == 0 ? 11 : (a % 30 == 0 ? 8 : 4);
    const int32_t inner = l.radius - tick;
    const int32_t outer = l.radius + 3;
    const uint32_t color =
        a % 90 == 0 ? kAccent : (a % 30 == 0 ? kText : kLine);
    M5.Display.drawLine(l.cx + static_cast<int32_t>(inner * cosf(theta)),
                        l.cy + static_cast<int32_t>(inner * sinf(theta)),
                        l.cx + static_cast<int32_t>(outer * cosf(theta)),
                        l.cy + static_cast<int32_t>(outer * sinf(theta)), color);
  }
  UseSmallFont();
  for (int i = 0; i < 4; ++i) {
    const float theta = static_cast<float>(i * 90) * kPi / 180.0f - kPi / 2.0f;
    M5.Display.setTextDatum(TC_DATUM);
    M5.Display.setTextColor(i == 0 ? kAccent : kMuted, kCard);
    M5.Display.drawString(cardinal[i],
                          l.cx + static_cast<int32_t>((l.radius + 19) * cosf(theta)),
                          l.cy + static_cast<int32_t>((l.radius + 19) * sinf(theta)) - 5);
  }

  if (yaw > 0.1f) {
    M5.Display.fillArc(l.cx, l.cy, l.radius - 13, l.radius - 8, -90,
                       -90 + yaw, kAccentDim);
    M5.Display.fillArc(l.cx, l.cy, l.radius - 7, l.radius, -90, -90 + yaw,
                       kAccent);
  }
  const float needle = yaw * kPi / 180.0f - kPi / 2.0f;
  const int32_t nx = l.cx + static_cast<int32_t>((l.radius - 22) * cosf(needle));
  const int32_t ny = l.cy + static_cast<int32_t>((l.radius - 22) * sinf(needle));
  const int32_t tx = l.cx - static_cast<int32_t>((l.radius - 22) * cosf(needle) * 0.35f);
  const int32_t ty = l.cy - static_cast<int32_t>((l.radius - 22) * sinf(needle) * 0.35f);
  M5.Display.drawLine(tx, ty, nx, ny, kAccent2);
  M5.Display.drawLine(tx + 1, ty, nx + 1, ny, kAccent2);
  M5.Display.drawLine(tx - 1, ty, nx - 1, ny, kAccent2);
  M5.Display.fillCircle(l.cx, l.cy, 7, kCard);
  M5.Display.drawCircle(l.cx, l.cy, 7, kAccent);
  M5.Display.fillCircle(l.cx, l.cy, 3, kAccent);

  UseBigFont();
  char text[32];
  snprintf(text, sizeof(text), "%.1f°", yaw);
  M5.Display.setTextDatum(TC_DATUM);
  M5.Display.setTextColor(kText, kCard);
  M5.Display.drawString(text, l.cx, l.cy + 18);
  UseSmallFont();
  M5.Display.setTextColor(kAccent, kCard);
  M5.Display.drawString("REL YAW", l.cx, l.cy + 48);

  UseSmallFont();
  if (count >= 2 && l.trend_w > 8) {
    M5.Display.fillRect(l.trend_x, l.trend_y, l.trend_w, 24, kTrack);
    for (uint8_t i = 1; i < count; ++i) {
      const uint8_t prev = (index + 32 - count + i - 1) % 32;
      const uint8_t cur = (index + 32 - count + i) % 32;
      const int32_t x0 = l.trend_x + static_cast<int32_t>((i - 1) * l.trend_w / 31);
      const int32_t x1 = l.trend_x + static_cast<int32_t>(i * l.trend_w / 31);
      const int32_t y0 = l.trend_y + 22 - static_cast<int32_t>(history[prev] / 360.0f * 20);
      const int32_t y1 = l.trend_y + 22 - static_cast<int32_t>(history[cur] / 360.0f * 20);
      M5.Display.drawLine(x1, l.trend_y + 23, x1, y1, kAccentDim);
      M5.Display.drawLine(x0, y0, x1, y1, kAccent);
    }
  }
  M5.Display.fillRect(l.compass_x + 16, l.compass_y + l.compass_h - 26,
                      l.compass_w - 32, 20, kCard);
  snprintf(text, sizeof(text), "ROLL %+.1f   PITCH %+.1f", roll, pitch);
  M5.Display.setTextDatum(TL_DATUM);
  M5.Display.setTextColor(kMuted, kCard);
  M5.Display.drawString(text, l.compass_x + 16, l.compass_y + l.compass_h - 24);
}

void DrawMiniValue(int32_t x, int32_t y, int32_t w, int32_t h, float value,
                   uint32_t color) {
  M5.Display.fillRect(x + 10, y + 24, w - 20, h - 26, kCard);
  M5.Display.fillRect(x + 4, y + 24, 3, h - 30, color);
  UseBigFont();
  char text[32];
  snprintf(text, sizeof(text), "%+.1f°", value);
  M5.Display.setTextDatum(TL_DATUM);
  M5.Display.setTextColor(color, kCard);
  M5.Display.drawString(text, x + 14, y + 26);
}

void DrawMotion(const ViewLayout& l, const RawImu& raw) {
  const int32_t row_h = l.motion_h > 90 ? (l.motion_h - 36) / 6 : 0;
  if (row_h <= 0) {
    return;
  }
  const float values[6] = {raw.accel[0], raw.accel[1], raw.accel[2],
                           raw.gyro[0], raw.gyro[1], raw.gyro[2]};
  const float ranges[6] = {2.0f, 2.0f, 2.0f, 250.0f, 250.0f, 250.0f};
  const uint32_t colors[6] = {kRed, kGreen, kAccent, kAmber, kMuted, kAccent2};
  const int32_t bar_x = l.motion_x + 82;
  const int32_t bar_w = l.motion_w - 196;

  UseSmallFont();
  for (int i = 0; i < 6; ++i) {
    const int32_t ry = l.motion_y + 30 + i * row_h;
    M5.Display.fillRect(bar_x, ry + row_h / 2 - 6, bar_w, 12, kCard);
    M5.Display.fillRoundRect(bar_x, ry + row_h / 2 - 6, bar_w, 12, 6, kLine);
    float clipped = values[i] > ranges[i] ? ranges[i]
                    : (values[i] < -ranges[i] ? -ranges[i] : values[i]);
    const int32_t len = static_cast<int32_t>(fabsf(clipped) / ranges[i] * (bar_w - 2));
    if (len > 0) {
      const int32_t bx = clipped >= 0.0f ? bar_x + 1 : bar_x + bar_w - 1 - len;
      M5.Display.fillRoundRect(bx, ry + row_h / 2 - 5, len, 10, 5, colors[i]);
    }
    char text[32];
    snprintf(text, sizeof(text), "%+.2f", values[i]);
    M5.Display.fillRect(l.motion_x + l.motion_w - 86, ry + row_h / 2 - 6, 72, 12,
                        kCard);
    M5.Display.setTextDatum(TR_DATUM);
    M5.Display.setTextColor(colors[i], kCard);
    M5.Display.drawString(text, l.motion_x + l.motion_w - 14, ry + row_h / 2 - 5);
    M5.Display.setTextDatum(TL_DATUM);
  }
}

void DrawFooter(const ViewLayout& l, uint32_t tx_hz, bool tx_ok, uint8_t status,
                float yaw) {
  const int32_t fy = l.h - 42;
  M5.Display.fillRect(16, fy + 10, l.w - 120, 24, kHeader);
  UseSmallFont();
  M5.Display.setTextDatum(TL_DATUM);
  char line[96];
  snprintf(line, sizeof(line), "TX %3u Hz   YAW %6.1f   %s", tx_hz, yaw,
           (status & protocol::kImuMissing) ? "NO-IMU"
           : (status & protocol::kClamped)  ? "CLAMP"
                                            : "BMI270");
  M5.Display.setTextColor(kMuted, kHeader);
  M5.Display.drawString(line, 16, fy + 16);
  DrawPill(l.w - 92, fy + 10, 76, 22, tx_ok ? "TX OK" : "TX FAIL",
           tx_ok ? kGreen : kRed);
}

}  // namespace

void ImuView::Begin() {
  const int32_t w = M5.Display.width();
  const int32_t h = M5.Display.height();
  if (w < 320 || h < 240) {
    return;
  }
  layout_ = ComputeLayout(w, h);
  M5.Display.startWrite();
  DrawStatic(layout_);
  M5.Display.endWrite();
}

void ImuView::Draw(const protocol::Sample& sample, const RawImu& raw,
                   uint32_t tx_hz, bool tx_ok) {
  if (layout_.w < 320 || layout_.h < 240) {
    return;
  }
  const Attitude attitude = EstimateAttitude(raw);
  const float yaw = sample.yaw_centideg / 100.0f;
  yaw_history_[yaw_index_] = yaw;
  yaw_index_ = (yaw_index_ + 1) % 32;
  if (yaw_count_ < 32) {
    ++yaw_count_;
  }

  M5.Display.startWrite();
  DrawCompass(layout_, yaw, attitude.roll_deg, attitude.pitch_deg, yaw_history_,
              yaw_count_, yaw_index_);
  DrawMiniValue(layout_.roll_x, layout_.roll_y, layout_.mini_w, layout_.mini_h,
                attitude.roll_deg, kGreen);
  DrawMiniValue(layout_.pitch_x, layout_.roll_y, layout_.mini_w, layout_.mini_h,
                attitude.pitch_deg, kAmber);
  DrawMotion(layout_, raw);
  DrawFooter(layout_, tx_hz, tx_ok, sample.status, yaw);
  M5.Display.endWrite();
}

}  // namespace meister_imu

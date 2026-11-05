/*
 * imu_reporter.h — FB_IMU の定周期送信
 *
 * Feedback（FB_STATE）とは別クラスにする。理由: 周期（20Hz）と
 * 失敗時の振る舞い（送らない＋sensor ビット）が FB_STATE と違う。
 * 送信間隔は設定の kImuIntervalMs（1000 / MSTE_IMU_HZ）で決まる。
 */
#pragma once

#include <stdint.h>

namespace meister {

class ImuReporter {
 public:
  /// MSTE_ENABLE_IMU=0 なら何もしない。BNO055 の reset 待ちで最大 ~700ms 止まる。
  void begin();

  /// 周期が過ぎていれば BNO055 を読んで FB_IMU を 1 フレーム送る。
  /// 読取失敗時は送らず ok() が false になる（ゴミを送らない）。
  void tick();

  /// IMU 生存。false の間 main が FB_STATE の sensor ビットを立てる。
  bool ok() const;

 private:
  uint32_t lastSendMs_ = 0;
  uint32_t lastRetryMs_ = 0;
  bool ok_ = false;
};

}  // namespace meister

/*
 * motor.h — 駆動 DC モータ 1 チャネル
 *
 * 速度指令（千分率 ±1000）を符号（方向）と絶対値（デューティ比）に分ける。
 */
#pragma once

#include <stdint.h>

namespace meister {
namespace hal {

class MotorChannel {
 public:
  MotorChannel(uint8_t pwmPin, int8_t dirPin) : pwmPin_(pwmPin), dirPin_(dirPin) {}

  bool begin();
  /// vel: -1000..+1000（千分率）。符号 → 方向、絶対値 → デューティ比
  void setVelocity(int16_t vel);
  void stop() { setVelocity(0); }

 private:
  static constexpr int8_t kNoPin = -1;
  uint8_t pwmPin_;
  int8_t dirPin_;
};

}  // namespace hal
}  // namespace meister

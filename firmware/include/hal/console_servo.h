/*
 * console_servo.h — 実サーボの代わりにコンソールへ角度を表示するチャネル
 *
 * ハード未接続のベンチで MSTE_SERVO_DRIVER_DEBUG を定義したとき使う。
 */
#pragma once

#include <stdint.h>

#include "hal/servo.h"

namespace meister {
namespace hal {

class ConsoleServoChannel : public IServoChannel {
 public:
  ConsoleServoChannel(uint8_t pin, int16_t rangeMinTenths, int16_t rangeMaxTenths);

  bool begin() override;
  bool setAngleTenths(int16_t tenths) override;
  void neutral() override;

 private:
  char name_[16];
  int16_t rangeMinTenths_;
  int16_t rangeMaxTenths_;
};

}  // namespace hal
}  // namespace meister

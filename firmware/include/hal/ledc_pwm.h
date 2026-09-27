/*
 * ledc_pwm.h — LEDC PWM の薄いラッパ
 *
 * arduino-esp32 3.x は pin ベース API（ledcAttach / ledcWrite）、2.x は
 * チャネルベース API。どちらの版でも同じ呼び出しで済むようにする。
 * Arduino 依存は実装側（src/hal/ledc_pwm.cpp）にだけ閉じている。
 */
#pragma once

#include <stdint.h>

namespace meister {
namespace hal {
namespace ledc {

/// 指定ピンに LEDC チャネルを割り当てる。失敗（ピン不正・チャネル枯渇）なら false
bool attach(uint8_t pin, uint32_t freq, uint8_t resolutionBits);

/// デューティ比を書き込む。attach されていない pin は黙って無視される
/// attach に成功した回数
uint16_t attached_count();

/// attach に失敗した回数（0 でないのは異常）
uint16_t failed_count();

void write(uint8_t pin, uint32_t duty);

}  // namespace ledc
}  // namespace hal
}  // namespace meister

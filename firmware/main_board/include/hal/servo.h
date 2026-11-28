// 実サーボ（LEDC PWM）とデバッグ用コンソール出力を同じ接口で扱う境界。
// 機構側は「角度を指定する」操作だけを知る。
#pragma once

#include <stdint.h>

namespace meister {
namespace hal {

class IServoChannel {
 public:
  virtual ~IServoChannel() = default;
  virtual bool begin() = 0;
  /// angleTenths: 0.1° 単位（ステアリング -900..900 / アーム 0..1800）
  virtual bool setAngleTenths(int16_t angleTenths) = 0;
  virtual void neutral() = 0;
};

}  // namespace hal
}  // namespace meister

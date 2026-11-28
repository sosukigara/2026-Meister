// 1000 の約数でない値は meister_config.h の側でコンパイル時に弾かれる。
#pragma once

#include <stdint.h>

namespace meister {

class Feedback {
 public:
  /// 周期が過ぎていれば FB_STATE を 1 フレーム送る
  void tick(uint8_t errorFlags);

 private:
  uint32_t lastSendMs_ = 0;
};

}  // namespace meister

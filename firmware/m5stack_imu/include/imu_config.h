#pragma once

#include <stdint.h>

namespace meister_imu {
namespace config {

// 送信周期とボーレート。設定の唯一の出所（R2）。
constexpr uint32_t kTxIntervalMs = 50;      // 20 Hz
constexpr uint32_t kDrawIntervalMs = 100;   // 画面更新 10 Hz
constexpr uint32_t kSerialBaud = 115200;

// プロトコル上の固定小数点スケール（実測値ではなく表現の選択）。
// 出典: M5Unified の imu_data_t は accel = G, gyro = deg/s。
// https://context7.com/m5stack/m5unified/llms.txt
constexpr float kAccelScale = 1000.0f;  // 1 G = 1000 count (milli-G)
constexpr float kGyroScale = 100.0f;    // 1 deg/s = 100 count

// 数学定数（画面表示用の簡易姿勢計算で使用）。
constexpr float kRadToDeg = 57.29577951308232f;

}  // namespace config
}  // namespace meister_imu

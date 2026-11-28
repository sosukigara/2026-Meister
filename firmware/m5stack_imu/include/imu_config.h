#pragma once

#include <stdint.h>

namespace meister_imu {
namespace config {

// 送信周期とボーレート。設定の唯一の出所（R2）。
constexpr uint32_t kTxIntervalMs = 50;      // 20 Hz
constexpr uint32_t kDrawIntervalMs = 16;    // 画面更新 60 Hz 目標（オフスクリーン描画でちらつきなし）
constexpr uint32_t kSerialBaud = 115200;
// この期間以上 IMU から新鮮な値が来なければ status に kImuMissing を立てる。
constexpr uint32_t kImuFreshTimeoutMs = 1000;

// プロトコル上の固定小数点スケール（実測値ではなく表現の選択）。
// 出典: M5Unified の imu_data_t は accel = G, gyro = deg/s。
// https://context7.com/m5stack/m5unified/llms.txt
constexpr float kAccelScale = 1000.0f;  // 1 G = 1000 count (milli-G)
constexpr float kGyroScale = 100.0f;    // 1 deg/s = 100 count

// 数学定数（画面表示用の簡易姿勢計算で使用）。
constexpr float kRadToDeg = 57.29577951308232f;

// yaw 積分のガード値。長い停顿や縦向きでは積分が発散するため制限する。
constexpr float kYawMaxDtS = 1.0f;
constexpr float kYawMinCosPitch = 0.2f;
// 起動直後に静止している間にジャイロの零点を平均して取る時間。
constexpr uint32_t kYawCalibrationMs = 1500;
// 静止検出時のバイアス追従係数（1 サンプルあたり）。ドリフトを抑えるためゆっくり追従する。
constexpr float kYawBiasAdaptRate = 0.0005f;
// 静止とみなす角速度の閾値 [deg/s]。
constexpr float kYawStillGyroThreshold = 2.0f;
// ジャイロの一次ローパス係数。積分のランダムウォークを抑える。
constexpr float kYawGyroLowpassAlpha = 0.30f;

}  // namespace config
}  // namespace meister_imu

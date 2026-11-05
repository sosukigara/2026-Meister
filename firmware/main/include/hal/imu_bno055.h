/*
 * imu_bno055.h — GY-BNO055（BNO055）yaw 読み出し
 *
 * Adafruit_BNO055 等の外部ライブラリは使わず素の Wire で読む。
 * 理由: 読むのは heading 2B + CALIB 1B だけで、依存追加に見合わない。
 * NDOF 融合・I2C の面倒は BNO055 側が持つ。呼び出し側は設定を知らない。
 */
#pragma once

#include <stdint.h>

namespace meister {
namespace hal {
namespace imu_bno055 {

/// BNO055 を NDOF で初期化する。sda/scl/addr は config の kImu* を渡す。
/// 未接続でも false を返すだけでハングしない。begin 失敗後の read は false。
bool begin(uint8_t sda, uint8_t scl, uint8_t addr);

/// 接続が確立しているか（begin 成功 + chip ID 照合済み）
bool ready();

/// heading を 0.1° 単位（0..3600）と CALIB_STAT 生値を読む。
/// I2C 失敗時は false（呼び出し側はフレームを送らない）。
bool readYaw(int16_t* yawTenths, uint8_t* calib);

}  // namespace imu_bno055
}  // namespace hal
}  // namespace meister

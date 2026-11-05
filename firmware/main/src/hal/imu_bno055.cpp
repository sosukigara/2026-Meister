#ifdef ARDUINO

#include "hal/imu_bno055.h"

#include <Arduino.h>
#include <Wire.h>

#include "meister_config.h"

/*
 * BNO055 レジスタ表（★要確認★ Bosch BNO055 datasheet v1.4 で照合すること。
 * web search 遮断のため記憶値で実装し、実機ベンチで chip ID と heading 変化を確認する）。
 *
 * | 名称       | アドレス | 値        |
 * |------------|----------|-----------|
 * | CHIP_ID    | 0x00     | 0xA0 期待 |
 * | EULER_H_L  | 0x1A     | heading 下位（続く 0x1B が上位）|
 * | CALIB_STAT | 0x35     | sys[7:6] gyro[5:4] acc[3:2] mag[1:0] |
 * | OPR_MODE   | 0x3D     | CONFIG=0x00 / NDOF=0x0C |
 * | PWR_MODE   | 0x3E     | NORMAL=0x00 |
 * | SYS_TRIG   | 0x3F     | RST=0x20 |
 * | PAGE_ID    | 0x07     | 0（page 0 固定）|
 *
 * 単位: Euler 1 LSB = 1/16°。heading 生値 0..5760 → 0.1° 単位 0..3600。
 * 復帰待ち: reset 後 650ms、NDOF 切替後 30ms（データシートの立上り時間。★要確認★）。
 */

namespace meister {
namespace hal {
namespace imu_bno055 {
namespace {

constexpr uint8_t kRegChipId = 0x00;
constexpr uint8_t kExpectedChipId = 0xA0;
constexpr uint8_t kRegEulerHeading = 0x1A;
constexpr uint8_t kRegCalibStat = 0x35;
constexpr uint8_t kRegOprMode = 0x3D;
constexpr uint8_t kOprModeConfig = 0x00;
constexpr uint8_t kOprModeNdof = 0x0C;
constexpr uint8_t kRegPwrMode = 0x3E;
constexpr uint8_t kPwrModeNormal = 0x00;
constexpr uint8_t kRegSysTrigger = 0x3F;
constexpr uint8_t kSysTriggerReset = 0x20;
constexpr uint8_t kRegPageId = 0x07;

uint8_t gAddr = 0;
bool gReady = false;

bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(gAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission(true) == 0;
}

bool readRegs(uint8_t reg, uint8_t* buf, size_t n) {
  Wire.beginTransmission(gAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(gAddr, static_cast<uint8_t>(n)) != n) {
    return false;
  }
  for (size_t i = 0; i < n; ++i) {
    buf[i] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

}  // namespace

bool begin(uint8_t sda, uint8_t scl, uint8_t addr) {
  gAddr = addr;
  gReady = false;
  Wire.begin(static_cast<int>(sda), static_cast<int>(scl));
  if (!writeReg(kRegSysTrigger, kSysTriggerReset)) {
    return false;  // 未接続（NACK）。650ms 待たずに抜ける。
  }
  delay(650);
  if (!writeReg(kRegPageId, 0)) {
    return false;
  }
  uint8_t chip = 0;
  if (!readRegs(kRegChipId, &chip, 1) || chip != kExpectedChipId) {
    return false;
  }
  if (!writeReg(kRegPwrMode, kPwrModeNormal)) {
    return false;
  }
  delay(50);
  if (!writeReg(kRegOprMode, kOprModeNdof)) {
    return false;
  }
  delay(30);
  gReady = true;
  return true;
}

bool ready() { return gReady; }

bool readYaw(int16_t* yawTenths, uint8_t* calib) {
  if (!gReady || yawTenths == nullptr || calib == nullptr) {
    return false;
  }
  uint8_t raw[2];
  if (!readRegs(kRegEulerHeading, raw, sizeof(raw))) {
    gReady = false;
    return false;
  }
  const int16_t heading =
      static_cast<int16_t>(static_cast<uint16_t>(raw[0]) |
                           (static_cast<uint16_t>(raw[1]) << 8));
  *yawTenths = static_cast<int16_t>((static_cast<int32_t>(heading) * 5 + 4) / 8);
  if (!readRegs(kRegCalibStat, calib, 1)) {
    gReady = false;
    return false;
  }
  return true;
}

}  // namespace imu_bno055
}  // namespace hal
}  // namespace meister

#endif  // ARDUINO

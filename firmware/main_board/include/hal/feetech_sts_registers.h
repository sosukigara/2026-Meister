/*
 * feetech_sts_registers.h — STS/SCS 系シリアルバスサーボの仕様
 *
 * 出典: Feetech 公式 Python SDK `feetech-servo-sdk` 1.0.0
 *       scservo_sdk/scservo_def.py / scservo_sdk/protocol_packet_handler.py
 *
 * SDK を実際に走らせて採った golden:
 *   PING  id=1        FF FF 01 02 01 FB
 *   READ  55/1        FF FF 05 04 02 37 01 BC
 *   READ  55/2        FF FF 05 04 02 37 02 BB
 *   WRITE 42/1B       FF FF 05 04 03 2A 00 C9
 *   WRITE 42/2B       FF FF 05 05 03 2A E8 03 DD
 *   SYNC_WRITE 2台    FF FF FE 0A 83 2A 02 05 E8 03 06 00 00 52
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace meister {
namespace feetech {

// ===========================================================================
// フレーム
// ===========================================================================
//   [0] 0xFF          HEADER0        scservo_def.py: PKT_HEADER0
//   [1] 0xFF          HEADER1        scservo_def.py: PKT_HEADER1
//   [2] id            0x01..0xFC     0xFE = 一斉送信
//   [3] length        length フィールドより後ろのバイト数
//                     = 命令 + パラメータ + チェックサム
//   [4] instruction   命令コード    応答時は error に同じオフセットを使う
//   [5..] parameters
//   [L-1] checksum
//
//   総長 = length + 4（+4 は HEADER0/HEADER1/ID/LENGTH 自体）
//   checksum = ~(sum(tx[2 .. 総長-2])) & 0xFF
//             XOR ではない。protocol_packet_handler.py txPacket() より。
constexpr uint8_t kHeader0 = 0xFF;
constexpr uint8_t kHeader1 = 0xFF;

constexpr size_t kOffsetId     = 2;
constexpr size_t kOffsetLength = 3;
constexpr size_t kOffsetCmd    = 4;  ///< 送信 = instruction / 応答 = error
constexpr size_t kOffsetParam0 = 5;

/// length フィールドの直前にある 4 バイト（HEADER0/HEADER1/ID/LENGTH）
constexpr size_t kLengthFieldSize = 4;
constexpr size_t kMinPacketSize   = 6;  ///< PING
/// protocol_packet_handler.py:5 TXPACKET_MAX_LEN / RXPACKET_MAX_LEN と同値
constexpr size_t kMaxPacketSize   = 250;

constexpr uint8_t kIdMin       = 0x01;
constexpr uint8_t kIdMax       = 0xFC;
constexpr uint8_t kIdBroadcast = 0xFE;

// ===========================================================================
// 命令（scservo_def.py INST_*）
//
// WRITE には長さバイトが無い。READ にはあるが WRITE にはないという非対称がある。
// WRITE の長さは LENGTH からしか分からない。
// ===========================================================================
enum Instruction : uint8_t {
  kInstPing      = 0x01,  ///< params なし                     LENGTH = 2
  kInstRead      = 0x02,  ///< addr, len                       LENGTH = 4
  kInstWrite     = 0x03,  ///< addr, data（長さバイト無し）    LENGTH = 3 + len
  kInstRegWrite  = 0x04,  ///< 遅延実行。ACTION で確定
  kInstAction    = 0x05,  ///< params なし                     LENGTH = 2
  kInstSyncRead  = 0x82,  ///< start_addr, data_len, param     LENGTH = 4 + param_len
  kInstSyncWrite = 0x83,  ///< start_addr, data_len, param     LENGTH = 4 + param_len
};

// ===========================================================================
// 応答のエラービット（protocol_packet_handler.py ERRBIT_*）
// ===========================================================================
enum ErrorBit : uint8_t {
  kErrVoltage     = 0x01,
  kErrAngle       = 0x02,
  kErrOverheat    = 0x04,
  kErrOverCurrent = 0x08,
  kErrOverload    = 0x20,
};

// ===========================================================================
// バイト順（scservo_def.py SCS_MAKEWORD / SCS_END）
// 要確認: STS3215 がどちら側かはモデルごとに違う。確定後にここを直す。
// ===========================================================================
enum class Endianness : uint8_t {
  kLittle = 0,  ///< SCS_END == 0
  kBig    = 1,  ///< SCS_END == 1
};
constexpr Endianness kEndianness = Endianness::kLittle;

// ===========================================================================
// 要確認: レジスタマップ
//
// ★未検証★ 以下のアドレスは一次情報で取得できていない。SDK 1.0.0 にレジスタ
// マップは含まれず（モデル別モジュールが別パッケージ）、web search も遮断。
// 記憶で書いた値を確定値として使わないこと。STS3215 のデータシートで照合する。
//
// 照合が完了するまで WRITE 命令は使わない。READ も Present Voltage 1 バイト
// だけで確認する。kRegisterMapVerified を true にするのは照合後だけ。
// ===========================================================================
namespace reg {

constexpr bool kRegisterMapVerified = false;

/// アドレス未取得の印。この値が設定されている項目は使用禁止
constexpr uint8_t kAddrUnknown = 0xFF;

/// EEPROM（命令系。Torque Enable が OFF の間は書けない）
namespace eprom {
constexpr uint8_t kBaud         = 3;
constexpr uint8_t kMinVoltLimit = 5;   ///< 0.1 V
constexpr uint8_t kMaxVoltLimit = 6;   ///< 0.1 V
constexpr uint8_t kVirtualDiv   = 38;
constexpr uint8_t kTorqueEnable = 40;  ///< 0=脱力 1=励磁
constexpr uint8_t kAcceleration = 41;
constexpr uint8_t kGoalPosL     = 42;
constexpr uint8_t kGoalPosH     = 43;
constexpr uint8_t kGoalSpeed    = 44;
constexpr uint8_t kGoalTime     = 46;
constexpr uint8_t kReturnDelay  = kAddrUnknown;  ///< 未取得。使用禁止
}  // namespace eprom

/// SRAM（計測系。常に読める）
namespace sram {
constexpr uint8_t kPresentVolt  = 55;  ///< 0.1 V（Phase 1 で最初に照合する）
constexpr uint8_t kPresentPosL  = 56;
constexpr uint8_t kPresentPosH  = 57;
constexpr uint8_t kPresentSpeed = 58;
constexpr uint8_t kPresentLoad  = 60;
constexpr uint8_t kRealTemp     = 62;  ///< 1 °C
}  // namespace sram

constexpr uint8_t kTorqueOff = 0;
constexpr uint8_t kTorqueOn  = 1;

/// 要確認: 1 revolution あたりのステップ数。0..4095 = 0..360° が一般的。
/// ここが 0..1000 型だと角度が 4 倍ズレる。かつ PWM 側でクランプされるので
/// 静かに不正な角度になる。照合するまで角度→ステップ変換は使わない。
constexpr uint16_t kStepsPerRev  = 4096;
constexpr float kDegreesPerStep = 360.0f / static_cast<float>(kStepsPerRev);

}  // namespace reg

/// 角度 [deg] → ステップ値。範囲外は丸める。
inline uint16_t DegreesToSteps(float degrees) {
  float steps = degrees / reg::kDegreesPerStep;
  if (steps < 0.0f) {
    steps = 0.0f;
  }
  const float maxSteps = static_cast<float>(reg::kStepsPerRev - 1);
  if (steps > maxSteps) {
    steps = maxSteps;
  }
  return static_cast<uint16_t>(steps + 0.5f);
}

/// ステップ値 → 角度 [deg]
inline float StepsToDegrees(uint16_t steps) {
  return static_cast<float>(steps) * reg::kDegreesPerStep;
}

}  // namespace feetech
}  // namespace meister

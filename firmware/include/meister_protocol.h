/*
 * meister_protocol.h — Meister ESP32 ⇔ PC バイナリプロトコル定義
 *
 * 半分散型アーキテクチャ（PC = ROS 2 / ESP32 = リアルタイム PWM 制御）の
 * UART 通信プロトコル。カスタムバイナリ・固定長フレーム + XOR チェックサム。
 * テキストベース（JSON 等）はデバッグに有利だが非決定的なため不採用。
 *
 * ============================================================================
 * バイトレイアウト（全フレーム共通、リトルエンディアン）
 * ============================================================================
 *
 *   [0]      ヘッダ      0xA5（同期バイト）
 *   [1]      種別        TypeId（1 バイト）
 *   [2..L-2] ペイロード  種別ごとに固定長
 *   [L-1]    チェックサム  ペイロード末尾までの全バイトの XOR
 *
 *   フレーム長 L = 2 + PayloadSize(種別) + 1（種別から一意に決まる固定長）
 *
 * チェックサム:
 *   checksum = buf[0] ^ buf[1] ^ ... ^ buf[L-2]
 *   検証: ComputeChecksum(フレーム, L) == 0（チェックサムも含めて XOR すると 0）
 *
 * ----------------------------------------------------------------------------
 * PC → ESP32 コマンドフレーム（下り）
 * ----------------------------------------------------------------------------
 * | 種別                 | ID   | ペイロード                                  | フレーム長 |
 * |----------------------|------|---------------------------------------------|-----------|
 * | CMD_MOTOR_VELOCITY   | 0x01 | int16 vel[6]  各輪速度 -1000..+1000 (千分率) | 15        |
 * | CMD_STEERING_ANGLE   | 0x02 | int16 ang[6]  舵角 -900..+900 (0.1° 単位)    | 15        |
 * | CMD_ARM_ANGLE        | 0x03 | int16 ang[4]  関節角 0..1800 (0.1° 単位)     | 11        |
 * | CMD_GRIPPER          | 0x04 | uint8 cmd     0=閉 1=開 2=停止               | 4         |
 * ----------------------------------------------------------------------------
 * ESP32 → PC フィードバックフレーム（上り）
 * ----------------------------------------------------------------------------
 * | 種別        | ID   | ペイロード                                      | フレーム長 |
 * |-------------|------|--------------------------------------------------|-----------|
 * | FB_STATE    | 0x81 | int16 enc[6] + uint8 state + uint8 error_flags  | 17        |
 * | FB_ERROR    | 0x82 | uint8 error_code                                | 4         |
 * ============================================================================
 *
 * 値の範囲: 速度/角度は実数値を 10 倍（0.1°）した int16 で送る。
 * エンコード/デコードはすべてリトルエンディアンで固定。
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace meister {
namespace proto {

// ---------------------------------------------------------------------------
// 同期ヘッダ
// ---------------------------------------------------------------------------
constexpr uint8_t kHeaderByte = 0xA5;  ///< フレーム先頭の同期バイト

// ---------------------------------------------------------------------------
// 種別 ID（TypeId）
// ---------------------------------------------------------------------------
enum TypeId : uint8_t {
  kTypeUnknown       = 0x00,
  kCmdMotorVelocity  = 0x01,  ///< 駆動モータ速度指令（6 輪）
  kCmdSteeringAngle  = 0x02,  ///< ステアリング舵角指令（6 輪）
  kCmdArmAngle       = 0x03,  ///< アームサーボ角度指令（4 軸）
  kCmdGripper        = 0x04,  ///< グリッパー開閉指令
  kFbState           = 0x81,  ///< 状態フィードバック（エンコーダ/状態/エラー）
  kFbError           = 0x82,  ///< エラー通知
};

// ---------------------------------------------------------------------------
// 台数・値域
// ---------------------------------------------------------------------------
constexpr size_t kNumDriveMotors    = 6;  ///< 駆動モータ数
constexpr size_t kNumSteeringServos = 6;  ///< ステアリングサーボ数
constexpr size_t kNumArmServos      = 4;  ///< アームサーボ数

constexpr int16_t kMinVelocity = -1000;   ///< 速度下限（-100.0%）
constexpr int16_t kMaxVelocity =  1000;   ///< 速度上限（+100.0%）
constexpr int16_t kMinSteering =  -900;   ///< 舵角下限（-90.0°）
constexpr int16_t kMaxSteering =   900;   ///< 舵角上限（+90.0°）
constexpr int16_t kMinArmAngle  =     0;  ///< アーム関節角下限（0.0°）
constexpr int16_t kMaxArmAngle  =  1800;  ///< アーム関節角上限（180.0°）

// グリッパー指令値
enum GripperCommand : uint8_t {
  kGripperClose = 0x00,
  kGripperOpen  = 0x01,
  kGripperStop  = 0x02,
};

// フィードバック: エラーフラグ（FB_STATE の error_flags）
enum FbErrorFlags : uint8_t {
  kFbStateNormal   = 0x00,
  kFbErrorMotor    = 0x01,  ///< モータ系エラー
  kFbErrorServo    = 0x02,  ///< サーボ系エラー
  kFbErrorSensor   = 0x04,  ///< センサ系エラー
  kFbErrorProtocol = 0x08,  ///< プロトコルエラー（フレーム破損等）
};

// ---------------------------------------------------------------------------
// ペイロードサイズ・フレームサイズ（種別ごとに固定）
// ---------------------------------------------------------------------------
constexpr size_t kPayloadMotorVelocity = kNumDriveMotors * sizeof(int16_t);   // 12
constexpr size_t kPayloadSteeringAngle = kNumSteeringServos * sizeof(int16_t); // 12
constexpr size_t kPayloadArmAngle      = kNumArmServos * sizeof(int16_t);      // 8
constexpr size_t kPayloadGripper       = 1;
constexpr size_t kPayloadState         = kNumDriveMotors * sizeof(int16_t) + 2; // 14
constexpr size_t kPayloadError         = 1;

constexpr size_t kMaxPayloadSize = kPayloadState;             // 14
constexpr size_t kMaxFrameSize   = 1 + 1 + kMaxPayloadSize + 1;  // 17

/// 種別に対応するペイロードサイズを返す（未知種別は 0）
inline constexpr size_t PayloadSize(TypeId type) {
  switch (type) {
    case kCmdMotorVelocity: return kPayloadMotorVelocity;
    case kCmdSteeringAngle: return kPayloadSteeringAngle;
    case kCmdArmAngle:      return kPayloadArmAngle;
    case kCmdGripper:       return kPayloadGripper;
    case kFbState:          return kPayloadState;
    case kFbError:          return kPayloadError;
    default:                return 0;
  }
}

/// 種別に対応するフレームサイズ（ヘッダ + 種別 + ペイロード + チェックサム）
inline constexpr size_t FrameSize(TypeId type) {
  return 1 + 1 + PayloadSize(type) + 1;
}

// ---------------------------------------------------------------------------
// デコード結果
// ---------------------------------------------------------------------------
enum class ParseResult : uint8_t {
  kOk,          ///< 正常にデコード
  kBadHeader,   ///< ヘッダバイト不一致
  kBadChecksum, ///< チェックサム不一致
  kUnknownType, ///< 未知の種別 ID
  kNeedMore,    ///< バッファにフレームが揃っていない
};

/// ParseResult の表示名（ログ・テスト用）
inline const char* ParseResultName(ParseResult r) {
  switch (r) {
    case ParseResult::kOk:          return "ok";
    case ParseResult::kBadHeader:   return "bad header";
    case ParseResult::kBadChecksum: return "bad checksum";
    case ParseResult::kUnknownType: return "unknown type";
    case ParseResult::kNeedMore:    return "need more data";
    default:                        return "?";
  }
}

// ---------------------------------------------------------------------------
// デコード済みフレーム
// ---------------------------------------------------------------------------
struct Frame {
  TypeId type = kTypeUnknown;
  uint8_t payload[kMaxPayloadSize] = {};  ///< ペイロードの生バイト
  size_t payloadSize = 0;
  size_t frameSize = 0;
};

// ---------------------------------------------------------------------------
// 関数プロトタイプ
// ---------------------------------------------------------------------------

/// データ列（len バイト）の XOR チェックサムを計算する
uint8_t ComputeChecksum(const uint8_t* data, size_t len);

/// フレーム全体（チェックサム含む）の整合性を検証する
/// 全バイトの XOR が 0 になれば正常
bool CheckChecksum(const uint8_t* frame, size_t frameSize);

// ---- エンコード（コマンド: PC → ESP32 / フィードバック: ESP32 → PC）----
/// 成功時はフレーム長、バッファ不足時は 0 を返す
size_t EncodeMotorVelocity(uint8_t* buf, size_t cap, const int16_t (&vel)[kNumDriveMotors]);
size_t EncodeSteeringAngle(uint8_t* buf, size_t cap, const int16_t (&angle)[kNumSteeringServos]);
size_t EncodeArmAngle(uint8_t* buf, size_t cap, const int16_t (&angle)[kNumArmServos]);
size_t EncodeGripper(uint8_t* buf, size_t cap, GripperCommand cmd);
size_t EncodeState(uint8_t* buf, size_t cap, const int16_t (&encoder)[kNumDriveMotors],
                   uint8_t state, uint8_t errorFlags);
size_t EncodeError(uint8_t* buf, size_t cap, uint8_t errorCode);

// ---- デコード・検証 ----
/// バッファ先頭のフレームを検証・デコードする。
/// 正常時 kOk を返し out に結果を格納。フレーム未達は kNeedMore。
ParseResult ParseFrame(const uint8_t* data, size_t len, Frame* out);

// ---- ペイロードアクセサ（デコード後）----
/// ペイロード内オフセット（バイト）から int16（リトルエンディアン）を読む
int16_t FrameGetInt16(const Frame& frame, size_t offset);
/// ペイロード内オフセット（バイト）から uint8 を読む
uint8_t FrameGetU8(const Frame& frame, size_t offset);

}  // namespace proto
}  // namespace meister

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "hal/generated_config.h"

namespace meister {
namespace proto {

constexpr uint8_t kHeaderByte = 0xA5;  ///< フレーム先頭の同期バイト

enum TypeId : uint8_t {
  kTypeUnknown       = 0x00,
  kCmdMotorVelocity  = 0x01,  ///< 駆動モータ速度指令（6 輪）
  kCmdSteeringAngle  = 0x02,  ///< ステアリング舵角指令（6 輪）
  kCmdArmAngle       = 0x03,  ///< アームサーボ角度指令（4 軸）
  kCmdGripper        = 0x04,  ///< グリッパー開閉指令
  kFbState           = 0x81,  ///< 状態フィードバック（エンコーダ/状態/エラー）
  kFbError           = 0x82,  ///< エラー通知
  kFbImu             = 0x83,  ///< IMU 方位（yaw + キャリブ状態）
};

// 値は config/meister_robot.yaml が唯一の出所で、tools/gen_config.py が
// firmware/main_board/include/hal/generated_config.h に生成する。ここでは再定義せず
// using で参照するだけにして、値が 2 か所に存在しないようにする。
using config::kNumDriveMotors;
using config::kNumSteeringServos;
using config::kNumArmServos;
using config::kNumGripperServos;
using config::kMinVelocity;
using config::kMaxVelocity;
using config::kMinSteering;
using config::kMaxSteering;
using config::kMinArmAngle;
using config::kMaxArmAngle;
using config::kMinRocker;
using config::kMaxRocker;
using config::kMinShoulder;
using config::kMaxShoulder;

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

constexpr size_t kPayloadMotorVelocity = kNumDriveMotors * sizeof(int16_t);   // 12
constexpr size_t kPayloadSteeringAngle = kNumSteeringServos * sizeof(int16_t); // 12
constexpr size_t kPayloadArmAngle      = kNumArmServos * sizeof(int16_t);      // 8
constexpr size_t kPayloadGripper       = 1;
constexpr size_t kPayloadState         = kNumDriveMotors * sizeof(int16_t) + 2; // 14
constexpr size_t kPayloadError         = 1;
constexpr size_t kPayloadImu           = sizeof(int16_t) + 1;  // yaw + calib = 3

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
    case kFbImu:            return kPayloadImu;
    default:                return 0;
  }
}

inline constexpr size_t FrameSize(TypeId type) {
  return 1 + 1 + PayloadSize(type) + 1;
}

enum class ParseResult : uint8_t {
  kOk,          ///< 正常にデコード
  kBadHeader,   ///< ヘッダバイト不一致
  kBadChecksum, ///< チェックサム不一致
  kUnknownType, ///< 未知の種別 ID
  kNeedMore,    ///< バッファにフレームが揃っていない
};

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

struct Frame {
  TypeId type = kTypeUnknown;
  uint8_t payload[kMaxPayloadSize] = {};  ///< ペイロードの生バイト
  size_t payloadSize = 0;
  size_t frameSize = 0;
};

/// データ列（len バイト）の XOR チェックサムを計算する
uint8_t ComputeChecksum(const uint8_t* data, size_t len);

/// 全バイトの XOR が 0 になれば正常
bool CheckChecksum(const uint8_t* frame, size_t frameSize);

/// 成功時はフレーム長、バッファ不足時は 0 を返す
size_t EncodeMotorVelocity(uint8_t* buf, size_t cap, const int16_t (&vel)[kNumDriveMotors]);
size_t EncodeSteeringAngle(uint8_t* buf, size_t cap, const int16_t (&angle)[kNumSteeringServos]);
size_t EncodeArmAngle(uint8_t* buf, size_t cap, const int16_t (&angle)[kNumArmServos]);
size_t EncodeGripper(uint8_t* buf, size_t cap, GripperCommand cmd);
size_t EncodeState(uint8_t* buf, size_t cap, const int16_t (&encoder)[kNumDriveMotors],
                   uint8_t state, uint8_t errorFlags);
size_t EncodeError(uint8_t* buf, size_t cap, uint8_t errorCode);
/// yaw_tenths: BNO055 heading の生値（0.1° 単位 0..3600）。calib: CALIB_STAT 生値。
size_t EncodeImu(uint8_t* buf, size_t cap, int16_t yawTenths, uint8_t calib);

/// 正常時 kOk を返し out に結果を格納。フレーム未達は kNeedMore。
ParseResult ParseFrame(const uint8_t* data, size_t len, Frame* out);

/// ペイロード内オフセット（バイト）から int16（リトルエンディアン）を読む
int16_t FrameGetInt16(const Frame& frame, size_t offset);
/// ペイロード内オフセット（バイト）から uint8 を読む
uint8_t FrameGetU8(const Frame& frame, size_t offset);

}  // namespace proto
}  // namespace meister

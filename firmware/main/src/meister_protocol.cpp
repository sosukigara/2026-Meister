/*
 * meister_protocol.cpp — バイナリプロトコルのエンコード/デコード実装
 *
 * フレーム形式（リトルエンディアン、固定長）:
 *   [ヘッダ 0xA5][種別][ペイロード][チェックサム]
 *   チェックサム = ペイロード末尾までの全バイト XOR
 *   検証: チェックサムを含む全バイトの XOR == 0
 *
 * このファイルは Arduino 非依存（ホスト側テストでもそのままコンパイル可能）。
 */
#include "meister_protocol.h"

#include <string.h>

namespace meister {
namespace proto {

// ---------------------------------------------------------------------------
// 内部ヘルパ: リトルエンディアン int16 の書込/読出
// ---------------------------------------------------------------------------
namespace {

void PutI16(uint8_t* dst, int16_t v) {
  dst[0] = static_cast<uint8_t>(v & 0xFF);
  dst[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

int16_t GetI16(const uint8_t* src) {
  return static_cast<int16_t>(static_cast<uint16_t>(src[0]) |
                              (static_cast<uint16_t>(src[1]) << 8));
}

/// 汎用エンコーダ: [ヘッダ][種別][payload(固定長)][チェックサム] を組み立てる
size_t BuildFrame(uint8_t* buf, size_t cap, TypeId type,
                  const uint8_t* payload, size_t payloadSize) {
  const size_t frameSize = FrameSize(type);
  if (cap < frameSize || payloadSize != PayloadSize(type)) {
    return 0;
  }
  size_t i = 0;
  buf[i++] = kHeaderByte;
  buf[i++] = static_cast<uint8_t>(type);
  for (size_t p = 0; p < payloadSize; ++p) {
    buf[i++] = payload[p];
  }
  // チェックサム = 先頭からペイロード末尾までの XOR
  buf[frameSize - 1] = ComputeChecksum(buf, frameSize - 1);
  return frameSize;
}

}  // namespace

// ---------------------------------------------------------------------------
// チェックサム
// ---------------------------------------------------------------------------
uint8_t ComputeChecksum(const uint8_t* data, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; ++i) {
    sum ^= data[i];
  }
  return sum;
}

bool CheckChecksum(const uint8_t* frame, size_t frameSize) {
  if (frameSize == 0) {
    return false;
  }
  // チェックサムも含めて XOR すると 0 になる
  return ComputeChecksum(frame, frameSize) == 0;
}

// ---------------------------------------------------------------------------
// エンコード: コマンド
// ---------------------------------------------------------------------------
size_t EncodeMotorVelocity(uint8_t* buf, size_t cap,
                           const int16_t (&vel)[kNumDriveMotors]) {
  uint8_t payload[kPayloadMotorVelocity];
  for (size_t i = 0; i < kNumDriveMotors; ++i) {
    PutI16(&payload[i * 2], vel[i]);
  }
  return BuildFrame(buf, cap, kCmdMotorVelocity, payload, sizeof(payload));
}

size_t EncodeSteeringAngle(uint8_t* buf, size_t cap,
                           const int16_t (&angle)[kNumSteeringServos]) {
  uint8_t payload[kPayloadSteeringAngle];
  for (size_t i = 0; i < kNumSteeringServos; ++i) {
    PutI16(&payload[i * 2], angle[i]);
  }
  return BuildFrame(buf, cap, kCmdSteeringAngle, payload, sizeof(payload));
}

size_t EncodeArmAngle(uint8_t* buf, size_t cap,
                      const int16_t (&angle)[kNumArmServos]) {
  uint8_t payload[kPayloadArmAngle];
  for (size_t i = 0; i < kNumArmServos; ++i) {
    PutI16(&payload[i * 2], angle[i]);
  }
  return BuildFrame(buf, cap, kCmdArmAngle, payload, sizeof(payload));
}

size_t EncodeGripper(uint8_t* buf, size_t cap, GripperCommand cmd) {
  const uint8_t payload[1] = {static_cast<uint8_t>(cmd)};
  return BuildFrame(buf, cap, kCmdGripper, payload, sizeof(payload));
}

// ---------------------------------------------------------------------------
// エンコード: フィードバック
// ---------------------------------------------------------------------------
size_t EncodeState(uint8_t* buf, size_t cap,
                   const int16_t (&encoder)[kNumDriveMotors],
                   uint8_t state, uint8_t errorFlags) {
  uint8_t payload[kPayloadState];
  for (size_t i = 0; i < kNumDriveMotors; ++i) {
    PutI16(&payload[i * 2], encoder[i]);
  }
  payload[kNumDriveMotors * 2]     = state;
  payload[kNumDriveMotors * 2 + 1] = errorFlags;
  return BuildFrame(buf, cap, kFbState, payload, sizeof(payload));
}

size_t EncodeError(uint8_t* buf, size_t cap, uint8_t errorCode) {
  const uint8_t payload[1] = {errorCode};
  return BuildFrame(buf, cap, kFbError, payload, sizeof(payload));
}

// ---------------------------------------------------------------------------
// デコード・検証
// ---------------------------------------------------------------------------
ParseResult ParseFrame(const uint8_t* data, size_t len, Frame* out) {
  if (out == nullptr) {
    return ParseResult::kUnknownType;
  }
  // ヘッダと種別が読めるか（最低 2 バイト）
  if (len < 2) {
    return ParseResult::kNeedMore;
  }
  if (data[0] != kHeaderByte) {
    return ParseResult::kBadHeader;
  }
  const TypeId type = static_cast<TypeId>(data[1]);
  const size_t payloadSize = PayloadSize(type);
  if (payloadSize == 0) {
    return ParseResult::kUnknownType;
  }
  const size_t frameSize = FrameSize(type);  // = 2 + payloadSize + 1
  if (len < frameSize) {
    return ParseResult::kNeedMore;
  }
  if (!CheckChecksum(data, frameSize)) {
    return ParseResult::kBadChecksum;
  }
  out->type = type;
  out->payloadSize = payloadSize;
  out->frameSize = frameSize;
  memcpy(out->payload, &data[2], payloadSize);
  return ParseResult::kOk;
}

// ---------------------------------------------------------------------------
// ペイロードアクセサ
// ---------------------------------------------------------------------------
int16_t FrameGetInt16(const Frame& frame, size_t offset) {
  if (offset + sizeof(int16_t) > frame.payloadSize) {
    return 0;
  }
  return GetI16(&frame.payload[offset]);
}

uint8_t FrameGetU8(const Frame& frame, size_t offset) {
  if (offset >= frame.payloadSize) {
    return 0;
  }
  return frame.payload[offset];
}

}  // namespace proto
}  // namespace meister

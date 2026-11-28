#pragma once

#include <stddef.h>
#include <stdint.h>

namespace meister_imu {
namespace protocol {

constexpr uint8_t kHeader = 0xA5;
constexpr uint8_t kTypeImu6Axis = 0x84;
constexpr size_t kPayloadSize = 19;  // timestamp(4) + 6 axes(12) + yaw(2) + status(1)
constexpr size_t kFrameSize = 1 + 1 + kPayloadSize + 1;

enum StatusBits : uint8_t {
  kClamped = 0x01,    // 固定小数点の範囲外をクランプした
  kImuMissing = 0x02  // M5.Imu.update() が新鮮な値を返さなかった
};

struct Sample {
  uint32_t timestamp_ms = 0;
  int16_t accel[3] = {0, 0, 0};  // milli-G
  int16_t gyro[3] = {0, 0, 0};   // centi-deg/s
  uint16_t yaw_centideg = 0;     // 積分 yaw [centi-deg], 0..36000
  uint8_t status = 0;
};

uint8_t ComputeChecksum(const uint8_t* data, size_t len);

// フレーム全バイトの XOR が 0 なら正常（main_board と同じ方式）。
bool CheckFrame(const uint8_t* frame, size_t len);

// 成功時は kFrameSize、バッファ不足時は 0。
size_t EncodeFrame(uint8_t* buf, size_t cap, const Sample& sample);

// ヘッダ・種別・チェックサム・長さを検証してから値を取り出す。
bool DecodeFrame(const uint8_t* data, size_t len, Sample* out);

}  // namespace protocol
}  // namespace meister_imu

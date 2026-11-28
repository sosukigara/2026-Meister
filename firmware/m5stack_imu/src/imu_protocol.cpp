#include "imu_protocol.h"

namespace meister_imu {
namespace protocol {
namespace {

void PutU32(uint8_t* dst, uint32_t v) {
  for (size_t i = 0; i < 4; ++i) {
    dst[i] = static_cast<uint8_t>(v >> (8 * i));
  }
}

uint32_t GetU32(const uint8_t* src) {
  uint32_t v = 0;
  for (size_t i = 0; i < 4; ++i) {
    v |= static_cast<uint32_t>(src[i]) << (8 * i);
  }
  return v;
}

void PutI16(uint8_t* dst, int16_t v) {
  const uint16_t u = static_cast<uint16_t>(v);
  dst[0] = static_cast<uint8_t>(u & 0xFF);
  dst[1] = static_cast<uint8_t>(u >> 8);
}

int16_t GetI16(const uint8_t* src) {
  const uint16_t u = static_cast<uint16_t>(src[0]) |
                     (static_cast<uint16_t>(src[1]) << 8);
  return static_cast<int16_t>(u);
}

}  // namespace

uint8_t ComputeChecksum(const uint8_t* data, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; ++i) {
    sum ^= data[i];
  }
  return sum;
}

bool CheckFrame(const uint8_t* frame, size_t len) {
  if (frame == nullptr || len != kFrameSize) {
    return false;
  }
  return ComputeChecksum(frame, len) == 0;
}

size_t EncodeFrame(uint8_t* buf, size_t cap, const Sample& sample) {
  if (buf == nullptr || cap < kFrameSize) {
    return 0;
  }

  buf[0] = kHeader;
  buf[1] = kTypeImu6Axis;
  PutU32(&buf[2], sample.timestamp_ms);
  for (size_t i = 0; i < 3; ++i) {
    PutI16(&buf[6 + 2 * i], sample.accel[i]);
    PutI16(&buf[12 + 2 * i], sample.gyro[i]);
  }
  PutI16(&buf[18], static_cast<int16_t>(sample.yaw_centideg));
  buf[20] = sample.status;
  buf[21] = ComputeChecksum(buf, kFrameSize - 1);
  return kFrameSize;
}

bool DecodeFrame(const uint8_t* data, size_t len, Sample* out) {
  if (out == nullptr || !CheckFrame(data, len) || data[1] != kTypeImu6Axis) {
    return false;
  }

  out->timestamp_ms = GetU32(&data[2]);
  for (size_t i = 0; i < 3; ++i) {
    out->accel[i] = GetI16(&data[6 + 2 * i]);
    out->gyro[i] = GetI16(&data[12 + 2 * i]);
  }
  out->yaw_centideg = static_cast<uint16_t>(GetI16(&data[18]));
  out->status = data[20];
  return true;
}

}  // namespace protocol
}  // namespace meister_imu

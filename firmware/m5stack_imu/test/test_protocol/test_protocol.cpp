/*
 * 6軸シリアルフレームのホスト側ユニットテスト（Unity）
 * 実行: `pio test -e native --filter test_protocol`
 */
#include <unity.h>

#include <climits>
#include <string.h>

#include "imu_protocol.h"

using namespace meister_imu;

namespace {

protocol::Sample NominalSample() {
  protocol::Sample s{};
  s.timestamp_ms = 0x01020304u;
  s.accel[0] = 500;
  s.accel[1] = -250;
  s.accel[2] = 1000;
  s.gyro[0] = 1000;
  s.gyro[1] = -2000;
  s.gyro[2] = 3000;
  s.status = 0x00;
  return s;
}

uint32_t ReadU32LE(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

int16_t ReadI16LE(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}

}  // namespace

void test_encode_layout_and_checksum(void) {
  uint8_t buf[protocol::kFrameSize] = {};
  const size_t n = protocol::EncodeFrame(buf, sizeof(buf), NominalSample());

  TEST_ASSERT_EQUAL_UINT(protocol::kFrameSize, n);
  TEST_ASSERT_EQUAL_UINT8(protocol::kHeader, buf[0]);
  TEST_ASSERT_EQUAL_UINT8(protocol::kTypeImu6Axis, buf[1]);
  TEST_ASSERT_EQUAL_UINT32(0x01020304u, ReadU32LE(&buf[2]));
  TEST_ASSERT_EQUAL_INT16(500, ReadI16LE(&buf[6]));
  TEST_ASSERT_EQUAL_INT16(-250, ReadI16LE(&buf[8]));
  TEST_ASSERT_EQUAL_INT16(1000, ReadI16LE(&buf[10]));
  TEST_ASSERT_EQUAL_INT16(1000, ReadI16LE(&buf[12]));
  TEST_ASSERT_EQUAL_INT16(-2000, ReadI16LE(&buf[14]));
  TEST_ASSERT_EQUAL_INT16(3000, ReadI16LE(&buf[16]));
  TEST_ASSERT_EQUAL_UINT8(0x00, buf[18]);
  TEST_ASSERT_TRUE(protocol::CheckFrame(buf, n));
}

void test_decode_roundtrip(void) {
  uint8_t buf[protocol::kFrameSize] = {};
  const size_t n = protocol::EncodeFrame(buf, sizeof(buf), NominalSample());
  protocol::Sample out{};

  TEST_ASSERT_TRUE(protocol::DecodeFrame(buf, n, &out));
  TEST_ASSERT_EQUAL_UINT32(0x01020304u, out.timestamp_ms);
  for (size_t i = 0; i < 3; ++i) {
    TEST_ASSERT_EQUAL_INT16(NominalSample().accel[i], out.accel[i]);
    TEST_ASSERT_EQUAL_INT16(NominalSample().gyro[i], out.gyro[i]);
  }
  TEST_ASSERT_EQUAL_UINT8(0x00, out.status);
}

void test_decode_negative_extremes(void) {
  protocol::Sample s{};
  s.accel[0] = INT16_MIN;
  s.accel[1] = INT16_MAX;
  s.gyro[2] = -1;
  s.status = protocol::kClamped | protocol::kImuMissing;

  uint8_t buf[protocol::kFrameSize] = {};
  TEST_ASSERT_EQUAL_UINT(protocol::kFrameSize,
                         protocol::EncodeFrame(buf, sizeof(buf), s));

  protocol::Sample decoded{};
  TEST_ASSERT_TRUE(protocol::DecodeFrame(buf, sizeof(buf), &decoded));
  TEST_ASSERT_EQUAL_INT16(INT16_MIN, decoded.accel[0]);
  TEST_ASSERT_EQUAL_INT16(INT16_MAX, decoded.accel[1]);
  TEST_ASSERT_EQUAL_INT16(-1, decoded.gyro[2]);
  TEST_ASSERT_EQUAL_UINT8(protocol::kClamped | protocol::kImuMissing,
                          decoded.status);
}

void test_reject_corrupt_frames(void) {
  uint8_t buf[protocol::kFrameSize] = {};
  const size_t n = protocol::EncodeFrame(buf, sizeof(buf), NominalSample());
  protocol::Sample out{};

  uint8_t bad_header[protocol::kFrameSize];
  memcpy(bad_header, buf, n);
  bad_header[0] = 0xA6;
  TEST_ASSERT_FALSE(protocol::CheckFrame(bad_header, n));
  TEST_ASSERT_FALSE(protocol::DecodeFrame(bad_header, n, &out));

  uint8_t bad_checksum[protocol::kFrameSize];
  memcpy(bad_checksum, buf, n);
  bad_checksum[6] ^= 0x01;
  TEST_ASSERT_FALSE(protocol::CheckFrame(bad_checksum, n));
  TEST_ASSERT_FALSE(protocol::DecodeFrame(bad_checksum, n, &out));

  uint8_t bad_type[protocol::kFrameSize];
  memcpy(bad_type, buf, n);
  bad_type[1] = 0x83;
  TEST_ASSERT_FALSE(protocol::DecodeFrame(bad_type, n, &out));

  TEST_ASSERT_FALSE(protocol::DecodeFrame(buf, n - 1, &out));
  TEST_ASSERT_FALSE(protocol::DecodeFrame(buf, 0, &out));
  TEST_ASSERT_FALSE(protocol::DecodeFrame(nullptr, n, &out));
}

void test_encode_requires_full_buffer(void) {
  uint8_t small[protocol::kFrameSize - 1] = {};
  TEST_ASSERT_EQUAL_UINT(0, protocol::EncodeFrame(small, sizeof(small),
                                                 NominalSample()));
  TEST_ASSERT_EQUAL_UINT(0, protocol::EncodeFrame(nullptr, 0, NominalSample()));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_encode_layout_and_checksum);
  RUN_TEST(test_decode_roundtrip);
  RUN_TEST(test_decode_negative_extremes);
  RUN_TEST(test_reject_corrupt_frames);
  RUN_TEST(test_encode_requires_full_buffer);
  return UNITY_END();
}

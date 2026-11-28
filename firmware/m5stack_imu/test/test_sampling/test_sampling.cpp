/*
 * IMU 生値 -> 固定小数点サンプル / 簡易姿勢のホスト側テスト（Unity）
 * 実行: `pio test -e native --filter test_sampling`
 */
#include <unity.h>

#include <climits>
#include <cmath>

#include "imu_sampling.h"

using namespace meister_imu;

namespace {

RawImu LevelSample() {
  RawImu raw{};
  raw.accel[0] = 0.0f;
  raw.accel[1] = 0.0f;
  raw.accel[2] = 1.0f;
  raw.gyro[0] = 10.0f;
  raw.gyro[1] = -20.0f;
  raw.gyro[2] = 30.0f;
  raw.timestamp_ms = 1234u;
  raw.present = true;
  return raw;
}

}  // namespace

void test_convert_nominal_values(void) {
  protocol::Sample out{};
  ConvertSample(LevelSample(), &out);

  TEST_ASSERT_EQUAL_UINT32(1234u, out.timestamp_ms);
  TEST_ASSERT_EQUAL_INT16(0, out.accel[0]);
  TEST_ASSERT_EQUAL_INT16(0, out.accel[1]);
  TEST_ASSERT_EQUAL_INT16(1000, out.accel[2]);
  TEST_ASSERT_EQUAL_INT16(1000, out.gyro[0]);
  TEST_ASSERT_EQUAL_INT16(-2000, out.gyro[1]);
  TEST_ASSERT_EQUAL_INT16(3000, out.gyro[2]);
  TEST_ASSERT_EQUAL_UINT8(0, out.status);
}

void test_convert_fractional_accel(void) {
  RawImu raw = LevelSample();
  raw.accel[0] = 0.5f;
  raw.accel[1] = -0.25f;

  protocol::Sample out{};
  ConvertSample(raw, &out);

  TEST_ASSERT_EQUAL_INT16(500, out.accel[0]);
  TEST_ASSERT_EQUAL_INT16(-250, out.accel[1]);
}

void test_convert_clamps_and_flags(void) {
  RawImu raw = LevelSample();
  raw.accel[0] = 1000.0f;
  raw.accel[1] = -1000.0f;
  raw.gyro[2] = -1e6f;

  protocol::Sample out{};
  ConvertSample(raw, &out);

  TEST_ASSERT_EQUAL_INT16(INT16_MAX, out.accel[0]);
  TEST_ASSERT_EQUAL_INT16(INT16_MIN, out.accel[1]);
  TEST_ASSERT_EQUAL_INT16(INT16_MIN, out.gyro[2]);
  TEST_ASSERT_TRUE((out.status & protocol::kClamped) != 0);
}

void test_convert_missing_imu_zeroes_values(void) {
  RawImu raw = LevelSample();
  raw.present = false;

  protocol::Sample out{};
  ConvertSample(raw, &out);

  TEST_ASSERT_EQUAL_INT16(0, out.accel[2]);
  TEST_ASSERT_EQUAL_INT16(0, out.gyro[0]);
  TEST_ASSERT_EQUAL_UINT8(protocol::kImuMissing, out.status);
}

void test_convert_non_finite_is_clamped_to_zero(void) {
  RawImu raw = LevelSample();
  raw.accel[0] = NAN;

  protocol::Sample out{};
  ConvertSample(raw, &out);

  TEST_ASSERT_EQUAL_INT16(0, out.accel[0]);
  TEST_ASSERT_TRUE((out.status & protocol::kClamped) != 0);
}

void test_attitude_level_and_tilted(void) {
  const Attitude level = EstimateAttitude(LevelSample());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, level.roll_deg);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, level.pitch_deg);

  RawImu tilted = LevelSample();
  tilted.accel[0] = 1.0f;
  tilted.accel[2] = 0.0f;
  const Attitude sideways = EstimateAttitude(tilted);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -90.0f, sideways.pitch_deg);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_convert_nominal_values);
  RUN_TEST(test_convert_fractional_accel);
  RUN_TEST(test_convert_clamps_and_flags);
  RUN_TEST(test_convert_missing_imu_zeroes_values);
  RUN_TEST(test_convert_non_finite_is_clamped_to_zero);
  RUN_TEST(test_attitude_level_and_tilted);
  return UNITY_END();
}

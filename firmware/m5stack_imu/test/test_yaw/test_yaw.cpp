/*
 * マイコン内ジャイロ積分による yaw 推定のホスト側テスト（Unity）
 * 実行: `pio test -e native --filter test_yaw`
 */
#include <unity.h>

#include <cmath>

#include "imu_yaw.h"

using namespace meister_imu;

namespace {

RawImu LevelRaw(float gz) {
  RawImu raw;
  raw.accel[2] = 1.0f;
  raw.gyro[2] = gz;
  raw.present = true;
  return raw;
}

}  // namespace

void test_integrates_constant_yaw_rate(void) {
  YawTracker yaw;
  yaw.Reset();
  yaw.Update(LevelRaw(10.0f), 1000);
  yaw.Update(LevelRaw(10.0f), 2000);

  TEST_ASSERT_FLOAT_WITHIN(0.1f, 10.0f, yaw.yaw_deg());
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 10.0f, yaw.yaw_rate_deg());
}

void test_wraps_at_360_degrees(void) {
  YawTracker yaw;
  yaw.Reset();
  yaw.Update(LevelRaw(200.0f), 0);
  yaw.Update(LevelRaw(200.0f), 1000);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 200.0f, yaw.yaw_deg());
  yaw.Update(LevelRaw(200.0f), 2000);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 40.0f, yaw.yaw_deg());
}

void test_subtracts_gyro_bias(void) {
  const float bias[3] = {0.0f, 0.0f, 5.0f};
  YawTracker yaw;
  yaw.Reset();
  yaw.SetBias(bias);
  yaw.Update(LevelRaw(10.0f), 0);
  yaw.Update(LevelRaw(10.0f), 1000);

  TEST_ASSERT_FLOAT_WITHIN(0.1f, 5.0f, yaw.yaw_rate_deg());
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 5.0f, yaw.yaw_deg());
}

void test_calibrates_bias_from_samples(void) {
  YawTracker yaw;
  yaw.Reset();
  const float a[3] = {0.0f, 0.0f, 2.0f};
  const float b[3] = {0.0f, 0.0f, 4.0f};
  const float c[3] = {0.0f, 0.0f, 6.0f};
  yaw.BiasSample(a);
  yaw.BiasSample(b);
  yaw.BiasSample(c);
  yaw.FinishBias();

  yaw.Update(LevelRaw(10.0f), 0);
  yaw.Update(LevelRaw(10.0f), 1000);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 6.0f, yaw.yaw_rate_deg());
}

void test_tilt_compensates_yaw_rate(void) {
  RawImu rolled;
  rolled.present = true;
  rolled.accel[0] = 0.0f;
  rolled.accel[1] = 1.0f;   // roll = +90 deg
  rolled.accel[2] = 0.0f;
  rolled.gyro[1] = 10.0f;   // body pitch-rate becomes world yaw-rate
  rolled.gyro[2] = 0.0f;

  YawTracker yaw;
  yaw.Reset();
  yaw.Update(rolled, 0);
  yaw.Update(rolled, 1000);

  TEST_ASSERT_FLOAT_WITHIN(0.2f, 10.0f, yaw.yaw_rate_deg());
  TEST_ASSERT_FLOAT_WITHIN(0.2f, 10.0f, yaw.yaw_deg());
}

void test_ignores_stale_samples(void) {
  YawTracker yaw;
  yaw.Reset();
  yaw.Update(LevelRaw(10.0f), 0);
  yaw.Update(LevelRaw(10.0f), 1000);
  const float before = yaw.yaw_deg();

  RawImu stale = LevelRaw(100.0f);
  stale.present = false;
  yaw.Update(stale, 2000);

  TEST_ASSERT_FLOAT_WITHIN(0.01f, before, yaw.yaw_deg());
}

void test_clamps_large_time_gap(void) {
  YawTracker yaw;
  yaw.Reset();
  yaw.Update(LevelRaw(100.0f), 1000);
  yaw.Update(LevelRaw(100.0f), 6000);  // 5 s gap must be clamped to 1 s

  TEST_ASSERT_TRUE(yaw.yaw_deg() <= 100.01f);
  TEST_ASSERT_TRUE(yaw.yaw_deg() >= 99.99f);
}

void test_stationary_does_not_accumulate_yaw(void) {
  YawTracker yaw;
  yaw.Reset();
  const RawImu noisy = LevelRaw(0.5f);  // 静止とみなせる微小な角速度
  yaw.Update(noisy, 0);
  yaw.Update(noisy, 1000);
  yaw.Update(noisy, 2000);

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, yaw.yaw_deg());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, yaw.yaw_rate_deg());
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_integrates_constant_yaw_rate);
  RUN_TEST(test_wraps_at_360_degrees);
  RUN_TEST(test_subtracts_gyro_bias);
  RUN_TEST(test_calibrates_bias_from_samples);
  RUN_TEST(test_tilt_compensates_yaw_rate);
  RUN_TEST(test_ignores_stale_samples);
  RUN_TEST(test_clamps_large_time_gap);
  RUN_TEST(test_stationary_does_not_accumulate_yaw);
  return UNITY_END();
}

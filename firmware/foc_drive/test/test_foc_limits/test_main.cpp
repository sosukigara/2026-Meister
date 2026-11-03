#include <unity.h>

#include <cmath>

#include "foc_limits.h"

namespace {

void test_current_limit_validation() {
  TEST_ASSERT_FALSE(foc::validCurrentLimit(0.0f));
  TEST_ASSERT_FALSE(foc::validCurrentLimit(-1.0f));
  TEST_ASSERT_FALSE(foc::validCurrentLimit(foc::kCurrentLimitMin - 0.01f));
  TEST_ASSERT_TRUE(foc::validCurrentLimit(foc::kCurrentLimitMin));
  TEST_ASSERT_TRUE(foc::validCurrentLimit(0.1f));
  TEST_ASSERT_TRUE(foc::validCurrentLimit(foc::kCurrentLimitMax));
  TEST_ASSERT_FALSE(foc::validCurrentLimit(foc::kCurrentLimitMax + 0.01f));
  TEST_ASSERT_FALSE(foc::validCurrentLimit(NAN));
  TEST_ASSERT_FALSE(foc::validCurrentLimit(INFINITY));
}

void test_align_voltage_validation() {
  TEST_ASSERT_FALSE(foc::validAlignVoltage(0.0f));
  TEST_ASSERT_FALSE(foc::validAlignVoltage(foc::kAlignVoltageMin - 0.01f));
  TEST_ASSERT_TRUE(foc::validAlignVoltage(foc::kAlignVoltageMin));
  TEST_ASSERT_TRUE(foc::validAlignVoltage(0.30f));
  TEST_ASSERT_TRUE(foc::validAlignVoltage(foc::kAlignVoltageMax));
  TEST_ASSERT_FALSE(foc::validAlignVoltage(foc::kAlignVoltageMax + 0.01f));
  TEST_ASSERT_FALSE(foc::validAlignVoltage(NAN));
}

void test_open_loop_voltage_clamp() {
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, foc::kOpenLoopVoltageMin,
                           foc::clampOpenLoopVoltage(0.02f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.30f, foc::clampOpenLoopVoltage(0.30f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, foc::kOpenLoopVoltageMax,
                           foc::clampOpenLoopVoltage(2.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, foc::kOpenLoopVoltageMin,
                           foc::clampOpenLoopVoltage(NAN));
}

void test_speed_clamp() {
  constexpr float limit = foc::kOpenLoopMaxRps;
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, limit, foc::clampSpeedRps(1e9f, limit));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -limit, foc::clampSpeedRps(-1e9f, limit));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, foc::clampSpeedRps(2.0f, limit));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, foc::clampSpeedRps(NAN, limit));
}

void test_angle_clamp() {
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, foc::kMaxAngleRad,
                           foc::clampAngleRad(1e9f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -foc::kMaxAngleRad,
                           foc::clampAngleRad(-1e9f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.5f, foc::clampAngleRad(1.5f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, foc::clampAngleRad(NAN));
}

void test_pole_pair_validation() {
  TEST_ASSERT_FALSE(foc::validPolePairs(0));
  TEST_ASSERT_TRUE(foc::validPolePairs(1));
  TEST_ASSERT_TRUE(foc::validPolePairs(21));
  TEST_ASSERT_TRUE(foc::validPolePairs(foc::kMaxPolePairs));
  TEST_ASSERT_FALSE(foc::validPolePairs(foc::kMaxPolePairs + 1));
}

void test_cooldown_wraps_at_49_days() {
  // until is in the future across the millis() wrap.
  TEST_ASSERT_TRUE(foc::cooldownActive(0xFFFFFF00u, 0x100u));
  TEST_ASSERT_FALSE(foc::cooldownActive(0x200u, 0x100u));
  TEST_ASSERT_FALSE(foc::cooldownActive(0x100u, 0x100u));
}

void test_timeout_wraps_at_49_days() {
  TEST_ASSERT_TRUE(foc::timeoutElapsed(0x10u, 0xFFFFFF00u, 100u));
  TEST_ASSERT_FALSE(foc::timeoutElapsed(0xFFFFFF50u, 0xFFFFFF00u, 100u));
  TEST_ASSERT_FALSE(foc::timeoutElapsed(0x100u, 0x100u, 100u));
}

}  // namespace

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_current_limit_validation);
  RUN_TEST(test_align_voltage_validation);
  RUN_TEST(test_open_loop_voltage_clamp);
  RUN_TEST(test_speed_clamp);
  RUN_TEST(test_angle_clamp);
  RUN_TEST(test_pole_pair_validation);
  RUN_TEST(test_cooldown_wraps_at_49_days);
  RUN_TEST(test_timeout_wraps_at_49_days);
  return UNITY_END();
}

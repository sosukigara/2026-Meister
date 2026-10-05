#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "meister_serial_bridge/kinematics.hpp"

namespace msb = meister_serial_bridge;
namespace {

msb::TwistKinematics Default() {
  msb::TwistKinematics k;
  k.wheelbase_m = 0.3;
  k.max_linear_vel_mps = 1.0;
  k.max_steering_deg = 30.0;
  return k;
}

}  // namespace

TEST(TwistToActuators, StraightForward) {
  const auto cmd = msb::TwistToActuators(0.5, 0.0, Default());
  EXPECT_EQ(cmd.steering_tenths, 0);
  EXPECT_EQ(cmd.velocity_permille, 500);
}

TEST(TwistToActuators, LeftTurnIsPositiveSteering) {
  const auto k = Default();
  const auto cmd = msb::TwistToActuators(0.5, 0.5, k);
  const int expected =
      static_cast<int>(std::nearbyint(std::atan2(0.3 * 0.5, 0.5) * 180.0 / M_PI * 10.0));
  EXPECT_EQ(cmd.steering_tenths, expected);
  EXPECT_GT(cmd.steering_tenths, 0);
  EXPECT_EQ(cmd.velocity_permille, 500);
}

TEST(TwistToActuators, SteeringClampsToMax) {
  const auto cmd = msb::TwistToActuators(0.1, 5.0, Default());
  EXPECT_EQ(cmd.steering_tenths, 300);
}

TEST(TwistToActuators, NearZeroVxGivesFullDeflectionNoDrive) {
  // アッカーマンはその場旋回できないので、停止中の旋回指示は最大舵角と前進 0。
  const auto k = Default();
  auto cmd = msb::TwistToActuators(0.0, 1.0, k);
  EXPECT_EQ(cmd.steering_tenths, 300);
  EXPECT_EQ(cmd.velocity_permille, 0);

  cmd = msb::TwistToActuators(0.0, -1.0, k);
  EXPECT_EQ(cmd.steering_tenths, -300);
  EXPECT_EQ(cmd.velocity_permille, 0);
}

TEST(TwistToActuators, StopIsAllZero) {
  const auto cmd = msb::TwistToActuators(0.0, 0.0, Default());
  EXPECT_EQ(cmd.steering_tenths, 0);
  EXPECT_EQ(cmd.velocity_permille, 0);
}

TEST(TwistToActuators, VelocityClamps) {
  auto k = Default();
  EXPECT_EQ(msb::TwistToActuators(2.0, 0.0, k).velocity_permille, 1000);
  EXPECT_EQ(msb::TwistToActuators(-2.0, 0.0, k).velocity_permille, -1000);
}

TEST(TwistToActuators, InvertFlags) {
  auto k = Default();
  k.drive_invert = true;
  EXPECT_EQ(msb::TwistToActuators(0.5, 0.0, k).velocity_permille, -500);

  k = Default();
  k.steer_invert = true;
  const int expected =
      static_cast<int>(std::nearbyint(std::atan2(0.3, 1.0) * 180.0 / M_PI * 10.0));
  EXPECT_EQ(msb::TwistToActuators(0.5, 0.5, k).steering_tenths, -expected);
}

TEST(TwistToActuators, RejectsNonPositiveLimits) {
  auto k = Default();
  k.max_linear_vel_mps = 0.0;
  EXPECT_THROW(msb::TwistToActuators(0.1, 0.0, k), std::invalid_argument);

  k = Default();
  k.max_steering_deg = -1.0;
  EXPECT_THROW(msb::TwistToActuators(0.1, 0.0, k), std::invalid_argument);
}

// Python 版は round()（偶数側）を使っていた。0.5 の境界で C++ の round と
// 結果が変わると UART に 1 クロック違う指令が流れるので、期待値を固定する。
TEST(TwistToActuators, RoundsHalfToEvenLikePythonRound) {
  auto k = Default();
  k.max_linear_vel_mps = 4.0;  // vx=0.002 -> 0.5 千分率
  EXPECT_EQ(msb::TwistToActuators(0.002, 0.0, k).velocity_permille, 0);

  k.max_linear_vel_mps = 3.0;  // vx=0.005 -> 1.666... -> 2
  EXPECT_EQ(msb::TwistToActuators(0.005, 0.0, k).velocity_permille, 2);
}

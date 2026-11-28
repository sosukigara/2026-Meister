#include <cmath>

#include <gtest/gtest.h>

#include "umeonigiri_serial_bridge/ackermann_imu_odometry.hpp"

namespace umeonigiri_serial_bridge {
namespace {

TEST(AckermannImuOdometryTest, RejectsMagneticYawJumpInDifferentialMode) {
  OdometryConfig cfg{};
  cfg.imu_use_differential_yaw = true;
  cfg.imu_max_jump_rad_per_s = 3.5;
  cfg.imu_wz_fusion_gain = 0.7;

  AckermannImuOdometry odom(cfg);
  odom.Step(1.00);
  odom.UpdateTwistCommand(0.20, 0.0, 1.00);
  odom.UpdateImuReport(ImuReport{0, 0x33}, 1.00);

  // 20 ms で 30 度（~26 rad/s）の磁気ジャンプを注入しても yaw は暴走しない
  odom.UpdateImuReport(ImuReport{300, 0x33}, 1.02);
  const auto& st = odom.Step(1.02);

  EXPECT_EQ(st.imu_rejected_jumps, 1U);
  EXPECT_NEAR(st.yaw, 0.0, 1e-3);
  EXPECT_NEAR(st.x, 0.004, 1e-4);
}

TEST(AckermannImuOdometryTest, PrefersEncoderFeedbackAndFallsBackOnTimeout) {
  OdometryConfig cfg{};
  cfg.use_encoder_feedback = true;
  cfg.encoder_units_to_mps = 0.001;
  cfg.encoder_timeout_s = 0.25;

  AckermannImuOdometry odom(cfg);
  odom.Step(1.0);
  odom.UpdateTwistCommand(0.25, 0.0, 1.0);

  Feedback fb{};
  fb.encoders.fill(180);  // 実測 0.18 m/s
  odom.UpdateWheelFeedback(fb, 1.0);

  const auto& st1 = odom.Step(1.1);
  EXPECT_TRUE(st1.using_encoder_feedback);
  EXPECT_NEAR(st1.vx, 0.18, 1e-6);
  EXPECT_NEAR(st1.x, 0.018, 1e-5);

  // エンコーダがタイムアウトしたら指令値にフォールバック
  odom.UpdateTwistCommand(0.25, 0.0, 1.35);
  const auto& st2 = odom.Step(1.4);
  EXPECT_FALSE(st2.using_encoder_feedback);
  EXPECT_NEAR(st2.vx, 0.25, 1e-6);
}

TEST(AckermannImuOdometryTest, DifferentialImuFusesSmoothTurnRate) {
  OdometryConfig cfg{};
  cfg.wheelbase_m = 0.60;
  cfg.imu_use_differential_yaw = true;
  cfg.imu_wz_fusion_gain = 0.7;
  cfg.imu_max_innovation_rad_per_s = 1.2;

  AckermannImuOdometry odom(cfg);
  odom.Step(1.00);
  odom.UpdateTwistCommand(0.25, 0.20, 1.00);
  odom.UpdateImuReport(ImuReport{0, 0x33}, 1.00);

  // 0.10 s で 1.15 deg (~0.20 rad/s) 回転
  odom.UpdateImuReport(ImuReport{12, 0x33}, 1.10);
  const auto& st = odom.Step(1.10);

  EXPECT_EQ(st.imu_rejected_jumps, 0U);
  EXPECT_GT(st.yaw, 0.01);
  EXPECT_LT(st.yaw, 0.03);
}

}  // namespace
}  // namespace umeonigiri_serial_bridge

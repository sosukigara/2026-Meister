/* kinematics.cpp の host テスト。

拘束ソルバ本体は幾何が未確定なので未実装。ここでは今確定している
2 つの挙動だけを固定する:
  - vy != 0 は前方適用せず kLateralUnsupported を返す
  - 幾何が未確定なら kGeometryNotFilled を返し、解なしと区別する

幾何が確定したら拘束のテストを追加する。
*/

#include <unity.h>

#include "command.h"
#include "kinematics.h"

using meister::ArmCommand;
using meister::RobotCommand;
using meister::TwistCommand;
using meister::kin::DriveSetpoint;
using meister::kin::RobotGeometry;
using meister::kin::RockerPosition;
using meister::kin::Solve;
using meister::kin::SolveInverse;
using meister::kin::SolveStatus;

namespace {

constexpr size_t kDummyTableSize = 3;

RockerPosition DummyTable() {
  RockerPosition table[kDummyTableSize];
  for (size_t i = 0; i < kDummyTableSize; ++i) {
    table[i].name = "dummy";
  }
  return table[0];
}

}  // namespace

void setUp() {}
void tearDown() {}

// vy != 0 は前方適用せず拒否する
void test_lateral_is_rejected() {
  RobotGeometry geo;
  RockerPosition table[kDummyTableSize];
  DriveSetpoint out;
  TwistCommand twist;
  twist.vx = 1000;
  twist.vy = 1;  // 1 でも拒否する。黙って前方だけ動かすと嘘になる

  const SolveStatus st = Solve(twist, geo, table, kDummyTableSize, &out);
  TEST_ASSERT_EQUAL(SolveStatus::kLateralUnsupported, st);
  TEST_ASSERT_EQUAL_INT8(-1, out.rocker_row);
  TEST_ASSERT_FALSE(meister::kin::TwistIsSupported(twist));
}

// vy == 0 でも幾何が未確定なら kGeometryNotFilled（「解なし」と区別する）
void test_geometry_not_filled_is_distinct_from_no_solution() {
  RobotGeometry geo;
  RockerPosition table[kDummyTableSize];
  DriveSetpoint out;
  TwistCommand twist;
  twist.vx = 1000;

  const SolveStatus st = Solve(twist, geo, table, kDummyTableSize, &out);
  TEST_ASSERT_EQUAL(SolveStatus::kGeometryNotFilled, st);
  TEST_ASSERT_TRUE(st != SolveStatus::kNoSolution);
}

// 幾何が未確定なら前進指令でも out は「解なし」の状態で残る
void test_setpoint_is_initialized_before_returning() {
  RobotGeometry geo;
  RockerPosition table[kDummyTableSize];
  DriveSetpoint out;
  out.rocker_row = 42;  // 呼び出し前のダミー値
  TwistCommand twist;

  Solve(twist, geo, table, kDummyTableSize, &out);
  TEST_ASSERT_EQUAL_INT8(-1, out.rocker_row);
  for (uint8_t i = 0; i < meister::config::kNumDriveMotors; ++i) {
    TEST_ASSERT_EQUAL_INT16(0, out.wheel_velocity[i]);
    TEST_ASSERT_EQUAL_INT16(0, out.wheel_steering[i]);
  }
}

// 逆変換も同じ状態で止まる
void test_inverse_also_stops_on_unfilled_geometry() {
  RobotGeometry geo;
  TwistCommand out;
  out.vx = 999;
  int16_t vel[meister::config::kNumDriveMotors] = {};
  int16_t steer[meister::config::kNumDriveMotors] = {};

  const SolveStatus st =
      SolveInverse(vel, steer, meister::config::kNumDriveMotors, geo, &out);
  TEST_ASSERT_EQUAL(SolveStatus::kGeometryNotFilled, st);
  TEST_ASSERT_EQUAL_INT16(0, out.vx);
}

// クランプは範囲外の値を端に丸める。0 にしない。
void test_clamp_rounds_to_bound_not_zero() {
  RobotCommand cmd;
  cmd.wheels[0].velocity = 2000;  // kMaxVelocity = 1000 を超過
  cmd.wheels[1].velocity = -2000;
  cmd.wheels[2].velocity = 1000;  // 境界値
  const RobotCommand d = meister::ClampCommand(cmd);

  TEST_ASSERT_EQUAL_INT16(1000, d.wheels[0].velocity);
  TEST_ASSERT_EQUAL_INT16(-1000, d.wheels[1].velocity);
  TEST_ASSERT_EQUAL_INT16(1000, d.wheels[2].velocity);
}

void test_clamp_arm_four_axes() {
  ArmCommand arm;
  // int16_t の範囲内、かつ設定範囲（0..1800）外。
  // 32767 などの範囲外は int16_t に渡る時点で回り込むので検証にならない。
  arm.shoulder_pitch = 3000;
  arm.elbow = 2000;
  arm.wrist = -5;
  arm.gripper = 5000;
  const ArmCommand d = meister::ClampArm(arm);

  TEST_ASSERT_EQUAL_INT16(meister::config::kMaxShoulder, d.shoulder_pitch);
  TEST_ASSERT_EQUAL_INT16(meister::config::kMaxJoint, d.elbow);
  TEST_ASSERT_EQUAL_INT16(meister::config::kMinJoint, d.wrist);
  TEST_ASSERT_EQUAL_INT16(meister::config::kMaxGripper, d.gripper);
  TEST_ASSERT_EQUAL_UINT8(4, meister::kArmAxisCount);
}

void test_twist_clamp() {
  TwistCommand t;
  t.vx = 20000;
  t.vy = -20000;
  t.wz = 150;
  const TwistCommand d = meister::ClampTwist(t);
  TEST_ASSERT_EQUAL_INT16(20000, d.vx);
  TEST_ASSERT_EQUAL_INT16(-20000, d.vy);
  TEST_ASSERT_EQUAL_INT16(150, d.wz);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_lateral_is_rejected);
  RUN_TEST(test_geometry_not_filled_is_distinct_from_no_solution);
  RUN_TEST(test_setpoint_is_initialized_before_returning);
  RUN_TEST(test_inverse_also_stops_on_unfilled_geometry);
  RUN_TEST(test_clamp_rounds_to_bound_not_zero);
  RUN_TEST(test_clamp_arm_four_axes);
  RUN_TEST(test_twist_clamp);
  return UNITY_END();
}

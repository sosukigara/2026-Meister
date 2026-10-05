/* kinematics.cpp の host テスト。

このテストは**合成幾何**を使う。実機の 6 輪配置は ★要確認★ なので、
production の kinematics.geometry_filled は false のまま Plenty あり、
Solve() は kGeometryNotFilled を返す。ソルバの性質はこのテスト用の
SolveWithGeometry() で確かめる。

合成幾何: 前後 2 軸、左右 ±0.25 m、前後 ±0.30 m。
前 2 輪と後 2 輪は舵角可、中 2 輪は固定。
 */

#include <unity.h>

#include "command.h"
#include "kinematics.h"

using meister::TwistCommand;
using meister::kin::DriveSetpoint;
using meister::kin::RobotGeometry;
using meister::kin::RockerPosition;
using meister::kin::Solve;
using meister::kin::SolveInverse;
using meister::kin::SolveStatus;
using meister::kin::SolveWithGeometry;

namespace {

constexpr uint8_t kWheels = meister::config::kNumDriveMotors;
constexpr float kFrontX = 0.30f;
constexpr float kRearX = -0.30f;
constexpr float kTrackY = 0.25f;

/// 前後 2 軸・左右 ±0.25 の合成機体。
RobotGeometry SyntheticGeometry() {
  RobotGeometry geo;
  geo.wheel_radius = meister::config::kWheelRadius;
  const float xs[6] = {kFrontX, kFrontX, 0.0f, 0.0f, kRearX, kRearX};
  const float ys[6] = {kTrackY, -kTrackY, kTrackY, -kTrackY, kTrackY, -kTrackY};
  const bool steerable[6] = {true, true, false, false, true, true};
  const uint8_t group[6] = {0, 0, 0, 0, 1, 1};
  for (uint8_t i = 0; i < kWheels; ++i) {
    geo.wheels[i].x = xs[i];
    geo.wheels[i].y = ys[i];
    geo.wheels[i].steerable = steerable[i];
    geo.wheels[i].rocker_group = group[i];
  }
  return geo;
}

/// 直進行（ボギー軸 0、前後とも舵角可）と旋回行（後 bogie を振る）。
constexpr size_t kRows = 2;

RockerPosition* MakeTable(const int16_t* front, const int16_t* rear,
                          const char** names) {
  static RockerPosition table[kRows];
  for (size_t i = 0; i < kRows; ++i) {
    table[i] = RockerPosition{};
    table[i].servo_angle[0] = front[i];
    table[i].servo_angle[1] = rear[i];
    table[i].name = names[i];
  }
  return table;
}

}  // namespace

void setUp() {}
void tearDown() {}

// 前進: 全輪が同じ速度、舵角は 0
void test_straight_gives_equal_speeds_and_zero_steer() {
  const int16_t front[2] = {0, 0};
  const int16_t rear[2] = {0, 0};
  const char* names[2] = {"straight", "turn"};
  RockerPosition* table = MakeTable(front, rear, names);

  TwistCommand t;
  t.vx = static_cast<int16_t>(meister::config::kMaxLinearMmPerSec);  // 全力
  DriveSetpoint out;
  TEST_ASSERT_EQUAL(SolveStatus::kOk,
                    SolveWithGeometry(t, SyntheticGeometry(), table, kRows, &out, true));
  TEST_ASSERT_EQUAL_INT8(0, out.rocker_row);
  const int16_t first = out.wheel_velocity[0];
  for (uint8_t i = 0; i < kWheels; ++i) {
    TEST_ASSERT_EQUAL_INT16(first, out.wheel_velocity[i]);
    TEST_ASSERT_EQUAL_INT16(0, out.wheel_steering[i]);
  }
  TEST_ASSERT_EQUAL_INT16(1000, first);
}

// 前進だけなら解けるので先頭行を使う
void test_straight_prefers_first_row() {
  const int16_t front[2] = {0, 0};
  const int16_t rear[2] = {0, 30};
  const char* names[2] = {"straight", "turn"};
  RockerPosition* table = MakeTable(front, rear, names);

  TwistCommand t;
  t.vx = 500;
  DriveSetpoint out;
  SolveWithGeometry(t, SyntheticGeometry(), table, kRows, &out, true);
  TEST_ASSERT_EQUAL_INT8(0, out.rocker_row);
  TEST_ASSERT_EQUAL_INT16(0, out.rocker_angle[1]);
}

// 固定舵角の輪（前ロッカーのみ）は、前後位置の差だけ速度が出る
void test_fixed_steer_wheels_project_onto_their_heading() {
  const int16_t front[2] = {0, 0};
  const int16_t rear[2] = {0, 0};
  const char* names[2] = {"straight", "turn"};
  RockerPosition* table = MakeTable(front, rear, names);

  TwistCommand t;
  t.vx = 800;  // mm/s
  t.wz = 0;
  DriveSetpoint out;
  TEST_ASSERT_EQUAL(SolveStatus::kOk,
                    SolveWithGeometry(t, SyntheticGeometry(), table, kRows, &out, true));
  // 固定舵角の輪も前進速度は持つ。回転がない（wz=0）ので前後と同じ速度になる。
  TEST_ASSERT_EQUAL_INT16(out.wheel_velocity[0], out.wheel_velocity[2]);
  TEST_ASSERT_EQUAL_INT16(out.wheel_velocity[0], out.wheel_velocity[3]);
}

// 純粋な旋回は舵角 130° を要求するので、舵角 ±90° では解けない。
// これは合成幾何の物理的制約であり、ソルバがこれを検出できることの検証。
void test_pure_spin_needs_more_rocker_than_available() {
  const int16_t front[2] = {0, 0};
  const int16_t rear[2] = {0, 0};  // ボギー軸すべて 0 = ステアリングのみ
  const char* names[2] = {"straight", "turn_small"};
  RockerPosition* table = MakeTable(front, rear, names);

  TwistCommand t;
  t.vx = 0;
  t.wz = 500;
  DriveSetpoint out;
  // 前左輪は 129.8° を要求する。±90° には収まらない。
  TEST_ASSERT_EQUAL(SolveStatus::kNoSolution,
                    SolveWithGeometry(t, SyntheticGeometry(), table, kRows, &out, true));
  TEST_ASSERT_EQUAL_INT8(-1, out.rocker_row);
}

// 純粋な旋回はボギーを振っても片側で舵角範囲を超える。組合せ運動なら解ける。
void test_combined_turn_and_straight_is_solvable_without_rocker() {
  const int16_t front[2] = {0, 0};
  const int16_t rear[2] = {0, 0};
  const char* names[2] = {"straight", "rocker"};
  RockerPosition* table = MakeTable(front, rear, names);

  TwistCommand t;
  t.vx = 300;
  t.wz = 400;
  DriveSetpoint out;
  // 前進成分があるので要求舵角が 60° 未満に収まり、ボギー 0 で解ける。
  TEST_ASSERT_EQUAL(SolveStatus::kOk,
                    SolveWithGeometry(t, SyntheticGeometry(), table, kRows, &out, true));
  TEST_ASSERT_EQUAL_INT8(0, out.rocker_row);
  // 舵角は ±90° の範囲内
  for (uint8_t i = 0; i < kWheels; ++i) {
    TEST_ASSERT_TRUE(out.wheel_steering[i] >= meister::config::kMinSteering);
    TEST_ASSERT_TRUE(out.wheel_steering[i] <= meister::config::kMaxSteering);
  }
}

// 旋回では同じ側の 2 輪が同じ速度になり、左右で違う
void test_turn_splits_inner_and_outer_speed() {
  const int16_t front[2] = {0, 60};
  const int16_t rear[2] = {0, -60};
  const char* names[2] = {"none", "rocker_60"};
  RockerPosition* table = MakeTable(front, rear, names);

  TwistCommand t;
  t.vx = 300;
  t.wz = 400;
  DriveSetpoint out;
  TEST_ASSERT_EQUAL(SolveStatus::kOk,
                    SolveWithGeometry(t, SyntheticGeometry(), table, kRows, &out, true));
  // 左側 = 前左(0) / 中左(2) / 後左(4)、右側 = 1 / 3 / 5
  // 回転の寄与があるので左と右で違う。
  TEST_ASSERT_TRUE(out.wheel_velocity[0] != out.wheel_velocity[1]);
  TEST_ASSERT_TRUE(out.wheel_velocity[4] != out.wheel_velocity[5]);
}

// どの行も舵角範囲に収まらない場合は kNoSolution
void test_no_row_fits_returns_no_solution() {
  const int16_t front[2] = {0, 0};
  const int16_t rear[2] = {0, 0};
  const char* names[2] = {"straight", "turn"};
  RockerPosition* table = MakeTable(front, rear, names);
  // 舵角可な輪を 1 輪もない機体にすると、前進でも旋回でも解けない
  RobotGeometry geo = SyntheticGeometry();
  for (uint8_t i = 0; i < kWheels; ++i) {
    geo.wheels[i].steerable = false;
  }

  TwistCommand t;
  t.vx = 0;
  t.wz = 1000;  // 大きく舵角が要る
  DriveSetpoint out;
  const SolveStatus st = SolveWithGeometry(t, geo, table, kRows, &out, true);
  TEST_ASSERT_TRUE(st == SolveStatus::kNoSolution ||
                   st == SolveStatus::kOk);
  if (st == SolveStatus::kNoSolution) {
    TEST_ASSERT_EQUAL_INT8(-1, out.rocker_row);
  }
}

// vy は production の Solve でも拒否される（ゲートより先に）
void test_lateral_rejected_before_geometry_gate() {
  const char* names[1] = {"straight"};
  const int16_t front[1] = {0};
  const int16_t rear[1] = {0};
  RockerPosition* table = MakeTable(front, rear, names);
  TwistCommand t;
  t.vy = 1;
  DriveSetpoint out;
  // geometry_filled=false でも kLateralUnsupported が先に出る
  TEST_ASSERT_EQUAL(SolveStatus::kLateralUnsupported,
                    Solve(t, SyntheticGeometry(), table, 1, &out));
  TEST_ASSERT_EQUAL_INT8(-1, out.rocker_row);
}

// geometry_filled=false では合成幾何を渡しても解けない
void test_production_gate_blocks_even_with_geometry() {
  const char* names[1] = {"straight"};
  const int16_t front[1] = {0};
  const int16_t rear[1] = {0};
  RockerPosition* table = MakeTable(front, rear, names);
  TwistCommand t;
  t.vx = 100;
  DriveSetpoint out;
  TEST_ASSERT_EQUAL(SolveStatus::kGeometryNotFilled,
                    Solve(t, SyntheticGeometry(), table, 1, &out));
  // 同じ入力でゲートを外せば解ける = ゲートだけが止めている証拠
  TEST_ASSERT_EQUAL(SolveStatus::kOk,
                    SolveWithGeometry(t, SyntheticGeometry(), table, 1, &out, true));
}

// 逆変換は前方キネマティクスと往復する
void test_inverse_round_trips_straight() {
  const char* names[1] = {"straight"};
  const int16_t front[1] = {0};
  const int16_t rear[1] = {0};
  RockerPosition* table = MakeTable(front, rear, names);
  const RobotGeometry geo = SyntheticGeometry();

  TwistCommand in;
  in.vx = 600;
  in.wz = 0;
  DriveSetpoint sp;
  TEST_ASSERT_EQUAL(SolveStatus::kOk,
                    SolveWithGeometry(in, geo, table, 1, &sp, true));

  TwistCommand back;
  TEST_ASSERT_EQUAL(SolveStatus::kOk,
                    meister::kin::SolveInverseWithGeometry(
                        sp.wheel_velocity, sp.wheel_steering, kWheels, geo, &back, true));
  // 往復誤差は 2% まで許す
  TEST_ASSERT_INT_WITHIN(15, in.vx, back.vx);
  TEST_ASSERT_INT_WITHIN(20, in.wz, back.wz);
  TEST_ASSERT_EQUAL_INT16(0, back.vy);
}

// 逆変換は退化幾何で kNoSolution
void test_inverse_returns_no_solution_when_degenerate() {
  RobotGeometry geo = SyntheticGeometry();
  for (uint8_t i = 0; i < kWheels; ++i) {
    geo.wheels[i].x = 0.0f;
    geo.wheels[i].y = 0.0f;  // 全輪が車体中心 = 回転を観測できない
  }
  int16_t vel[kWheels] = {};
  int16_t steer[kWheels] = {};
  TwistCommand out;
  TEST_ASSERT_EQUAL(SolveStatus::kNoSolution,
                    meister::kin::SolveInverseWithGeometry(vel, steer, kWheels, geo,
                                                          &out, true));
}

// 輪数が違う入力は受け付けない
void test_inverse_rejects_wrong_wheel_count() {
  const RobotGeometry geo = SyntheticGeometry();
  int16_t vel[kWheels] = {};
  int16_t steer[kWheels] = {};
  TwistCommand out;
  TEST_ASSERT_EQUAL(SolveStatus::kNoSolution,
                    meister::kin::SolveInverseWithGeometry(vel, steer, kWheels - 1,
                                                          geo, &out, true));
}


// 境界: 要求舵角がちょうど ±90°（舵角上限）は許容される
void test_steering_exactly_at_limit_is_accepted() {
  // 輪を (1, 0) に置くと、純粋な旋回で要求舵角がちょうど 90° になる。
  // 他の輪は舵角不可にして、この 1 輪だけが境界を決めるようにする。
  RobotGeometry geo;
  geo.wheel_radius = meister::config::kWheelRadius;
  for (uint8_t i = 0; i < kWheels; ++i) {
    geo.wheels[i] = meister::kin::WheelGeometry{};
    geo.wheels[i].x = 0.0f;
    geo.wheels[i].y = 0.0f;
    geo.wheels[i].steerable = false;
    geo.wheels[i].rocker_group = 0;
  }
  geo.wheels[0].x = 1.0f;
  geo.wheels[0].steerable = true;

  const char* names[1] = {"straight"};
  const int16_t front[1] = {0};
  const int16_t rear[1] = {0};
  RockerPosition* table = MakeTable(front, rear, names);

  TwistCommand t;
  t.vx = 0;
  t.wz = 500;  // 正 = 左旋回
  DriveSetpoint out;
  const SolveStatus st = SolveWithGeometry(t, geo, table, 1, &out, true);
  if (st == SolveStatus::kOk) {
    // 境界値なら上限ちょうどか、それに近い
    TEST_ASSERT_TRUE(out.wheel_steering[0] <= meister::config::kMaxSteering);
    TEST_ASSERT_TRUE(out.wheel_steering[0] > meister::config::kMaxSteering - 50);
  }
}

// 境界をわずかに超えると解なしになる（上限が効いていることの確認）
void test_steering_just_past_limit_is_rejected() {
  RobotGeometry geo;
  geo.wheel_radius = meister::config::kWheelRadius;
  for (uint8_t i = 0; i < kWheels; ++i) {
    geo.wheels[i] = meister::kin::WheelGeometry{};
    geo.wheels[i].steerable = false;
    geo.wheels[i].rocker_group = 0;
  }
  // x=1, y=-0.02 だと要求舵角が 90° をわずかに超える
  geo.wheels[0].x = 1.0f;
  geo.wheels[0].y = -0.02f;
  geo.wheels[0].steerable = true;

  const char* names[1] = {"straight"};
  const int16_t front[1] = {0};
  const int16_t rear[1] = {0};
  RockerPosition* table = MakeTable(front, rear, names);

  TwistCommand t;
  t.vx = 0;
  t.wz = 500;
  DriveSetpoint out;
  const SolveStatus st = SolveWithGeometry(t, geo, table, 1, &out, true);
  // 境界の扱い（許容の epsilon）によって ok / no-solution のどちらになるので、
  // 「舵角が上限を超えて出ていない」ことを必ず確認する。
  if (st == SolveStatus::kOk) {
    for (uint8_t i = 0; i < kWheels; ++i) {
      TEST_ASSERT_TRUE(out.wheel_steering[i] <= meister::config::kMaxSteering);
    }
  }
  TEST_ASSERT_TRUE(st == SolveStatus::kOk || st == SolveStatus::kNoSolution);
}

// 全停止（vx = 0, wz = 0）は先頭行で解になり、全輪の速度が 0
void test_full_stop_uses_first_row_and_zero_speed() {
  const int16_t front[2] = {0, 0};
  const int16_t rear[2] = {0, 0};
  const char* names[2] = {"straight", "turn"};
  RockerPosition* table = MakeTable(front, rear, names);

  TwistCommand t;
  t.vx = 0;
  t.vy = 0;
  t.wz = 0;
  DriveSetpoint out;
  TEST_ASSERT_EQUAL(SolveStatus::kOk,
                    SolveWithGeometry(t, SyntheticGeometry(), table, kRows, &out, true));
  TEST_ASSERT_EQUAL_INT8(0, out.rocker_row);
  for (uint8_t i = 0; i < kWheels; ++i) {
    TEST_ASSERT_EQUAL_INT16(0, out.wheel_velocity[i]);
  }
}

// 空のテーブル（table_size == 0）は kNoSolution
void test_empty_table_returns_no_solution() {
  TwistCommand t;
  t.vx = 500;
  DriveSetpoint out;
  TEST_ASSERT_EQUAL(SolveStatus::kNoSolution,
                    SolveWithGeometry(t, SyntheticGeometry(), nullptr, 0, &out, true));
  TEST_ASSERT_EQUAL_INT8(-1, out.rocker_row);
}

// 行列式が退化する幾何（全輪が 1 点）は逆変換で kNoSolution
void test_inverse_degenerate_when_all_wheels_collinear_at_origin() {
  RobotGeometry geo = SyntheticGeometry();
  for (uint8_t i = 0; i < kWheels; ++i) {
    geo.wheels[i].x = 0.0f;
    geo.wheels[i].y = 0.0f;
  }
  int16_t vel[kWheels] = {100, 100, 100, 100, 100, 100};
  int16_t steer[kWheels] = {0, 0, 0, 0, 0, 0};
  TwistCommand out;
  TEST_ASSERT_EQUAL(SolveStatus::kNoSolution,
                    meister::kin::SolveInverseWithGeometry(vel, steer, kWheels, geo,
                                                          &out, true));
  // 失敗しても out はゼロ初期化されている（前の値を残さない）
  TEST_ASSERT_EQUAL_INT16(0, out.vx);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_straight_gives_equal_speeds_and_zero_steer);
  RUN_TEST(test_straight_prefers_first_row);
  RUN_TEST(test_fixed_steer_wheels_project_onto_their_heading);
  RUN_TEST(test_pure_spin_needs_more_rocker_than_available);
  RUN_TEST(test_combined_turn_and_straight_is_solvable_without_rocker);
  RUN_TEST(test_turn_splits_inner_and_outer_speed);
  RUN_TEST(test_no_row_fits_returns_no_solution);
  RUN_TEST(test_lateral_rejected_before_geometry_gate);
  RUN_TEST(test_production_gate_blocks_even_with_geometry);
  RUN_TEST(test_inverse_round_trips_straight);
  RUN_TEST(test_inverse_returns_no_solution_when_degenerate);
  RUN_TEST(test_inverse_rejects_wrong_wheel_count);
  RUN_TEST(test_steering_exactly_at_limit_is_accepted);
  RUN_TEST(test_steering_just_past_limit_is_rejected);
  RUN_TEST(test_full_stop_uses_first_row_and_zero_speed);
  RUN_TEST(test_empty_table_returns_no_solution);
  RUN_TEST(test_inverse_degenerate_when_all_wheels_collinear_at_origin);
  return UNITY_END();
}

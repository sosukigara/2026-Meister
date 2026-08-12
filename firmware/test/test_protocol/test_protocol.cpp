/*
 * test_protocol.cpp — プロトコル層のホスト側ユニットテスト（Unity）
 *
 * 実行: `pio test -e native`（firmware/ ディレクトリ内で）
 *
 * カバー範囲:
 *   - 各コマンド種別のエンコード/デコードラウンドトリップ
 *   - チェックサムの検証（正常系 / 破損系）
 *   - 不正フレームの拒否（ヘッダ不一致・チェックサム不一致・既知/未知種別・途中切れ）
 */
#include <unity.h>

#include "meister_protocol.h"

using namespace meister;

// ---------------------------------------------------------------------------
// ラウンドトリップ（エンコード → パース → フィールド一致）
// ---------------------------------------------------------------------------

void test_roundtrip_motor_velocity(void) {
  const int16_t vel[proto::kNumDriveMotors] = {1000, 500, 0, -500, -1000, 321};
  uint8_t buf[proto::kMaxFrameSize];

  const size_t n = proto::EncodeMotorVelocity(buf, sizeof(buf), vel);

  TEST_ASSERT_EQUAL_UINT(proto::FrameSize(proto::kCmdMotorVelocity), n);
  TEST_ASSERT_EQUAL_UINT8(proto::kHeaderByte, buf[0]);
  TEST_ASSERT_EQUAL_UINT8(proto::kCmdMotorVelocity, buf[1]);

  proto::Frame f;
  TEST_ASSERT_EQUAL(proto::ParseResult::kOk, proto::ParseFrame(buf, n, &f));
  TEST_ASSERT_EQUAL(proto::kCmdMotorVelocity, f.type);
  for (size_t i = 0; i < proto::kNumDriveMotors; ++i) {
    TEST_ASSERT_EQUAL_INT16(vel[i], proto::FrameGetInt16(f, i * 2));
  }
}

void test_roundtrip_steering_angle(void) {
  const int16_t ang[proto::kNumSteeringServos] = {-900, -450, 0, 450, 900, -123};
  uint8_t buf[proto::kMaxFrameSize];

  const size_t n = proto::EncodeSteeringAngle(buf, sizeof(buf), ang);

  TEST_ASSERT_EQUAL_UINT(proto::FrameSize(proto::kCmdSteeringAngle), n);

  proto::Frame f;
  TEST_ASSERT_EQUAL(proto::ParseResult::kOk, proto::ParseFrame(buf, n, &f));
  TEST_ASSERT_EQUAL(proto::kCmdSteeringAngle, f.type);
  for (size_t i = 0; i < proto::kNumSteeringServos; ++i) {
    TEST_ASSERT_EQUAL_INT16(ang[i], proto::FrameGetInt16(f, i * 2));
  }
}

void test_roundtrip_arm_angle(void) {
  const int16_t ang[proto::kNumArmServos] = {0, 600, 900, 1800};
  uint8_t buf[proto::kMaxFrameSize];

  const size_t n = proto::EncodeArmAngle(buf, sizeof(buf), ang);

  TEST_ASSERT_EQUAL_UINT(proto::FrameSize(proto::kCmdArmAngle), n);

  proto::Frame f;
  TEST_ASSERT_EQUAL(proto::ParseResult::kOk, proto::ParseFrame(buf, n, &f));
  TEST_ASSERT_EQUAL(proto::kCmdArmAngle, f.type);
  for (size_t i = 0; i < proto::kNumArmServos; ++i) {
    TEST_ASSERT_EQUAL_INT16(ang[i], proto::FrameGetInt16(f, i * 2));
  }
}

void test_roundtrip_gripper(void) {
  uint8_t buf[proto::kMaxFrameSize];

  const proto::GripperCommand cmds[3] = {proto::kGripperClose, proto::kGripperOpen, proto::kGripperStop};
  for (size_t c = 0; c < 3; ++c) {
    const proto::GripperCommand cmd = cmds[c];
    const size_t n = proto::EncodeGripper(buf, sizeof(buf), cmd);
    TEST_ASSERT_EQUAL_UINT(proto::FrameSize(proto::kCmdGripper), n);

    proto::Frame f;
    TEST_ASSERT_EQUAL(proto::ParseResult::kOk, proto::ParseFrame(buf, n, &f));
    TEST_ASSERT_EQUAL(proto::kCmdGripper, f.type);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(cmd), proto::FrameGetU8(f, 0));
  }
}

void test_roundtrip_state_feedback(void) {
  const int16_t enc[proto::kNumDriveMotors] = {-1000, 777, 0, 42, -42, 1000};
  uint8_t buf[proto::kMaxFrameSize];

  const size_t n = proto::EncodeState(buf, sizeof(buf), enc, 3, proto::kFbErrorProtocol);

  TEST_ASSERT_EQUAL_UINT(proto::FrameSize(proto::kFbState), n);

  proto::Frame f;
  TEST_ASSERT_EQUAL(proto::ParseResult::kOk, proto::ParseFrame(buf, n, &f));
  TEST_ASSERT_EQUAL(proto::kFbState, f.type);
  for (size_t i = 0; i < proto::kNumDriveMotors; ++i) {
    TEST_ASSERT_EQUAL_INT16(enc[i], proto::FrameGetInt16(f, i * 2));
  }
  TEST_ASSERT_EQUAL_UINT8(3, proto::FrameGetU8(f, proto::kNumDriveMotors * 2));
  TEST_ASSERT_EQUAL_UINT8(proto::kFbErrorProtocol,
                          proto::FrameGetU8(f, proto::kNumDriveMotors * 2 + 1));
}

void test_roundtrip_error_feedback(void) {
  uint8_t buf[proto::kMaxFrameSize];

  const size_t n = proto::EncodeError(buf, sizeof(buf), 0xAB);
  TEST_ASSERT_EQUAL_UINT(proto::FrameSize(proto::kFbError), n);

  proto::Frame f;
  TEST_ASSERT_EQUAL(proto::ParseResult::kOk, proto::ParseFrame(buf, n, &f));
  TEST_ASSERT_EQUAL(proto::kFbError, f.type);
  TEST_ASSERT_EQUAL_UINT8(0xAB, proto::FrameGetU8(f, 0));
}

// ---------------------------------------------------------------------------
// チェックサム
// ---------------------------------------------------------------------------

void test_checksum_valid_frame(void) {
  const int16_t vel[proto::kNumDriveMotors] = {100, -200, 300, -400, 500, -600};
  uint8_t buf[proto::kMaxFrameSize];
  const size_t n = proto::EncodeMotorVelocity(buf, sizeof(buf), vel);

  // チェックサムを含む全バイトの XOR == 0
  TEST_ASSERT_TRUE(proto::CheckChecksum(buf, n));
  // チェックサムは最終バイトにある
  TEST_ASSERT_EQUAL_UINT8(proto::ComputeChecksum(buf, n - 1), buf[n - 1]);
}

void test_checksum_corrupted_payload(void) {
  const int16_t vel[proto::kNumDriveMotors] = {100, -200, 300, -400, 500, -600};
  uint8_t buf[proto::kMaxFrameSize];
  const size_t n = proto::EncodeMotorVelocity(buf, sizeof(buf), vel);

  buf[3] ^= 0x40;  // ペイロードを 1 bit 破損

  proto::Frame f;
  TEST_ASSERT_EQUAL(proto::ParseResult::kBadChecksum, proto::ParseFrame(buf, n, &f));
}

// ---------------------------------------------------------------------------
// 不正フレームの拒否
// ---------------------------------------------------------------------------

void test_reject_bad_header(void) {
  const int16_t vel[proto::kNumDriveMotors] = {1, 2, 3, 4, 5, 6};
  uint8_t buf[proto::kMaxFrameSize];
  const size_t n = proto::EncodeMotorVelocity(buf, sizeof(buf), vel);

  buf[0] = 0x00;  // ヘッダ破損

  proto::Frame f;
  TEST_ASSERT_EQUAL(proto::ParseResult::kBadHeader, proto::ParseFrame(buf, n, &f));
}

void test_reject_bad_checksum(void) {
  const int16_t vel[proto::kNumDriveMotors] = {1, 2, 3, 4, 5, 6};
  uint8_t buf[proto::kMaxFrameSize];
  const size_t n = proto::EncodeMotorVelocity(buf, sizeof(buf), vel);

  buf[n - 1] ^= 0xFF;  // チェックサム破損

  proto::Frame f;
  TEST_ASSERT_EQUAL(proto::ParseResult::kBadChecksum, proto::ParseFrame(buf, n, &f));
}

void test_reject_unknown_type(void) {
  const int16_t vel[proto::kNumDriveMotors] = {1, 2, 3, 4, 5, 6};
  uint8_t buf[proto::kMaxFrameSize];
  const size_t n = proto::EncodeMotorVelocity(buf, sizeof(buf), vel);

  buf[1] = 0x7F;  // 未定義の種別

  proto::Frame f;
  TEST_ASSERT_EQUAL(proto::ParseResult::kUnknownType, proto::ParseFrame(buf, n, &f));
}

void test_reject_truncated(void) {
  const int16_t vel[proto::kNumDriveMotors] = {1, 2, 3, 4, 5, 6};
  uint8_t buf[proto::kMaxFrameSize];
  const size_t n = proto::EncodeMotorVelocity(buf, sizeof(buf), vel);

  proto::Frame f;
  // フレーム全体未達 → kNeedMore
  TEST_ASSERT_EQUAL(proto::ParseResult::kNeedMore, proto::ParseFrame(buf, n - 1, &f));
  // ヘッダすら未達 → kNeedMore
  TEST_ASSERT_EQUAL(proto::ParseResult::kNeedMore, proto::ParseFrame(buf, 1, &f));
  TEST_ASSERT_EQUAL(proto::ParseResult::kNeedMore, proto::ParseFrame(buf, 0, &f));
}

void test_parse_first_frame_only(void) {
  // 複数フレームが連結されたバッファでは先頭フレームのみデコードする
  const int16_t vel[proto::kNumDriveMotors] = {10, 20, 30, 40, 50, 60};
  uint8_t buf[proto::kMaxFrameSize * 2];
  const size_t n = proto::EncodeMotorVelocity(buf, sizeof(buf), vel);
  const size_t n2 = proto::EncodeMotorVelocity(&buf[n], sizeof(buf) - n, vel);

  proto::Frame f;
  TEST_ASSERT_EQUAL(proto::ParseResult::kOk, proto::ParseFrame(buf, n + n2, &f));
  TEST_ASSERT_EQUAL_UINT(n, f.frameSize);
  TEST_ASSERT_EQUAL_INT16(10, proto::FrameGetInt16(f, 0));
}

void test_frame_sizes_fixed(void) {
  // 種別ごとのフレーム長が設計どおり固定であること
  TEST_ASSERT_EQUAL_UINT(15, proto::FrameSize(proto::kCmdMotorVelocity));
  TEST_ASSERT_EQUAL_UINT(15, proto::FrameSize(proto::kCmdSteeringAngle));
  TEST_ASSERT_EQUAL_UINT(11, proto::FrameSize(proto::kCmdArmAngle));
  TEST_ASSERT_EQUAL_UINT(4, proto::FrameSize(proto::kCmdGripper));
  TEST_ASSERT_EQUAL_UINT(17, proto::FrameSize(proto::kFbState));
  TEST_ASSERT_EQUAL_UINT(4, proto::FrameSize(proto::kFbError));
  TEST_ASSERT_EQUAL_UINT(0, proto::PayloadSize(proto::kTypeUnknown));
}

void test_encode_insufficient_buffer(void) {
  const int16_t vel[proto::kNumDriveMotors] = {1, 2, 3, 4, 5, 6};
  uint8_t small[4];  // フレーム長 (15) 未満

  TEST_ASSERT_EQUAL_UINT(0, proto::EncodeMotorVelocity(small, sizeof(small), vel));
}

// ---------------------------------------------------------------------------

int main(void) {
  UNITY_BEGIN();

  RUN_TEST(test_roundtrip_motor_velocity);
  RUN_TEST(test_roundtrip_steering_angle);
  RUN_TEST(test_roundtrip_arm_angle);
  RUN_TEST(test_roundtrip_gripper);
  RUN_TEST(test_roundtrip_state_feedback);
  RUN_TEST(test_roundtrip_error_feedback);

  RUN_TEST(test_checksum_valid_frame);
  RUN_TEST(test_checksum_corrupted_payload);

  RUN_TEST(test_reject_bad_header);
  RUN_TEST(test_reject_bad_checksum);
  RUN_TEST(test_reject_unknown_type);
  RUN_TEST(test_reject_truncated);
  RUN_TEST(test_parse_first_frame_only);
  RUN_TEST(test_frame_sizes_fixed);
  RUN_TEST(test_encode_insufficient_buffer);

  return UNITY_END();
}

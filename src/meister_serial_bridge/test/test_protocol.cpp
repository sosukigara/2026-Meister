#include <gtest/gtest.h>

#include <vector>

#include "meister_serial_bridge/frames.hpp"
#include "meister_serial_bridge/serial_io.hpp"
#include "meister_serial_bridge/stream_parser.hpp"

namespace msb = meister_serial_bridge;
namespace proto = meister::proto;

namespace {

std::vector<uint8_t> VelocityFrame(std::initializer_list<int16_t> vals) {
  msb::VelocityArray a{};
  std::size_t i = 0;
  for (int16_t v : vals) a[i++] = v;
  uint8_t buf[proto::kMaxFrameSize];
  const auto n = msb::EncodeVelocity(buf, sizeof(buf), a);
  return {buf, buf + n};
}

std::vector<uint8_t> SteeringFrame(std::initializer_list<int16_t> vals) {
  msb::SteeringArray a{};
  std::size_t i = 0;
  for (int16_t v : vals) a[i++] = v;
  uint8_t buf[proto::kMaxFrameSize];
  const auto n = msb::EncodeSteering(buf, sizeof(buf), a);
  return {buf, buf + n};
}

std::vector<int16_t> ReadChannels(const proto::Frame& f, std::size_t count) {
  std::vector<int16_t> out;
  for (std::size_t i = 0; i < count; ++i) {
    out.push_back(proto::FrameGetInt16(f, i * sizeof(int16_t)));
  }
  return out;
}

}  // namespace

// ヘッダと種別、ペイロード、チェックサムの配置とリトルエンディアンを検証する。
// Python 版 test_protocol.py からの移植。Python 側は Python 実装を使っていたが、
// 現在は firmware の encoder を直接使うので、このテストが「PC と firmware の
// バイト一致」の担保になる。
TEST(Frames, MotorVelocityLayout) {
  const auto frame = VelocityFrame({1000, 500, 0, -500, -1000, 321});
  EXPECT_EQ(frame.size(), 15u);
  EXPECT_EQ(frame[0], proto::kHeaderByte);
  EXPECT_EQ(frame[1], static_cast<uint8_t>(proto::kCmdMotorVelocity));
  EXPECT_EQ(frame[2], 0xE8);
  EXPECT_EQ(frame[3], 0x03);
  EXPECT_EQ(frame[4], 0xF4);
  EXPECT_EQ(frame[5], 0x01);
  EXPECT_EQ(frame[6], 0x00);
  EXPECT_EQ(frame[7], 0x00);
  EXPECT_EQ(frame[8], 0x0C);
  EXPECT_EQ(frame[9], 0xFE);
  EXPECT_EQ(proto::ComputeChecksum(frame.data(), frame.size()), 0u);
}

TEST(Frames, SteeringClampsToFirmwareRange) {
  const auto frame = SteeringFrame({900, -900, 2000, -2000, 123, -123});
  EXPECT_EQ(frame.size(), 15u);
  EXPECT_EQ(frame[1], static_cast<uint8_t>(proto::kCmdSteeringAngle));

  msb::StreamFrameParser parser;
  std::vector<proto::Frame> out;
  parser.Feed(frame.data(), frame.size(), &out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(ReadChannels(out[0], 6),
            (std::vector<int16_t>{900, -900, 900, -900, 123, -123}));
}

TEST(Frames, ArmAndGripperLayout) {
  msb::ArmArray arm{};
  arm = {0, 900, 1800, 2500};
  uint8_t buf[proto::kMaxFrameSize];
  const auto arm_n = msb::EncodeArm(buf, sizeof(buf), arm);
  EXPECT_EQ(arm_n, 11u);
  EXPECT_EQ(buf[1], static_cast<uint8_t>(proto::kCmdArmAngle));

  const auto gripper_n = msb::EncodeGripper(buf, sizeof(buf), proto::kGripperOpen);
  EXPECT_EQ(gripper_n, 4u);
  EXPECT_EQ(buf[0], proto::kHeaderByte);
  EXPECT_EQ(buf[1], static_cast<uint8_t>(proto::kCmdGripper));
  EXPECT_EQ(buf[2], proto::kGripperOpen);
  EXPECT_EQ(proto::ComputeChecksum(buf, gripper_n), 0u);
}

TEST(Frames, ArmClampsToJointRange) {
  // firmware の ClampArm は肘と手首に kMinJoint/kMaxJoint を使う。PC 側も同じ表で
  // 丸めないと int16 を折り返して別の角度が送られる。
  msb::ArmArray arm{0, 900, 1800, 2500};
  uint8_t buf[proto::kMaxFrameSize];
  msb::EncodeArm(buf, sizeof(buf), arm);
  msb::StreamFrameParser parser;
  std::vector<proto::Frame> out;
  parser.Feed(buf, 11, &out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(ReadChannels(out[0], 4), (std::vector<int16_t>{0, 900, 1800, 1800}));
}

TEST(StreamParser, RestoresFramesSplitByteByByte) {
  msb::StreamFrameParser parser;
  const auto f1 = VelocityFrame({100, -200, 300, -400, 500, -600});
  const auto f2 = SteeringFrame({10, 20, 30, 40, 50, 60});
  std::vector<uint8_t> stream = f1;
  stream.insert(stream.end(), f2.begin(), f2.end());

  std::vector<proto::Frame> out;
  for (uint8_t b : stream) {
    parser.Feed(&b, 1, &out);
  }
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].type, proto::kCmdMotorVelocity);
  EXPECT_EQ(ReadChannels(out[0], 6),
            (std::vector<int16_t>{100, -200, 300, -400, 500, -600}));
  EXPECT_EQ(out[1].type, proto::kCmdSteeringAngle);
}

TEST(StreamParser, DropsCorruptFrameAndResyncs) {
  msb::StreamFrameParser parser;
  auto corrupt = VelocityFrame({1, 2, 3, 4, 5, 6});
  corrupt[5] ^= 0xFF;
  const auto good = VelocityFrame({1, 2, 3, 4, 5, 6});

  std::vector<uint8_t> stream = corrupt;
  stream.insert(stream.end(), good.begin(), good.end());

  std::vector<proto::Frame> out;
  parser.Feed(stream.data(), stream.size(), &out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(proto::FrameGetInt16(out[0], 0), 1);
  EXPECT_TRUE(proto::CheckChecksum(good.data(), good.size()));
}

TEST(StreamParser, ResyncsAfterGarbageBeforeHeader) {
  msb::StreamFrameParser parser;
  std::vector<uint8_t> stream = {0x00, 0xFF, 0xA5, 0x11, 0x22};
  const auto good = VelocityFrame({7, 8, 9, 10, 11, 12});
  stream.insert(stream.end(), good.begin(), good.end());

  std::vector<proto::Frame> out;
  parser.Feed(stream.data(), stream.size(), &out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(ReadChannels(out[0], 6), (std::vector<int16_t>{7, 8, 9, 10, 11, 12}));
}

TEST(Feedback, DecodesStateAndFlags) {
  uint8_t buf[proto::kMaxFrameSize];
  int16_t encoders[proto::kNumDriveMotors] = {1, 2, 3, 4, 5, 6};
  const auto n = proto::EncodeState(buf, sizeof(buf), encoders, 0x03,
                                    proto::kFbErrorProtocol);
  proto::Frame f;
  ASSERT_EQ(proto::ParseFrame(buf, n, &f), proto::ParseResult::kOk);

  const auto fb = msb::DecodeStateFrame(f);
  EXPECT_EQ(fb.state, 0x03);
  EXPECT_EQ(fb.error_flags, proto::kFbErrorProtocol);
  EXPECT_TRUE(fb.HasProtocolError());
  EXPECT_NE(fb.Describe().find("error=0x08"), std::string::npos);
}

TEST(Feedback, IgnoresNonStateFrame) {
  uint8_t buf[proto::kMaxFrameSize];
  const auto n = msb::EncodeGripper(buf, sizeof(buf), proto::kGripperStop);
  proto::Frame f;
  ASSERT_EQ(proto::ParseFrame(buf, n, &f), proto::ParseResult::kOk);
  const auto fb = msb::DecodeStateFrame(f);
  EXPECT_FALSE(fb.HasProtocolError());
  EXPECT_EQ(fb.state, 0u);
}

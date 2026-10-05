#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "fake_esp32.hpp"
#include "meister_serial_bridge/comm_check.hpp"
#include "meister_serial_bridge/stream_parser.hpp"

namespace msb = meister_serial_bridge;
namespace proto = meister::proto;

namespace {

/// フェイクを 1 プロセス内で使い回す。Python 版はクロージャで注入していた。
msb::SerialOpener Opener(msb::test::FakeEsp32* esp) {
  return [esp](const std::string&, int) -> std::shared_ptr<msb::SerialIo> {
    return msb::test::NonOwning(esp);
  };
}

}  // namespace

TEST(CommCheck, PassesWhenEsp32AnswersBothDirections) {
  msb::test::FakeEsp32 esp;
  const auto result = msb::RunCommCheck("fake", msb::kDefaultBaud, 0.2, false, Opener(&esp));

  ASSERT_EQ(result.steps.size(), 3u);
  EXPECT_TRUE(result.ok());
  EXPECT_EQ(result.steps[0].name, "uplink");
  EXPECT_EQ(result.steps[1].name, "downlink");
  EXPECT_EQ(result.steps[2].name, "err_flag");
}

TEST(CommCheck, FailsWhenEsp32NeverAnswers) {
  msb::test::FakeEsp32Options opts;
  opts.reply = false;
  msb::test::FakeEsp32 esp(opts);
  EXPECT_THROW(msb::RunCommCheck("fake", msb::kDefaultBaud, 0.2, false, Opener(&esp)),
               msb::CommError);
}

TEST(CommCheck, FailsWhenValidCommandIsFlaggedAsError) {
  msb::test::FakeEsp32Options opts;
  opts.reject_valid = true;
  msb::test::FakeEsp32 esp(opts);
  EXPECT_THROW(msb::RunCommCheck("fake", msb::kDefaultBaud, 0.2, false, Opener(&esp)),
               msb::CommError);
}

TEST(CommCheck, FailsWhenCorruptFrameIsNotAcknowledged) {
  msb::test::FakeEsp32Options opts;
  opts.notice_corrupt = false;
  msb::test::FakeEsp32 esp(opts);
  EXPECT_THROW(msb::RunCommCheck("fake", msb::kDefaultBaud, 0.2, false, Opener(&esp)),
               msb::CommError);
}

// 実機で実行しても動かないよう、指令は全 0 とグリッパー停止だけ。
TEST(CommCheck, SendsOnlyNeutralCommands) {
  msb::test::FakeEsp32 esp;
  msb::RunCommCheck("fake", msb::kDefaultBaud, 0.2, false, Opener(&esp));

  msb::StreamFrameParser parser;
  std::vector<proto::Frame> frames;
  parser.Feed(esp.written().data(), esp.written().size(), &frames);
  // 手順 3 の壊したフレームはパーサが破棄するので、残るのは有効な指令だけ。
  ASSERT_EQ(frames.size(), 4u);
  EXPECT_EQ(frames[0].type, proto::kCmdMotorVelocity);
  EXPECT_EQ(frames[1].type, proto::kCmdSteeringAngle);
  EXPECT_EQ(frames[2].type, proto::kCmdArmAngle);
  EXPECT_EQ(frames[3].type, proto::kCmdGripper);
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t ch = 0; ch * 2 < frames[i].payloadSize; ++ch) {
      EXPECT_EQ(proto::FrameGetInt16(frames[i], ch * 2), 0);
    }
  }
  EXPECT_EQ(proto::FrameGetU8(frames[3], 0), proto::kGripperStop);
}

// DTR は IO0 に直結している。assert のままだとブートローダで止まる。
TEST(CommCheck, ReleasesDtrBeforeAnyIo) {
  msb::test::FakeEsp32 ok;
  EXPECT_TRUE(msb::RunCommCheck("fake", msb::kDefaultBaud, 0.2, false, Opener(&ok)).ok());

  msb::test::FakeEsp32Options opts;
  opts.dtr_stuck = true;
  msb::test::FakeEsp32 stuck(opts);
  EXPECT_THROW(msb::RunCommCheck("fake", msb::kDefaultBaud, 0.2, false, Opener(&stuck)),
               msb::CommError);
}

TEST(CommCheck, NeutralFramesAreAllZero) {
  const auto frames = msb::NeutralCommandFrames();
  ASSERT_EQ(frames.size(), 4u);
  EXPECT_EQ(frames[0].size(), proto::FrameSize(proto::kCmdMotorVelocity));
  EXPECT_EQ(frames[1].size(), proto::FrameSize(proto::kCmdSteeringAngle));
  EXPECT_EQ(frames[2].size(), proto::FrameSize(proto::kCmdArmAngle));
  EXPECT_EQ(frames[3].size(), proto::FrameSize(proto::kCmdGripper));
  for (const auto& frame : frames) {
    EXPECT_EQ(proto::CheckChecksum(frame.data(), frame.size()), true);
  }
  EXPECT_EQ(frames[3][2], proto::kGripperStop);
}

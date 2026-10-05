#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "fake_esp32.hpp"
#include "meister_serial_bridge/feedback_hz_measure.hpp"
#include "meister_serial_bridge/stream_parser.hpp"

namespace msb = meister_serial_bridge;
namespace proto = meister::proto;

namespace {

/// あらかじめ用意したバイト列を返すだけのテストダブル。
class ScriptedEsp32 final : public msb::SerialIo {
 public:
  explicit ScriptedEsp32(std::vector<std::vector<uint8_t>> chunks)
      : chunks_(std::move(chunks)) {}

  std::size_t Read(uint8_t* out, std::size_t max, int) override {
    if (buf_.empty() && !chunks_.empty()) {
      const auto chunk = chunks_.front();
      chunks_.erase(chunks_.begin());
      buf_.insert(buf_.end(), chunk.begin(), chunk.end());
    }
    const std::size_t n = std::min(max, buf_.size());
    std::copy(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(n), out);
    buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(n));
    return n;
  }

  void Write(const uint8_t* data, std::size_t len) override {
    written_.insert(written_.end(), data, data + len);
  }

  void Flush() override {}

  void ResetInputBuffer() override { buf_.clear(); }

  void SetDtr(bool) override {}

  void SetRts(bool) override {}

  const std::vector<uint8_t>& written() const { return written_; }

 private:
  std::vector<std::vector<uint8_t>> chunks_;
  std::vector<uint8_t> buf_;
  std::vector<uint8_t> written_;
};

std::vector<uint8_t> StateFrame(uint8_t error_flags) {
  int16_t encoders[proto::kNumDriveMotors] = {};
  uint8_t buf[proto::kMaxFrameSize];
  const auto n = proto::EncodeState(buf, sizeof(buf), encoders, 0, error_flags);
  return {buf, buf + n};
}

std::vector<uint8_t> Repeat(const std::vector<uint8_t>& frame, int times) {
  std::vector<uint8_t> out;
  for (int i = 0; i < times; ++i) {
    out.insert(out.end(), frame.begin(), frame.end());
  }
  return out;
}

msb::SerialOpener Opener(msb::SerialIo* esp) {
  return [esp](const std::string&, int) -> std::shared_ptr<msb::SerialIo> {
    return msb::test::NonOwning(esp);
  };
}

constexpr double kDurationS = 0.15;

}  // namespace

TEST(FeedbackHz, CountsEveryFrameWithoutLoss) {
  ScriptedEsp32 esp({Repeat(StateFrame(0), 20)});
  const auto m = msb::MeasureFeedbackHz("fake", msb::kDefaultBaud, kDurationS, false, false,
                                        Opener(&esp));
  EXPECT_EQ(m.frames, 20);
  EXPECT_EQ(m.LostFrames(), 0);
  EXPECT_EQ(m.protocol_errors, 0);
  EXPECT_EQ(m.commands_sent, 0);
}

TEST(FeedbackHz, CountsProtocolErrorFlagFrames) {
  ScriptedEsp32 esp({Repeat(StateFrame(0), 10), StateFrame(proto::kFbErrorProtocol)});
  const auto m = msb::MeasureFeedbackHz("fake", msb::kDefaultBaud, kDurationS, false, false,
                                        Opener(&esp));
  EXPECT_EQ(m.frames, 11);
  EXPECT_EQ(m.protocol_errors, 1);
}

TEST(FeedbackHz, DetectsFrameDroppedOnTheWire) {
  // 壊れた（または UART 側で失われた）フレームはバイト数だけ増えて復号数が
  // 増えないので、欠落として検出できる。
  auto corrupt = StateFrame(0);
  corrupt.back() ^= 0xFF;
  auto stream = Repeat(StateFrame(0), 10);
  stream.insert(stream.end(), corrupt.begin(), corrupt.end());

  ScriptedEsp32 esp({stream});
  const auto m = msb::MeasureFeedbackHz("fake", msb::kDefaultBaud, kDurationS, false, false,
                                        Opener(&esp));
  EXPECT_EQ(m.frames, 10);
  EXPECT_EQ(m.LostFrames(), 1);
}

TEST(FeedbackHz, SendsNeutralCommandsWhenRequested) {
  ScriptedEsp32 esp({Repeat(StateFrame(0), 20)});
  const auto m = msb::MeasureFeedbackHz("fake", msb::kDefaultBaud, kDurationS,
                                        /*with_commands=*/true, false, Opener(&esp));
  EXPECT_GE(m.commands_sent, 1);

  msb::StreamFrameParser parser;
  std::vector<proto::Frame> frames;
  parser.Feed(esp.written().data(), esp.written().size(), &frames);
  ASSERT_FALSE(frames.empty());
  for (const auto& frame : frames) {
    for (std::size_t ch = 0; ch * 2 < frame.payloadSize; ++ch) {
      EXPECT_EQ(proto::FrameGetInt16(frame, ch * 2), 0);
    }
  }
}

TEST(FeedbackHz, ReportsGapStatistics) {
  ScriptedEsp32 esp({Repeat(StateFrame(0), 20)});
  const auto m = msb::MeasureFeedbackHz("fake", msb::kDefaultBaud, kDurationS, false, false,
                                        Opener(&esp));
  ASSERT_EQ(m.frames, 20);
  EXPECT_GT(m.measured_hz, 0.0);
  EXPECT_GE(m.max_ms, m.p95_ms);
  EXPECT_GE(m.p95_ms, m.mean_ms);
}

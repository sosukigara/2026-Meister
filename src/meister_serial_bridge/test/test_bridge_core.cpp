#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "fake_esp32.hpp"
#include "meister_serial_bridge/bridge_core.hpp"
#include "meister_serial_bridge/comm_check.hpp"
#include "meister_serial_bridge/stream_parser.hpp"

namespace msb = meister_serial_bridge;
namespace proto = meister::proto;

namespace {

/// テストから明示的に進められる単調時計。
class FakeClock {
 public:
  double now() const { return now_s_; }
  void Advance(double seconds) { now_s_ += seconds; }

 private:
  double now_s_ = 1000.0;
};

struct Harness {
  // serial_link.cpp の LibSerialIo は open 後に DTR を解放する。
  msb::test::FakeEsp32 esp{MakeEspOptions()};
  FakeClock clock;
  std::unique_ptr<msb::BridgeCore> core;

  static msb::test::FakeEsp32Options MakeEspOptions() {
    msb::test::FakeEsp32Options opts;
    opts.dtr_asserted_at_open = false;
    return opts;
  }

  Harness() {
    msb::BridgeParams params;
    params.serial_port = "fake";
    params.baud = msb::kDefaultBaud;
    params.kinematics.wheelbase_m = 0.3;
    params.kinematics.max_linear_vel_mps = 1.0;
    params.kinematics.max_steering_deg = 30.0;
    params.cmd_timeout_s = 0.5;
    auto* esp_ptr = &esp;
    core = std::make_unique<msb::BridgeCore>(
        params,
        [esp_ptr](const std::string&, int) -> std::shared_ptr<msb::SerialIo> {
          return msb::test::NonOwning(esp_ptr);
        },
        [this] { return clock.now(); });
  }

  /// 送られたバイトをフレームに復元して種別を並べる。
  std::vector<proto::TypeId> WrittenTypes() {
    msb::StreamFrameParser parser;
    std::vector<proto::Frame> frames;
    parser.Feed(esp.written().data(), esp.written().size(), &frames);
    std::vector<proto::TypeId> out;
    for (const auto& f : frames) {
      out.push_back(f.type);
    }
    return out;
  }
};

}  // namespace

TEST(BridgeCore, CmdVelWritesSteeringThenVelocity) {
  Harness h;
  h.core->OnCmdVel(0.5, 0.0);
  EXPECT_EQ(h.WrittenTypes(),
            (std::vector<proto::TypeId>{proto::kCmdSteeringAngle, proto::kCmdMotorVelocity}));

  msb::StreamFrameParser parser;
  std::vector<proto::Frame> frames;
  parser.Feed(h.esp.written().data(), h.esp.written().size(), &frames);
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(proto::FrameGetInt16(frames[0], 0), 0);
  EXPECT_EQ(proto::FrameGetInt16(frames[1], 0), 500);
}

// 関節角 1 通で CMD_ARM_ANGLE が 1 フレームだけ出る。
TEST(BridgeCore, ArmMessageWritesOneArmFrame) {
  Harness h;
  ASSERT_TRUE(h.core->OnArmJoint({0, 900, 1800, 450}));
  EXPECT_EQ(h.WrittenTypes(), (std::vector<proto::TypeId>{proto::kCmdArmAngle}));

  msb::StreamFrameParser parser;
  std::vector<proto::Frame> frames;
  parser.Feed(h.esp.written().data(), h.esp.written().size(), &frames);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(proto::FrameGetInt16(frames[0], 0), 0);
  EXPECT_EQ(proto::FrameGetInt16(frames[0], 2), 900);
  EXPECT_EQ(proto::FrameGetInt16(frames[0], 4), 1800);
  EXPECT_EQ(proto::FrameGetInt16(frames[0], 6), 450);
}

TEST(BridgeCore, GripperMessageWritesOneGripperFrame) {
  Harness h;
  h.core->OnGripper(proto::kGripperOpen);
  EXPECT_EQ(h.WrittenTypes(), (std::vector<proto::TypeId>{proto::kCmdGripper}));
}

// 値域外は firmware と同じ表で丸めて送る。折り返して別角度になるのを防ぐ。
TEST(BridgeCore, OutOfRangeJointIsClampedNotWrapped) {
  Harness h;
  ASSERT_TRUE(h.core->OnArmJoint({-100, 900, 1800, 2500}));
  msb::StreamFrameParser parser;
  std::vector<proto::Frame> frames;
  parser.Feed(h.esp.written().data(), h.esp.written().size(), &frames);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(proto::FrameGetInt16(frames[0], 0), 0);
  EXPECT_EQ(proto::FrameGetInt16(frames[0], 6), 1800);
}

// 軸数が違うメッセージでノードを落とさない。
TEST(BridgeCore, WrongJointCountIsIgnored) {
  Harness h;
  EXPECT_FALSE(h.core->OnArmJoint({0, 900, 1800}));
  EXPECT_FALSE(h.core->OnArmJoint({0, 900, 1800, 450, 500}));
  EXPECT_TRUE(h.esp.written().empty());
}

TEST(BridgeCore, WatchdogStopsAfterTimeoutOnlyOnce) {
  Harness h;
  h.core->OnCmdVel(0.5, 0.0);

  h.clock.Advance(0.4);
  EXPECT_FALSE(h.core->OnWatchdog(h.clock.now()).send_stop);

  h.clock.Advance(0.2);
  EXPECT_TRUE(h.core->OnWatchdog(h.clock.now()).send_stop);

  const auto written_after_first_stop = h.esp.written().size();
  EXPECT_FALSE(h.core->OnWatchdog(h.clock.now()).send_stop);
  EXPECT_EQ(h.esp.written().size(), written_after_first_stop);
}

TEST(BridgeCore, CmdVelResetsWatchdog) {
  Harness h;
  h.core->OnCmdVel(0.5, 0.0);
  h.clock.Advance(0.4);
  h.core->OnCmdVel(0.5, 0.0);
  h.clock.Advance(0.4);
  EXPECT_FALSE(h.core->OnWatchdog(h.clock.now()).send_stop);
}

// アームは停止させず最終指令を保持する。警告は遷移したとき 1 回だけ。
TEST(BridgeCore, ArmTimeoutHoldsLastTarget) {
  Harness h;
  h.core->OnCmdVel(0.5, 0.0);
  ASSERT_TRUE(h.core->OnArmJoint({0, 900, 1800, 450}));
  const auto before = h.esp.written().size();

  // 速度指令だけ延長し、アームだけを無通信にする。両方切ると停止フレームが
  // 混ざって「送っていない」を測れなくなる。
  h.clock.Advance(0.6);
  h.core->OnCmdVel(0.5, 0.0);
  const auto after_cmd = h.esp.written().size();
  ASSERT_GT(after_cmd, before);
  const auto verdict = h.core->OnWatchdog(h.clock.now());
  EXPECT_TRUE(verdict.arm_idle_started);
  EXPECT_EQ(h.esp.written().size(), after_cmd);

  // 2 回目は警告しない（遷移していない）。
  EXPECT_FALSE(h.core->OnWatchdog(h.clock.now()).arm_idle_started);
}

TEST(BridgeCore, ArmCommandClearsIdleWarning) {
  Harness h;
  h.clock.Advance(0.6);
  EXPECT_TRUE(h.core->OnWatchdog(h.clock.now()).arm_idle_started);

  ASSERT_TRUE(h.core->OnArmJoint({0, 900, 1800, 450}));
  h.clock.Advance(0.6);
  EXPECT_TRUE(h.core->OnWatchdog(h.clock.now()).arm_idle_started);
}

TEST(BridgeCore, SendStopWritesZeroFrames) {
  Harness h;
  h.core->SendStop();
  EXPECT_EQ(h.WrittenTypes(),
            (std::vector<proto::TypeId>{proto::kCmdSteeringAngle, proto::kCmdMotorVelocity}));

  msb::StreamFrameParser parser;
  std::vector<proto::Frame> frames;
  parser.Feed(h.esp.written().data(), h.esp.written().size(), &frames);
  ASSERT_EQ(frames.size(), 2u);
  for (std::size_t ch = 0; ch * 2 < frames[0].payloadSize; ++ch) {
    EXPECT_EQ(proto::FrameGetInt16(frames[0], ch * 2), 0);
  }
  for (std::size_t ch = 0; ch * 2 < frames[1].payloadSize; ++ch) {
    EXPECT_EQ(proto::FrameGetInt16(frames[1], ch * 2), 0);
  }
}

TEST(BridgeCore, RxOnceDecodesStateFrames) {
  Harness h;
  int16_t encoders[proto::kNumDriveMotors] = {1, 2, 3, 4, 5, 6};
  uint8_t buf[proto::kMaxFrameSize];
  const auto n = proto::EncodeState(buf, sizeof(buf), encoders, 0x02, 0);
  h.esp.InjectRx(buf, n);

  std::vector<msb::Feedback> states;
  std::vector<uint8_t> errors;
  ASSERT_TRUE(h.core->RxOnce(&states, &errors, 0));
  ASSERT_EQ(states.size(), 1u);
  EXPECT_EQ(states[0].state, 0x02);
  EXPECT_EQ(states[0].encoders[5], 6);
  EXPECT_TRUE(errors.empty());
}

TEST(BridgeCore, RxOnceReportsErrorFrames) {
  Harness h;
  uint8_t buf[proto::kMaxFrameSize];
  const auto n = proto::EncodeError(buf, sizeof(buf), 0x07);
  h.esp.InjectRx(buf, n);

  std::vector<msb::Feedback> states;
  std::vector<uint8_t> errors;
  ASSERT_TRUE(h.core->RxOnce(&states, &errors, 0));
  EXPECT_TRUE(states.empty());
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_EQ(errors[0], 0x07);
}

TEST(BridgeCore, RxOnceKeepsPartialFrameAcrossCalls) {
  Harness h;
  int16_t encoders[proto::kNumDriveMotors] = {7, 7, 7, 7, 7, 7};
  uint8_t frame[proto::kMaxFrameSize];
  const auto n = proto::EncodeState(frame, sizeof(frame), encoders, 0, 0);

  // 途中まで喂って、残りで 1 フレームが復元されることを確かめる。
  h.esp.InjectRx(frame, 5);
  std::vector<msb::Feedback> states;
  std::vector<uint8_t> errors;
  ASSERT_TRUE(h.core->RxOnce(&states, &errors, 0));
  EXPECT_TRUE(states.empty());

  h.esp.InjectRx(frame + 5, n - 5);
  ASSERT_TRUE(h.core->RxOnce(&states, &errors, 0));
  ASSERT_EQ(states.size(), 1u);
  EXPECT_EQ(states[0].encoders[0], 7);
}

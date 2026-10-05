#include "meister_serial_bridge/bridge_core.hpp"

#include "meister_serial_bridge/frames.hpp"

namespace meister_serial_bridge {

namespace proto = meister::proto;

namespace {
constexpr std::size_t kRxBufBytes = 256;
}  // namespace

BridgeCore::BridgeCore(BridgeParams params, LinkFactory factory, Clock clock)
    : params_(std::move(params)),
      factory_(std::move(factory)),
      clock_(std::move(clock)),
      last_cmd_time_s_(clock_()),
      last_arm_time_s_(clock_()) {}

std::shared_ptr<SerialIo> BridgeCore::CurrentLink(std::string* error) {
  std::lock_guard<std::mutex> lock(link_mutex_);
  if (link_) {
    return link_;
  }
  const double now = clock_();
  if (now < next_open_attempt_s_) {
    return nullptr;
  }
  next_open_attempt_s_ = now + params_.reopen_interval_s;
  try {
    link_ = factory_(params_.serial_port, params_.baud);
  } catch (const std::exception& e) {
    if (error) {
      *error = e.what();
    }
    return nullptr;
  }
  if (!link_ && error) {
    *error = "factory returned no link";
  }
  return link_;
}

void BridgeCore::DropLink() {
  std::lock_guard<std::mutex> lock(link_mutex_);
  link_.reset();
}

void BridgeCore::WriteJoined(const std::vector<uint8_t>& bytes) {
  const std::shared_ptr<SerialIo> link = CurrentLink();
  if (!link) {
    return;
  }
  link->Write(bytes.data(), bytes.size());
  link->Flush();
}

void BridgeCore::WriteFrames(const std::vector<std::vector<uint8_t>>& frames) {
  std::vector<uint8_t> joined;
  for (const auto& f : frames) {
    joined.insert(joined.end(), f.begin(), f.end());
  }
  std::lock_guard<std::mutex> lock(tx_mutex_);
  WriteJoined(joined);
}

void BridgeCore::OnCmdVel(double vx, double wz) {
  const ActuatorCommand cmd = TwistToActuators(vx, wz, params_.kinematics);

  uint8_t buf[proto::kMaxFrameSize];
  std::vector<std::vector<uint8_t>> frames;
  const auto steer =
      EncodeSteering(buf, sizeof(buf), Uniform<SteeringArray>(cmd.steering_tenths));
  frames.emplace_back(buf, buf + steer);
  const auto vel =
      EncodeVelocity(buf, sizeof(buf), Uniform<VelocityArray>(cmd.velocity_permille));
  frames.emplace_back(buf, buf + vel);
  WriteFrames(frames);

  last_cmd_time_s_ = clock_();
  zero_sent_ = false;
}

bool BridgeCore::OnArmJoint(const std::vector<int16_t>& angles) {
  if (angles.size() != kArmChannels) {
    return false;
  }
  ArmArray arr{};
  for (std::size_t i = 0; i < arr.size(); ++i) {
    arr[i] = angles[i];
  }
  // クランプはフレーム境界（frames.hpp）。値域は firmware と同じ 1 か所。
  uint8_t buf[proto::kMaxFrameSize];
  const auto n = EncodeArm(buf, sizeof(buf), arr);
  WriteFrames({std::vector<uint8_t>(buf, buf + n)});
  last_arm_time_s_ = clock_();
  arm_idle_logged_ = false;
  return true;
}

void BridgeCore::OnGripper(uint8_t cmd) {
  uint8_t buf[proto::kMaxFrameSize];
  const auto n = EncodeGripper(buf, sizeof(buf), cmd);
  WriteFrames({std::vector<uint8_t>(buf, buf + n)});
}

void BridgeCore::SendStop() {
  uint8_t buf[proto::kMaxFrameSize];
  std::vector<uint8_t> joined;
  auto add = [&](std::size_t n) { joined.insert(joined.end(), buf, buf + n); };
  add(EncodeSteering(buf, sizeof(buf), Uniform<SteeringArray>(0)));
  add(EncodeVelocity(buf, sizeof(buf), Uniform<VelocityArray>(0)));
  std::lock_guard<std::mutex> lock(tx_mutex_);
  WriteJoined(joined);
  zero_sent_ = true;
}

WatchdogVerdict BridgeCore::OnWatchdog(double now_s) {
  WatchdogVerdict verdict;
  // アームは停止させず最終指令を保持する。CMD_ARM_ANGLE は絶対位置の目標値
  // （firmware は Arm::setAngleTenths）なので、途絶えたときのアームはすでに
  // 最終目標で静止している。0 を送ると原点への移動になり、通信断のたびに
  // アームが動き出す。保持だけが無害なのでフレームは送らない。
  if (!arm_idle_logged_ && now_s >= last_arm_time_s_ + params_.cmd_timeout_s) {
    arm_idle_logged_ = true;
    verdict.arm_idle_started = true;
  }
  if (zero_sent_ || now_s < last_cmd_time_s_ + params_.cmd_timeout_s) {
    return verdict;
  }
  SendStop();
  verdict.send_stop = true;
  return verdict;
}

bool BridgeCore::RxOnce(std::vector<Feedback>* states, std::vector<uint8_t>* fb_errors,
                        int timeout_ms) {
  const std::shared_ptr<SerialIo> link = CurrentLink();
  if (!link) {
    return false;
  }
  uint8_t buf[kRxBufBytes];
  const std::size_t n = link->Read(buf, sizeof(buf), timeout_ms);
  if (n == 0) {
    return true;
  }
  std::vector<proto::Frame> frames;
  parser_.Feed(buf, n, &frames);
  for (const auto& frame : frames) {
    if (frame.type == proto::kFbState) {
      states->push_back(DecodeStateFrame(frame));
    } else if (frame.type == proto::kFbError) {
      fb_errors->push_back(frame.payload[0]);
    }
  }
  return true;
}

}  // namespace meister_serial_bridge

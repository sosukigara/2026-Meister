#include "meister_serial_bridge/frames.hpp"

namespace meister_serial_bridge {
namespace {

int16_t ClampTo(int16_t v, int16_t lo, int16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

template <std::size_t N>
void ClampAll(std::array<int16_t, N>* values, int16_t lo, int16_t hi) {
  for (auto& v : *values) {
    v = ClampTo(v, lo, hi);
  }
}

}  // namespace

size_t EncodeSteering(uint8_t* buf, std::size_t cap, const SteeringArray& tenths) {
  SteeringArray clamped = tenths;
  ClampAll(&clamped, meister::proto::kMinSteering, meister::proto::kMaxSteering);
  const int16_t channels[kSteerChannels] = {clamped[0], clamped[1], clamped[2],
                                           clamped[3], clamped[4], clamped[5]};
  return meister::proto::EncodeSteeringAngle(buf, cap, channels);
}

size_t EncodeVelocity(uint8_t* buf, std::size_t cap, const VelocityArray& permille) {
  VelocityArray clamped = permille;
  ClampAll(&clamped, meister::proto::kMinVelocity, meister::proto::kMaxVelocity);
  const int16_t channels[kDriveChannels] = {clamped[0], clamped[1], clamped[2],
                                            clamped[3], clamped[4], clamped[5]};
  return meister::proto::EncodeMotorVelocity(buf, cap, channels);
}

size_t EncodeArm(uint8_t* buf, std::size_t cap, const ArmArray& tenths) {
  ArmArray clamped = tenths;
  // firmware の ClampArm は肘と手首に kMinJoint/kMaxJoint を使う。別の表だと
  // PC 側だけ先に効いて、線上のバイトが firmware の解釈と食い違う。
  ClampAll(&clamped, meister::config::kMinJoint, meister::config::kMaxJoint);
  const int16_t channels[kArmChannels] = {clamped[0], clamped[1], clamped[2], clamped[3]};
  return meister::proto::EncodeArmAngle(buf, cap, channels);
}

size_t EncodeGripper(uint8_t* buf, std::size_t cap, uint8_t cmd) {
  const auto value = static_cast<meister::proto::GripperCommand>(ClampTo(cmd, 0, 2));
  return meister::proto::EncodeGripper(buf, cap, value);
}

}  // namespace meister_serial_bridge

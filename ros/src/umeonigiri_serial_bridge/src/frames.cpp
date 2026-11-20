#include "umeonigiri_serial_bridge/frames.hpp"

#include <algorithm>

namespace umeonigiri_serial_bridge {
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

/// firmware の Encode* は C 配列の参照を取る。軸数を変えたときに初期化子の
/// 個数で壊れないよう、配列長はプロトコル定数から取る。
template <std::size_t N>
void CopyChannels(const std::array<int16_t, N>& src, int16_t (&dst)[N]) {
  std::copy(src.begin(), src.end(), dst);
}

}  // namespace

size_t EncodeSteering(uint8_t* buf, std::size_t cap, const SteeringArray& tenths) {
  SteeringArray clamped = tenths;
  ClampAll(&clamped, meister::proto::kMinSteering, meister::proto::kMaxSteering);
  int16_t channels[kSteerChannels];
  CopyChannels(clamped, channels);
  return meister::proto::EncodeSteeringAngle(buf, cap, channels);
}

size_t EncodeVelocity(uint8_t* buf, std::size_t cap, const VelocityArray& permille) {
  VelocityArray clamped = permille;
  ClampAll(&clamped, meister::proto::kMinVelocity, meister::proto::kMaxVelocity);
  int16_t channels[kDriveChannels];
  CopyChannels(clamped, channels);
  return meister::proto::EncodeMotorVelocity(buf, cap, channels);
}

size_t EncodeArm(uint8_t* buf, std::size_t cap, const ArmArray& tenths) {
  ArmArray clamped = tenths;
  // CMD_ARM_ANGLE は Arm::setAngles -> PwmServoChannel::setAngleTenths の経路で
  // kMinArmAngle/kMaxArmAngle に丸められる。ClampArm（kMinJoint/kMaxJoint）は
  // 別経路の表であり、ここで使うと PC 側だけ先に効いて食い違う。
  ClampAll(&clamped, meister::config::kMinArmAngle, meister::config::kMaxArmAngle);
  int16_t channels[kArmChannels];
  CopyChannels(clamped, channels);
  return meister::proto::EncodeArmAngle(buf, cap, channels);
}

size_t EncodeGripper(uint8_t* buf, std::size_t cap, uint8_t cmd) {
  const auto value = static_cast<meister::proto::GripperCommand>(ClampTo(cmd, 0, 2));
  return meister::proto::EncodeGripper(buf, cap, value);
}

}  // namespace umeonigiri_serial_bridge

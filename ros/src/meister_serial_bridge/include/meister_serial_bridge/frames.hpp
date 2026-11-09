#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "meister_protocol.h"

namespace meister_serial_bridge {

constexpr std::size_t kDriveChannels = meister::proto::kNumDriveMotors;
constexpr std::size_t kSteerChannels = meister::proto::kNumSteeringServos;
constexpr std::size_t kArmChannels = meister::proto::kNumArmServos;

using VelocityArray = std::array<int16_t, kDriveChannels>;
using SteeringArray = std::array<int16_t, kSteerChannels>;
using ArmArray = std::array<int16_t, kArmChannels>;

/// 全チャネルに同じ値を入れる。6 輪モデルでは舵角と速度を全軸に流す。
template <typename Array>
Array Uniform(int16_t v) {
  Array a{};
  a.fill(v);
  return a;
}

/// 送信用フレームの組み立て。0 を返すとバッファ不足（firmware の contract と同じ）。
///
/// firmware の Encode* は丸めない。受信側で command.h の Clamp が効くが、
/// int16 を超える値は PutI16 で折り返されるので、PC 側のフレーム境界で
/// 同じ値域に丸める。値域は hal/generated_config.h の 1 か所から取る。
size_t EncodeSteering(uint8_t* buf, std::size_t cap, const SteeringArray& tenths);
size_t EncodeVelocity(uint8_t* buf, std::size_t cap, const VelocityArray& permille);
size_t EncodeArm(uint8_t* buf, std::size_t cap, const ArmArray& tenths);
size_t EncodeGripper(uint8_t* buf, std::size_t cap, uint8_t cmd);

}  // namespace meister_serial_bridge

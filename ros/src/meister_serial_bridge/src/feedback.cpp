#include "meister_serial_bridge/serial_io.hpp"

#include <cmath>
#include <sstream>

namespace meister_serial_bridge {
namespace {

std::string ToHex2(uint8_t v) {
  static const char kDigits[] = "0123456789ABCDEF";
  std::string out(2, '0');
  out[0] = kDigits[(v >> 4) & 0x0F];
  out[1] = kDigits[v & 0x0F];
  return out;
}

}  // namespace

std::string Feedback::Describe() const {
  std::ostringstream os;
  os << "enc(placeholder)={";
  for (std::size_t i = 0; i < encoders.size(); ++i) {
    if (i > 0) os << ", ";
    os << encoders[i];
  }
  os << "} state=0x" << ToHex2(state) << " error=0x" << ToHex2(error_flags);
  return os.str();
}

ImuReport DecodeImuFrame(const meister::proto::Frame& frame) {
  ImuReport imu;
  if (frame.type != meister::proto::kFbImu) {
    return imu;
  }
  imu.yaw_tenths = meister::proto::FrameGetInt16(frame, 0);
  imu.calib = meister::proto::FrameGetU8(frame, 2);
  return imu;
}

YawQuat YawTenthsToQuat(int16_t yaw_tenths) {
  // 0.1° → rad。yaw のみの回転なので x=y=0。
  const double half = static_cast<double>(yaw_tenths) * M_PI / 3600.0;
  return YawQuat{std::sin(half), std::cos(half)};
}

Feedback DecodeStateFrame(const meister::proto::Frame& frame) {
  Feedback fb;
  if (frame.type != meister::proto::kFbState) {
    return fb;
  }
  for (std::size_t i = 0; i < fb.encoders.size(); ++i) {
    fb.encoders[i] = meister::proto::FrameGetInt16(frame, i * sizeof(int16_t));
  }
  const std::size_t enc_end = fb.encoders.size() * sizeof(int16_t);
  fb.state = meister::proto::FrameGetU8(frame, enc_end);
  fb.error_flags = meister::proto::FrameGetU8(frame, enc_end + 1);
  return fb;
}

}  // namespace meister_serial_bridge

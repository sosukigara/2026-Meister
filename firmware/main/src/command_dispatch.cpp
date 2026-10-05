#ifdef ARDUINO

#include <Arduino.h>

#include "command_dispatch.h"

#include "meister_config.h"

namespace meister {

void CommandDispatch::begin() {
  chassis_.begin();
  arm_.begin();
}

void CommandDispatch::feedRxByte(uint8_t byte) {
  using namespace proto;
  switch (rxState_) {
    case RxState::kWaitHeader:
      if (byte == kHeaderByte) {
        rxBuf_[0] = byte;
        rxState_ = RxState::kWaitType;
      }
      break;

    case RxState::kWaitType: {
      const TypeId type = static_cast<TypeId>(byte);
      if (PayloadSize(type) == 0) {
        rxState_ = RxState::kWaitHeader;  // 未知種別 → 再同期
        return;
      }
      rxBuf_[1] = byte;
      rxPayloadLeft_ = PayloadSize(type);
      rxIndex_ = 2;
      rxState_ = (rxPayloadLeft_ > 0) ? RxState::kPayload : RxState::kChecksum;
      break;
    }

    case RxState::kPayload:
      if (rxIndex_ < proto::kMaxFrameSize) {
        rxBuf_[rxIndex_++] = byte;
      }
      if (--rxPayloadLeft_ == 0) {
        rxState_ = RxState::kChecksum;
      }
      break;

    case RxState::kChecksum:
      if (rxIndex_ < proto::kMaxFrameSize) {
        rxBuf_[rxIndex_++] = byte;
      }
      onFrameComplete(rxBuf_, rxIndex_);
      rxState_ = RxState::kWaitHeader;
      break;
  }
}

void CommandDispatch::onFrameComplete(const uint8_t* data, size_t len) {
  using namespace proto;
  Frame frame;
  const ParseResult result = ParseFrame(data, len, &frame);
  if (result != ParseResult::kOk) {
    ++rxErrorCount_;
    MSTE_LOG("[rx] invalid frame: %s (count=%lu)\n", ParseResultName(result),
             static_cast<unsigned long>(rxErrorCount_));
    return;
  }

  switch (frame.type) {
    case kCmdMotorVelocity: {
      int16_t velocities[kNumDriveMotors];
      for (size_t i = 0; i < kNumDriveMotors; ++i) {
        velocities[i] = FrameGetInt16(frame, i * 2);
      }
      chassis_.setVelocities(velocities);
      break;
    }

    case kCmdSteeringAngle: {
      int16_t angles[kNumSteeringServos];
      for (size_t i = 0; i < kNumSteeringServos; ++i) {
        angles[i] = FrameGetInt16(frame, i * 2);
      }
      chassis_.setSteering(angles);
      break;
    }

    case kCmdArmAngle: {
      int16_t angles[kNumArmServos];
      for (size_t i = 0; i < kNumArmServos; ++i) {
        angles[i] = FrameGetInt16(frame, i * 2);
      }
      arm_.setAngles(angles);
      break;
    }

    case kCmdGripper:
      arm_.setGripperCommand(static_cast<GripperCommand>(FrameGetU8(frame, 0)));
      break;

    default:  // FB_STATE / FB_ERROR など上り方向は受信側では無視
      break;
  }
}

uint8_t CommandDispatch::rxErrorFlags() const {
  return rxErrorCount_ > 0 ? proto::kFbErrorProtocol : proto::kFbStateNormal;
}

}  // namespace meister

#endif  // ARDUINO

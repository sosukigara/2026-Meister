#ifdef ARDUINO

#include <Arduino.h>

#include "arm.h"

#include "meister_config.h"

namespace meister {

void Gripper::setCommand(proto::GripperCommand cmd) {
  switch (cmd) {
    case proto::kGripperOpen:
      servo_.setAngleTenths(kOpenTenths);  // 開
      break;
    case proto::kGripperClose:
      servo_.setAngleTenths(kCloseTenths);  // 閉
      break;
    case proto::kGripperStop:
    default:
      break;  // 現在値を保持
  }
}

Arm::Arm()
    : armServos_{ArmServoImpl(config::kArmPins[0], proto::kMinArmAngle, proto::kMaxArmAngle),
                 ArmServoImpl(config::kArmPins[1], proto::kMinArmAngle, proto::kMaxArmAngle),
                 ArmServoImpl(config::kArmPins[2], proto::kMinArmAngle, proto::kMaxArmAngle),
                 ArmServoImpl(config::kArmPins[3], proto::kMinArmAngle, proto::kMaxArmAngle)},
      gripperServoChannel_(config::kGripperPin, proto::kMinArmAngle, proto::kMaxArmAngle),
      gripper_(gripperServoChannel_) {}

void Arm::begin() {
  for (auto& s : armServos_) {
    s.begin();
  }
  gripperServoChannel_.begin();
}

void Arm::setAngles(const int16_t (&angles)[proto::kNumArmServos]) {
  for (size_t i = 0; i < proto::kNumArmServos; ++i) {
    armServos_[i].setAngleTenths(angles[i]);
  }
}

void Arm::setGripperCommand(proto::GripperCommand cmd) { gripper_.setCommand(cmd); }

}  // namespace meister

#endif  // ARDUINO

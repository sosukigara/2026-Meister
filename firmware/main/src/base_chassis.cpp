#ifdef ARDUINO

#include <Arduino.h>

#include "base_chassis.h"

#include "meister_config.h"

namespace meister {

BaseChassis::BaseChassis()
    : motors_{hal::MotorChannel(config::kMotorPins[0], config::kMotorDirPins[0]),
              hal::MotorChannel(config::kMotorPins[1], config::kMotorDirPins[1]),
              hal::MotorChannel(config::kMotorPins[2], config::kMotorDirPins[2]),
              hal::MotorChannel(config::kMotorPins[3], config::kMotorDirPins[3]),
              hal::MotorChannel(config::kMotorPins[4], config::kMotorDirPins[4]),
              hal::MotorChannel(config::kMotorPins[5], config::kMotorDirPins[5])},
      steerServos_{
          SteerServoImpl(config::kSteerPins[0], proto::kMinSteering, proto::kMaxSteering),
          SteerServoImpl(config::kSteerPins[1], proto::kMinSteering, proto::kMaxSteering),
          SteerServoImpl(config::kSteerPins[2], proto::kMinSteering, proto::kMaxSteering),
          SteerServoImpl(config::kSteerPins[3], proto::kMinSteering, proto::kMaxSteering),
          SteerServoImpl(config::kSteerPins[4], proto::kMinSteering, proto::kMaxSteering),
          SteerServoImpl(config::kSteerPins[5], proto::kMinSteering, proto::kMaxSteering),
      } {}

void BaseChassis::begin() {
  for (auto& m : motors_) {
    m.begin();
  }
  for (auto& s : steerServos_) {
    s.begin();
  }
}

void BaseChassis::setVelocities(const int16_t (&velocities)[proto::kNumDriveMotors]) {
  for (size_t i = 0; i < proto::kNumDriveMotors; ++i) {
    motors_[i].setVelocity(velocities[i]);
  }
}

void BaseChassis::setSteering(const int16_t (&angles)[proto::kNumSteeringServos]) {
  for (size_t i = 0; i < proto::kNumSteeringServos; ++i) {
    steerServos_[i].setAngleTenths(angles[i]);
  }
}

}  // namespace meister

#endif  // ARDUINO

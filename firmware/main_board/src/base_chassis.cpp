#ifdef ARDUINO

#include <Arduino.h>

#include "base_chassis.h"

#include "meister_config.h"

namespace meister {

BaseChassis::BaseChassis()
    : steerServos_{
          SteerServoImpl(config::kSteerPins[0], proto::kMinSteering, proto::kMaxSteering),
          SteerServoImpl(config::kSteerPins[1], proto::kMinSteering, proto::kMaxSteering),
          SteerServoImpl(config::kSteerPins[2], proto::kMinSteering, proto::kMaxSteering),
          SteerServoImpl(config::kSteerPins[3], proto::kMinSteering, proto::kMaxSteering),
          SteerServoImpl(config::kSteerPins[4], proto::kMinSteering, proto::kMaxSteering),
          SteerServoImpl(config::kSteerPins[5], proto::kMinSteering, proto::kMaxSteering),
      } {}

void BaseChassis::begin() {
  for (auto& s : steerServos_) {
    s.begin();
  }
}

void BaseChassis::setSteering(const int16_t (&angles)[proto::kNumSteeringServos]) {
  for (size_t i = 0; i < proto::kNumSteeringServos; ++i) {
    steerServos_[i].setAngleTenths(angles[i]);
  }
}

}  // namespace meister

#endif  // ARDUINO

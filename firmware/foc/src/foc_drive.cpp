#include "foc_drive.h"

#include "foc_config.h"

FocDrive::FocDrive(FocBoard& board)
    : board_(board),
      driver_(foc_cfg::kDriverPinA, foc_cfg::kDriverPinB, foc_cfg::kDriverPinC,
              foc_cfg::kDriverEnable),
      cs_(foc_cfg::kShuntOhms, foc_cfg::kCurrentSenseGain, foc_cfg::kCurrentSensePinA,
          foc_cfg::kCurrentSensePinB),
      sensor_(AS5600_I2C),
      motor_(foc_cfg::kPolePairs) {}

bool FocDrive::begin() {
  driver_.voltage_power_supply = foc_cfg::kSupplyVolts;
  driver_.init();
  motor_.linkDriver(&driver_);

  cs_.init();
  motor_.linkCurrentSense(&cs_);

  // Fast mode has to land before the first sensor read, otherwise init() caches the slow
  // digital filter for the whole session.
  if (!board_.setAs5600FastMode()) {
    Serial.println(F("#ERR AS5600 CONF write failed"));
  }
  sensor_.init(&board_.i2c());
  motor_.linkSensor(&sensor_);

  motor_.torque_controller = TorqueControlType::foc_current;
  motor_.controller = MotionControlType::torque;
  motor_.modulation_centered = 0;

  motor_.current_limit = foc_cfg::kCurrentLimitAmps;
  motor_.velocity_limit = foc_cfg::kVelocityLimitRps;
  motor_.voltage_limit = foc_cfg::kVoltageLimitVolts;
  motor_.voltage_sensor_align = foc_cfg::kAlignVoltageVolts;

  applyGains();

  if (board_.undervoltage()) {
    // The gate has to come before motor_.init(): alignment energises the phases, so a low
    // supply here would show up as a bogus alignment rather than as a refusal.
    Serial.print(F("#ERR undervoltage, drive gated. vin="));
    Serial.print(board_.vinVolts(), 2);
    Serial.println(F(" V"));
    return false;
  }

  // init() bakes the limits into the PIDs; initFOC() runs the alignment because
  // zero_electric_angle is still NOT_SET.
  motor_.init();
  const bool aligned = motor_.initFOC() != 0;
  motor_.disable();
  ready_ = aligned;
  reportZeroElec("#ZERO_ELEC");
  if (!aligned) Serial.println(F("#ERR alignment failed, drive gated"));
  return aligned;
}

void FocDrive::applyGains() {
  motor_.PID_current_q.P = foc_cfg::kPidCurrentP;
  motor_.PID_current_q.I = foc_cfg::kPidCurrentI;
  motor_.PID_current_d.P = foc_cfg::kPidCurrentP;
  motor_.PID_current_d.I = foc_cfg::kPidCurrentI;
  motor_.LPF_current_q.Tf = foc_cfg::kLpfCurrentTf;
  motor_.LPF_current_d.Tf = foc_cfg::kLpfCurrentTf;
  motor_.PID_velocity.P = foc_cfg::kPidVelP;
  motor_.PID_velocity.I = foc_cfg::kPidVelI;
  motor_.PID_velocity.D = foc_cfg::kPidVelD;
  motor_.P_angle.P = foc_cfg::kPidAngleP;
}

void FocDrive::reportZeroElec(const char* tag) {
  Serial.print(tag);
  Serial.print(' ');
  Serial.print(motor_.zero_electric_angle, 4);
  Serial.print(F(" rad"));
  if (motor_.zero_electric_angle >= foc_cfg::kZeroElecHealthyMin &&
      motor_.zero_electric_angle <= foc_cfg::kZeroElecHealthyMax) {
    Serial.print(F(" (in expected range)"));
  } else {
    Serial.print(F(" (OUTSIDE expected range, do not run)"));
  }
  Serial.println();
}

bool FocDrive::calibrate() {
  // Alignment inside initFOC() only runs when sensor_direction is unset, so both fields
  // have to be cleared or the command would silently reuse the previous result.
  motor_.zero_electric_angle = NOT_SET;
  motor_.sensor_direction = static_cast<int>(NOT_SET);
  motor_.enable();
  const bool ok = motor_.initFOC() != 0;
  motor_.disable();
  reportZeroElec(ok ? "#CAL_OK" : "#CAL_FAIL");
  return ok;
}

bool FocDrive::setAlignVoltage(float volts) {
  if (!ready_) return false;
  if (board_.undervoltage()) {
    Serial.println(F("#ERR undervoltage, alignment refused"));
    return false;
  }
  motor_.voltage_sensor_align = volts;
  motor_.enable();
  const bool ok = motor_.initFOC() != 0;
  motor_.disable();
  Serial.print(F("#ALIGN_V "));
  Serial.print(volts, 2);
  Serial.println(ok ? F(" V ok") : F(" V FAILED"));
  return ok;
}

bool FocDrive::setCurrentLimit(float amps) {
  if (!ready_) return false;
  // The pair has to be written together: a later init() re-bakes PID_velocity.limit from
  // current_limit, so setting current_limit alone leaves the velocity loop with the old
  // ceiling until the next init. The limit only bounds the torque mode.
  motor_.current_limit = amps;
  motor_.PID_velocity.limit = amps;
  if (torque_sp_ > amps) torque_sp_ = amps;
  if (torque_sp_ < -amps) torque_sp_ = -amps;
  Serial.print(F("#CUR_LIM "));
  Serial.println(amps, 3);
  return true;
}

bool FocDrive::commandTorque(float torque) {
  if (!ready_) return false;
  const float limit = motor_.current_limit;
  torque_sp_ = torque > limit ? limit : (torque < -limit ? -limit : torque);
  motor_.controller = MotionControlType::torque;
  motor_.enable();
  motor_.move(torque_sp_);
  return true;
}

bool FocDrive::commandVelocity(float vel_rps) {
  if (!ready_) return false;
  // W 0 is active braking, not a stop. Clearing the velocity integrator first is what the
  // article does to avoid the start-up jerk from a stale wind-up value.
  motor_.controller = MotionControlType::velocity;
  resetVelocityPid();
  motor_.enable();
  motor_.move(vel_rps);
  return true;
}

bool FocDrive::commandAngle(float rad) {
  if (!ready_) return false;
  motor_.controller = MotionControlType::angle;
  motor_.enable();
  motor_.move(rad);
  return true;
}

void FocDrive::stop() {
  motor_.disable();
  resetVelocityPid();
  torque_sp_ = 0.0f;
}

void FocDrive::poll() { motor_.move(); }

void FocDrive::resetVelocityPid() {
  const float ramp = motor_.PID_velocity.output_ramp;
  const float limit = motor_.PID_velocity.limit;
  motor_.PID_velocity = PIDController(foc_cfg::kPidVelP, foc_cfg::kPidVelI, foc_cfg::kPidVelD,
                                      ramp, limit);
}

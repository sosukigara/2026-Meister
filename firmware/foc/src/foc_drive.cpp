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
  // digital filter for the whole session. Its failure is also the sensor-absent signal:
  // no ACK means nothing to align against, so initFOC() is skipped rather than run blind.
  sensor_ok_ = board_.setAs5600FastMode();
  if (!sensor_ok_) {
    Serial.println(F("#ERR AS5600 CONF write failed"));
  }
  sensor_.init(&board_.i2c());
  if (sensor_ok_) motor_.linkSensor(&sensor_);

  motor_.torque_controller = TorqueControlType::foc_current;
  motor_.controller = MotionControlType::torque;
  motor_.modulation_centered = 0;

  motor_.current_limit = foc_cfg::kCurrentLimitAmps;
  // phase_resistance is left NOT_SET on purpose. Setting it makes the open-loop modes
  // derive Uq = current_limit * phase_resistance, which for this low-ohm motor is below
  // the LEDC resolution and produces no output at all (see foc_config.h).
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
  motor_inited_ = true;
  if (!sensor_ok_) {
    motor_.disable();
    Serial.println(F("#NOTE no sensor, closed loop gated. O/N open loop only."));
    return false;
  }
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
  user_current_limit_ = amps;
  if (target_ > amps) target_ = amps;
  if (target_ < -amps) target_ = -amps;
  Serial.print(F("#CUR_LIM "));
  Serial.println(amps, 3);
  return true;
}

bool FocDrive::commandTorque(float torque) {
  if (!ready_) return false;
  sine_on_ = false;  // TEMP
  motor_.current_limit = user_current_limit_;
  motor_.voltage_limit = foc_cfg::kVoltageLimitVolts;
  const float limit = motor_.current_limit;
  target_ = torque > limit ? limit : (torque < -limit ? -limit : torque);
  motor_.controller = MotionControlType::torque;
  motor_.enable();
  return true;
}

bool FocDrive::commandVelocity(float vel_rps) {
  if (!ready_) return false;
  // W 0 is active braking, not a stop. Clearing the velocity integrator first is what the
  // article does to avoid the start-up jerk from a stale wind-up value.
  sine_on_ = false;  // TEMP
  motor_.current_limit = user_current_limit_;
  motor_.voltage_limit = foc_cfg::kVoltageLimitVolts;
  motor_.controller = MotionControlType::velocity;
  resetVelocityPid();
  target_ = vel_rps;
  motor_.enable();
  return true;
}

bool FocDrive::commandAngle(float rad) {
  if (!ready_) return false;
  sine_on_ = false;  // TEMP
  motor_.current_limit = user_current_limit_;
  motor_.voltage_limit = foc_cfg::kVoltageLimitVolts;
  motor_.controller = MotionControlType::angle;
  target_ = rad;
  motor_.enable();
  return true;
}

bool FocDrive::commandOpenLoopVelocity(float vel_rps) {
  if (!motor_inited_) {
    Serial.println(F("#ERR open loop needs motor init (24V?)"));
    return false;
  }
  if (millis() < openloop_cool_until_) {
    Serial.println(F("#BUSY cooling, wait"));
    return false;
  }
  sine_on_ = false;  // TEMP
  motor_.controller = MotionControlType::velocity_openloop;
  motor_.current_limit = user_current_limit_ < foc_cfg::kOpenLoopCurrentAmps
                             ? user_current_limit_
                             : foc_cfg::kOpenLoopCurrentAmps;
  motor_.voltage_limit = foc_cfg::kOpenLoopVoltageVolts;
  // SimpleFOC open-loop targets are rad/s. The command and the UI are in rev/s, so
  // convert here; without this "O 2" would ask for 2 rad/s (0.32 rev/s) and look stuck.
  target_ = vel_rps * _2PI;
  openloop_t0ms_ = millis();
  motor_.enable();
  return true;
}

bool FocDrive::commandSine(float amp_rps) {
  if (!motor_inited_) {
    Serial.println(F("#ERR open loop needs motor init (24V?)"));
    return false;
  }
  if (millis() < openloop_cool_until_) {
    Serial.println(F("#BUSY cooling, wait"));
    return false;
  }
  motor_.controller = MotionControlType::velocity_openloop;
  motor_.current_limit = user_current_limit_ < foc_cfg::kOpenLoopCurrentAmps
                             ? user_current_limit_
                             : foc_cfg::kOpenLoopCurrentAmps;
  motor_.voltage_limit = foc_cfg::kOpenLoopVoltageVolts;
  sine_amp_ = (amp_rps >= 0.0f ? amp_rps : -amp_rps) * _2PI;  // rev/s -> rad/s
  sine_t0ms_ = millis();
  sine_on_ = true;
  openloop_t0ms_ = sine_t0ms_;
  motor_.enable();
  return true;
}

bool FocDrive::commandAngleOpenLoop(float rad) {
  if (!motor_inited_) {
    Serial.println(F("#ERR open loop needs motor init (24V?)"));
    return false;
  }
  if (millis() < openloop_cool_until_) {
    Serial.println(F("#BUSY cooling, wait"));
    return false;
  }
  sine_on_ = false;  // TEMP
  motor_.controller = MotionControlType::angle_openloop;
  motor_.current_limit = user_current_limit_ < foc_cfg::kOpenLoopCurrentAmps
                             ? user_current_limit_
                             : foc_cfg::kOpenLoopCurrentAmps;
  motor_.voltage_limit = foc_cfg::kOpenLoopVoltageVolts;
  target_ = rad;
  openloop_t0ms_ = millis();
  motor_.enable();
  return true;
}

bool FocDrive::diagnose() {
  if (!motor_inited_) return false;
  if (board_.undervoltage()) {
    Serial.println(F("#ERR undervoltage, diagnosis refused"));
    return false;
  }
  sine_on_ = false;  // TEMP
  motor_.controller = MotionControlType::velocity_openloop;
  motor_.current_limit = user_current_limit_ < foc_cfg::kOpenLoopCurrentAmps
                             ? user_current_limit_
                             : foc_cfg::kOpenLoopCurrentAmps;
  motor_.voltage_limit = foc_cfg::kOpenLoopVoltageVolts;
  motor_.enable();
  motor_.move(0.0f);
  float ia = 0.0f, ib = 0.0f;
  constexpr int kSamples = 100;
  for (int i = 0; i < kSamples; ++i) {
    const PhaseCurrent_s c = cs_.getPhaseCurrents();
    ia += c.a;
    ib += c.b;
    delay(2);
  }
  motor_.disable();
  ia /= kSamples;
  ib /= kSamples;
  const float ma = ia >= 0.0f ? ia : -ia;
  const float mb = ib >= 0.0f ? ib : -ib;
  Serial.print(F("#DIAG ia_a="));
  Serial.print(ia, 3);
  Serial.print(F(" ib_a="));
  Serial.print(ib, 3);
  if (ma < 0.02f && mb < 0.02f) {
    Serial.println(F(" OPEN (winding open or driver dead)"));
  } else {
    Serial.println(F(" CURRENT FLOWS (winding alive, voltage too weak)"));
  }
  return true;
}

void FocDrive::stop() {
  motor_.disable();
  resetVelocityPid();
  sine_on_ = false;  // TEMP
  openloop_cool_until_ = millis() + foc_cfg::kOpenLoopCooldownMs;
  target_ = 0.0f;
  ol_over_ = false;
  ol_imag_ = 0.0f;
  motor_.voltage_limit = foc_cfg::kVoltageLimitVolts;
}

float FocDrive::readPhaseCurrentMax() {
  const PhaseCurrent_s c = cs_.getPhaseCurrents();
  const float ia = c.a >= 0.0f ? c.a : -c.a;
  const float ib = c.b >= 0.0f ? c.b : -c.b;
  const float icv = c.a + c.b;  // ic is not wired; ia+ib+ic=0 recovers it
  const float ic = icv >= 0.0f ? icv : -icv;
  const float m = ia > ib ? ia : ib;
  return ic > m ? ic : m;
}

void FocDrive::poll() {
  if (motor_.controller == MotionControlType::velocity_openloop ||
      motor_.controller == MotionControlType::angle_openloop) {  // TEMP G
    if (motor_.enabled &&
        millis() - openloop_t0ms_ > foc_cfg::kOpenLoopTimeoutMs) {
      stop();
      Serial.println(F("#TIMEOUT open loop stopped"));
      return;
    }
    if (sine_on_) {  // TEMP
      const float t = (millis() - sine_t0ms_) * 0.001f;
      target_ = sine_amp_ * sinf(6.2831853f * foc_cfg::kSineFreqHz * t);
    }
    if (motor_.enabled) {
      // Software over-current limit. SimpleFOC open loop applies a fixed voltage with no
      // current feedback, so the shunt is the only place to catch a stall. Chop the
      // voltage while the current is high, restore it with hysteresis once it drops, and
      // trip if chopping cannot recover within kOpenLoopOcTripMs.
      ol_imag_ = readPhaseCurrentMax();
      if (ol_imag_ > foc_cfg::kOpenLoopCurrentAmps) {
        motor_.voltage_limit = 0.0f;
        if (!ol_over_) {
          ol_over_ = true;
          ol_over_since_ms_ = millis();
        } else if (millis() - ol_over_since_ms_ > foc_cfg::kOpenLoopOcTripMs) {
          const float seen = ol_imag_;
          stop();
          Serial.print(F("#OC overcurrent trip, I="));
          Serial.print(seen, 2);
          Serial.println(F(" A"));
          return;
        }
      } else if (ol_imag_ < foc_cfg::kOpenLoopCurrentAmps * 0.7f) {
        motor_.voltage_limit = foc_cfg::kOpenLoopVoltageVolts;
        ol_over_ = false;
      }
      motor_.move(target_);
    }
    return;
  }
  motor_.loopFOC();
  if (motor_.enabled) motor_.move(target_);
}

void FocDrive::resetVelocityPid() {
  const float ramp = motor_.PID_velocity.output_ramp;
  const float limit = motor_.PID_velocity.limit;
  motor_.PID_velocity = PIDController(foc_cfg::kPidVelP, foc_cfg::kPidVelI, foc_cfg::kPidVelD,
                                      ramp, limit);
}

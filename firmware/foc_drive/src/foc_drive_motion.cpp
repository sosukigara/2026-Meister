#include "foc_drive.h"

#include "foc_config.h"

// Motion commands. The per-loop safety state machine lives in foc_drive_safety.cpp so both
// files stay under the repo's 300-line limit.

bool FocDrive::motionAllowed() {
  if (fault_) {
    printPrefix();
    Serial.println(F("ERR fault latched, S to clear"));
    return false;
  }
  if (!motor_inited_) {
    printPrefix();
    Serial.println(F("ERR motor not initialized (12V?)"));
    return false;
  }
  return true;
}

bool FocDrive::prepareClosedLoop(MotionControlType type) {
  if (!ready_ || !sensor_ok_) return false;
  const bool starting = !motor_.enabled || motor_.controller != type;
  if (!starting) return true;
  if (motor_.enabled && openLoopActive()) {
    printPrefix();
    Serial.println(F("ERR stop open loop first"));
    return false;
  }
  // The runtime health check only runs while enabled. Probe before raising the gate so a
  // sensor that died while the drive was idle cannot be used for the first FOC cycles.
  if (!board_.as5600Alive()) {
    ready_ = false;
    printPrefix();
    Serial.println(F("ERR AS5600 not responding, recalibrate with C"));
    // A mode switch can reach this while the old controller is still enabled. Do not keep
    // driving on the sensor's constant 0xFFF angle with the guards now bypassed.
    if (motor_.enabled) emergencyStop();
    return false;
  }
  motor_.controller = type;
  motor_.current_limit = user_current_limit_;
  motor_.voltage_limit = foc_cfg::kVoltageLimitVolts;
  // Only the transition resets the integrator. Resetting on every deadman resend would
  // turn the velocity PI into a P controller and pulse the PWM at the command rate.
  if (type == MotionControlType::velocity) resetVelocityPid();
  motor_.enable();
  return true;
}

bool FocDrive::commandTorque(float torque) {
  if (!motionAllowed()) return false;
  if (!ready_) return false;
  if (!prepareClosedLoop(MotionControlType::torque)) return false;
  last_motion_cmd_ms_ = millis();
  target_ = applyInvert(foc::clampSpeedRps(torque, user_current_limit_));
  return true;
}

bool FocDrive::commandVelocity(float vel_rps) {
  if (!motionAllowed()) return false;
  if (!ready_) return false;
  if (!prepareClosedLoop(MotionControlType::velocity)) return false;
  last_motion_cmd_ms_ = millis();
  target_ = applyInvert(foc::clampSpeedRps(vel_rps, foc::kClosedLoopMaxRps)) * _2PI;
  return true;
}

bool FocDrive::commandAngle(float rad) {
  if (!motionAllowed()) return false;
  if (!ready_) return false;
  if (!prepareClosedLoop(MotionControlType::angle)) return false;
  last_motion_cmd_ms_ = millis();
  target_ = applyInvert(foc::clampAngleRad(rad));
  return true;
}

bool FocDrive::startOpenLoop(float target_rad_s) {
  if (motor_.enabled && !openLoopActive()) {
    printPrefix();
    Serial.println(F("ERR stop the closed loop first"));
    return false;
  }
  if (foc::cooldownActive(millis(), openloop_cool_until_)) {
    printPrefix();
    Serial.println(F("BUSY cooling, wait"));
    return false;
  }
  if (!pole_pairs_confirmed_) {
    printPrefix();
    Serial.println(F("WARN pole pairs unconfirmed; speed will be wrong until P is set"));
  }
  motor_.controller = MotionControlType::velocity_openloop;
  motor_.current_limit = user_current_limit_;
  motor_.voltage_limit = openloop_voltage_;
  target_ = target_rad_s;
  openloop_t0ms_ = millis();
  motor_.enable();
  return true;
}

bool FocDrive::commandOpenLoopVelocity(float vel_rps) {
  if (!motionAllowed()) return false;
  const float target =
      applyInvert(foc::clampSpeedRps(vel_rps, foc::kOpenLoopMaxRps)) * _2PI;
  // A resend while the session is running only moves the target. It must not restart the
  // 10 s timeout, re-enable the driver, or reset the session.
  if (openLoopActive() && motor_.enabled) {
    if (motor_.controller != MotionControlType::velocity_openloop) {
      printPrefix();
      Serial.println(F("ERR stop the current open loop first"));
      return false;
    }
    target_ = target;
    return true;
  }
  return startOpenLoop(target);
}

bool FocDrive::diagnose() {
  if (!motionAllowed()) return false;
  if (motor_.enabled) {
    printPrefix();
    Serial.println(F("ERR diagnosis needs a stopped channel"));
    return false;
  }
  if (foc::cooldownActive(millis(), openloop_cool_until_)) {
    printPrefix();
    Serial.println(F("BUSY cooling, wait"));
    return false;
  }
  if (board_.undervoltage()) {
    printPrefix();
    Serial.println(F("ERR undervoltage, diagnosis refused"));
    return false;
  }

  motor_.controller = MotionControlType::velocity_openloop;
  motor_.current_limit = user_current_limit_;
  motor_.voltage_limit = openloop_voltage_;
  motor_.enable();
  motor_.move(0.0f);
  float ia = 0.0f, ib = 0.0f;
  constexpr int kSamples = 100;
  for (int i = 0; i < kSamples; ++i) {
    const PhaseCurrent_s c = cs_.getPhaseCurrents();
    const float ca = fabsf(c.a);
    const float cb = fabsf(c.b);
    const float cc = fabsf(c.a + c.b);
    float peak = ca > cb ? ca : cb;
    if (cc > peak) peak = cc;
    // This loop blocks poll(), so the normal 4 A / full-scale trips cannot run. Check
    // every sample and abort a shorted winding immediately.
    if (peak >= foc_cfg::kOpenLoopOcTripAmps ||
        peak >= foc_cfg::kCurrentSenseFullScaleAmps * 0.98f) {
      emergencyStop();
      printPrefix();
      Serial.print(F("DIAG OC aborted, I="));
      Serial.print(peak, 2);
      Serial.println(F(" A"));
      return false;
    }
    ia += c.a;
    ib += c.b;
    delay(2);
  }
  motor_.disable();
  ia /= kSamples;
  ib /= kSamples;
  const float ma = ia >= 0.0f ? ia : -ia;
  const float mb = ib >= 0.0f ? ib : -ib;
  const float measured = ma > mb ? ma : mb;
  const float expected = openloop_voltage_ / foc_cfg::kPhaseResistanceOhms;
  const float ratio = expected > 0.0f ? measured / expected : 0.0f;
  openloop_cool_until_ = millis() + foc_cfg::kDiagnoseCooldownMs;

  printPrefix();
  Serial.print(F("DIAG ia_a="));
  Serial.print(ia, 3);
  Serial.print(F(" ib_a="));
  Serial.print(ib, 3);
  Serial.print(F(" expected_a="));
  Serial.print(expected, 2);
  Serial.print(F(" ratio="));
  Serial.print(ratio, 2);
  if (ratio < 0.20f || ratio > 5.0f) {
    Serial.println(F(" CS_SUSPECT (check R010 marking / gain / wiring)"));
  } else {
    Serial.println(F(" CS_OK"));
  }
  return true;
}

void FocDrive::resetVelocityPid() {
  const float ramp = motor_.PID_velocity.output_ramp;
  const float limit = motor_.PID_velocity.limit;
  motor_.PID_velocity = PIDController(foc_cfg::kPidVelP, foc_cfg::kPidVelI, foc_cfg::kPidVelD,
                                      ramp, limit);
}

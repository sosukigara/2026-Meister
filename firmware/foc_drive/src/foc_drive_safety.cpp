#include "foc_drive.h"

#include "foc_config.h"

// Runtime safety: stop/fault handling, overcurrent, current-sense plausibility, supply and
// sensor health, and the per-loop state machine. Kept apart from the motion commands so
// each file has one job.

void FocDrive::stop() {
  // Only an actually-driven open loop earns the cooldown. diagnose() and the session
  // timeout leave the controller at velocity_openloop while disabled; re-arming 10 s from
  // a later S/C/Z/P/D would overwrite D's 2 s cooldown and block the next D for 10 s.
  const bool was_openloop = motor_.enabled && openLoopActive();
  motor_.disable();
  resetVelocityPid();
  // The cooldown exists to bound open-loop duty. A closed-loop stop should not block the
  // next open-loop test for 10 s.
  if (was_openloop) openloop_cool_until_ = millis() + foc_cfg::kOpenLoopCooldownMs;
  target_ = 0.0f;
  current_over_ = false;
  cs_suspect_ = false;
  phase_current_max_ = 0.0f;
  motor_.voltage_limit = foc_cfg::kVoltageLimitVolts;
}

void FocDrive::emergencyStop() {
  stop();
  fault_ = true;
}

void FocDrive::clearFault() {
  fault_ = false;
  sensor_fail_count_ = 0;
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

bool FocDrive::checkOverCurrent(uint32_t now, float trip_amps, uint32_t trip_ms) {
  if (phase_current_max_ >= foc_cfg::kCurrentSenseFullScaleAmps * 0.98f) {
    const float seen = phase_current_max_;
    emergencyStop();
    printPrefix();
    Serial.print(F("OC_SAT current sense at full scale, I="));
    Serial.print(seen, 2);
    Serial.println(F(" A"));
    return false;
  }
  if (phase_current_max_ > trip_amps) {
    if (!current_over_) {
      current_over_ = true;
      current_over_since_ms_ = now;
    } else if (foc::timeoutElapsed(now, current_over_since_ms_, trip_ms)) {
      const float seen = phase_current_max_;
      emergencyStop();
      printPrefix();
      Serial.print(F("OC overcurrent trip, I="));
      Serial.print(seen, 2);
      Serial.println(F(" A"));
      return false;
    }
  } else {
    current_over_ = false;
  }
  return true;
}

bool FocDrive::checkCurrentSenseFeedback(uint32_t now) {
  if (!foc_cfg::kCurrentSenseCheckEnabled || !ready_) return true;
  const float speed = fabsf(motor_.shaft_velocity);
  const float vq = fabsf(motor_.voltage.q);
  const float iq = fabsf(motor_.current.q);
  const float expected = vq / foc_cfg::kPhaseResistanceOhms;
  const bool low_speed_suspect =
      speed < foc_cfg::kCsCheckMaxSpeedRad &&
      vq >= foc_cfg::kCsCheckMinVoltageVolts &&
      iq < expected * foc_cfg::kCsCheckMinRatio;
  const bool saturated_suspect = vq > foc_cfg::kCsLossVoltageVolts &&
                                 iq < foc_cfg::kCsLossCurrentAmps;
  if (!low_speed_suspect && !saturated_suspect) {
    cs_suspect_ = false;
    return true;
  }
  if (!cs_suspect_) {
    cs_suspect_ = true;
    cs_suspect_since_ms_ = now;
    return true;
  }
  if (foc::timeoutElapsed(now, cs_suspect_since_ms_, foc_cfg::kCsCheckMs)) {
    emergencyStop();
    printPrefix();
    Serial.print(F("CS_LOSS measured I="));
    Serial.print(iq, 2);
    Serial.print(F(" A vs expected "));
    Serial.print(expected, 2);
    Serial.println(F(" A (shunt/gain/wiring?)"));
    return false;
  }
  return true;
}

bool FocDrive::checkHealth(uint32_t now) {
  if (!motor_.enabled) return true;
  if (!foc::timeoutElapsed(now, last_health_ms_, foc_cfg::kHealthPeriodMs)) return true;
  last_health_ms_ = now;

  const float vin = board_.vinVolts();
  if (vin < foc_cfg::kRuntimeUndervoltageVolts) {
    if (!vin_low_) {
      vin_low_ = true;
      vin_low_since_ms_ = now;
    } else if (foc::timeoutElapsed(now, vin_low_since_ms_,
                                   foc_cfg::kRuntimeUndervoltageMs)) {
      emergencyStop();
      printPrefix();
      Serial.print(F("VIN_LOW stopped, vin="));
      Serial.print(vin, 2);
      Serial.println(F(" V"));
      return false;
    }
  } else {
    vin_low_ = false;
  }

  if (ready_ && sensor_ok_) {
    if (!board_.as5600Alive()) {
      if (++sensor_fail_count_ >= 3) {
        ready_ = false;
        emergencyStop();
        printPrefix();
        Serial.println(F("SENSOR_LOST AS5600 NACK, recalibrate with C"));
        return false;
      }
    } else {
      sensor_fail_count_ = 0;
    }
  }
  return true;
}

void FocDrive::poll() {
  if (fault_) {
    if (motor_.enabled) motor_.disable();
    return;
  }

  const uint32_t now = millis();
  if (motor_.enabled && !checkHealth(now)) return;

  if (openLoopActive()) {
    if (motor_.enabled &&
        foc::timeoutElapsed(now, openloop_t0ms_, foc_cfg::kOpenLoopTimeoutMs)) {
      stop();
      printPrefix();
      Serial.println(F("TIMEOUT open loop stopped"));
      return;
    }
    if (motor_.enabled) {
      // No active current loop: the winding itself bounds the current at the configured
      // open-loop voltage. The trip is a backstop for a phase short or a bad shunt.
      phase_current_max_ = readPhaseCurrentMax();
      if (!checkOverCurrent(now, foc_cfg::kOpenLoopOcTripAmps,
                            foc_cfg::kOpenLoopOcTripMs)) {
        return;
      }
      motor_.move(target_);
    }
    return;
  }

  // Closed-loop deadman: W/T/A are not one-shot. The PC has to keep sending them.
  if (motor_.enabled &&
      foc::timeoutElapsed(now, last_motion_cmd_ms_, foc_cfg::kCommandTimeoutMs)) {
    stop();
    printPrefix();
    Serial.println(F("CMD_TIMEOUT stopped"));
    return;
  }
  motor_.loopFOC();
  if (motor_.enabled) {
    phase_current_max_ = readPhaseCurrentMax();
    if (!checkOverCurrent(now, foc_cfg::kClosedLoopOcTripAmps,
                          foc_cfg::kClosedLoopOcTripMs)) {
      return;
    }
    if (!checkCurrentSenseFeedback(now)) return;
    motor_.move(target_);
  }
}

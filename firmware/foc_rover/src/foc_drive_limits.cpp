#include "foc_drive.h"

#include "foc_config.h"

// Limit/configuration commands. Every value is validated against foc_limits.h; the drive
// never trusts the console to have done it.

bool FocDrive::setCurrentLimit(float amps) {
  if (!foc::validCurrentLimit(amps)) {
    printPrefix();
    Serial.print(F("ERR L must be >0 and <="));
    Serial.print(foc::kCurrentLimitMax, 1);
    Serial.println(F(" A"));
    return false;
  }
  if (!ready_) return false;
  // The pair has to be written together: a later init() re-bakes PID_velocity.limit from
  // current_limit, so setting current_limit alone leaves the velocity loop with the old
  // ceiling until the next init. The limit only bounds the torque/velocity modes.
  motor_.current_limit = amps;
  motor_.PID_velocity.limit = amps;
  user_current_limit_ = amps;
  // Only the torque target is in amps. Velocity/angle targets are rad/s and rad; clamping
  // them to an amp value would silently rewrite the command.
  if (motor_.controller == MotionControlType::torque) {
    if (target_ > amps) target_ = amps;
    if (target_ < -amps) target_ = -amps;
  }
  printPrefix();
  Serial.print(F("CUR_LIM "));
  Serial.println(amps, 3);
  return true;
}

bool FocDrive::setPolePairs(int pairs) {
  if (!foc::validPolePairs(pairs)) {
    printPrefix();
    Serial.print(F("ERR pole pairs "));
    Serial.print(foc::kMinPolePairs);
    Serial.print("-");
    Serial.println(foc::kMaxPolePairs);
    return false;
  }
  if (motor_.enabled) stop();
  motor_.pole_pairs = pairs;
  pole_pairs_confirmed_ = true;
  ready_ = false;
  printPrefix();
  Serial.print(F("POLE_PAIRS "));
  Serial.print(pairs);
  Serial.println(F(" (run C for closed loop)"));
  return true;
}

bool FocDrive::setOpenLoopVoltage(float volts) {
  if (!std::isfinite(volts) || volts <= 0.0f) {
    printPrefix();
    Serial.println(F("ERR V needs a positive voltage"));
    return false;
  }
  const float clamped = foc::clampOpenLoopVoltage(volts);
  openloop_voltage_ = clamped;
  printPrefix();
  Serial.print(F("OPEN_LOOP "));
  Serial.print(clamped, 2);
  Serial.println(F(" V"));
  if (clamped != volts) {
    printPrefix();
    Serial.print(F("WARN V clamped to "));
    Serial.print(foc::kOpenLoopVoltageMin, 2);
    Serial.print(F("-"));
    Serial.print(foc::kOpenLoopVoltageMax, 2);
    Serial.println(F(" V"));
  }
  if (clamped > foc::kOpenLoopVoltageWarn) {
    printPrefix();
    Serial.println(F("WARN >0.45 V is a short-burst bench value on LA8308"));
  }
  return true;
}

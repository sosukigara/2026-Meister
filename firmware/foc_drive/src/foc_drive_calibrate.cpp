#include "foc_drive.h"

#include "foc_config.h"

// Calibration commands. Kept separate from bring-up and motion: alignment blocks for
// seconds and must never run while another channel is energised.

void FocDrive::reportZeroElec(const char* tag) {
  printPrefix();
  Serial.print(tag);
  Serial.print(' ');
  Serial.print(motor_.zero_electric_angle, 4);
  Serial.println(F(" rad (LA8308 reference value unknown; verify PP and direction)"));
}

bool FocDrive::alignNow(float volts, const char* ok_tag) {
  // Alignment inside initFOC() only runs when the fields are unset, so both have to be
  // cleared or the command would silently reuse the previous result.
  motor_.voltage_sensor_align = volts;
  motor_.zero_electric_angle = NOT_SET;
  motor_.sensor_direction = static_cast<int>(NOT_SET);
  motor_.enable();
  const bool ok = motor_.initFOC() != 0;
  motor_.disable();
  sensor_fail_count_ = 0;
  // initFOC() returns success even when SimpleFOC's PP check printed a mismatch. The
  // operator confirms the pole pairs; the firmware only knows that P was explicitly set.
  ready_ = ok && pole_pairs_confirmed_;
  reportZeroElec(ok ? ok_tag : "ALIGN_FAIL");
  if (!ok) {
    printPrefix();
    Serial.println(F("ERR alignment failed, closed loop gated"));
  }
  return ready_;
}

bool FocDrive::calibrate() {
  if (!motionAllowed()) return false;
  if (!sensor_ok_) {
    printPrefix();
    Serial.println(F("ERR calibrate needs AS5600"));
    return false;
  }
  if (!pole_pairs_confirmed_) {
    printPrefix();
    Serial.println(F("ERR set P <pairs> before C (LA8308 value unconfirmed)"));
    return false;
  }
  if (motor_.enabled) {
    printPrefix();
    Serial.println(F("ERR calibrate needs a stopped channel"));
    return false;
  }
  if (board_.undervoltage()) {
    printPrefix();
    Serial.println(F("ERR undervoltage, calibrate refused"));
    return false;
  }
  return alignNow(foc_cfg::kAlignVoltageVolts, "CAL_OK");
}

bool FocDrive::setAlignVoltage(float volts) {
  if (!foc::validAlignVoltage(volts)) {
    printPrefix();
    Serial.print(F("ERR Z "));
    Serial.print(foc::kAlignVoltageMin, 2);
    Serial.print(F("-"));
    Serial.print(foc::kAlignVoltageMax, 2);
    Serial.println(F(" V"));
    return false;
  }
  if (!motionAllowed()) return false;
  if (!sensor_ok_) {
    printPrefix();
    Serial.println(F("ERR alignment needs AS5600"));
    return false;
  }
  if (!pole_pairs_confirmed_) {
    printPrefix();
    Serial.println(F("ERR set P <pairs> before Z"));
    return false;
  }
  if (motor_.enabled) {
    printPrefix();
    Serial.println(F("ERR alignment needs a stopped channel"));
    return false;
  }
  if (board_.undervoltage()) {
    printPrefix();
    Serial.println(F("ERR undervoltage, alignment refused"));
    return false;
  }
  const bool ok = alignNow(volts, "ALIGN_OK");
  printPrefix();
  Serial.print(F("ALIGN_V "));
  Serial.print(volts, 2);
  Serial.println(ok ? F(" V ok") : F(" V FAILED"));
  return ok;
}

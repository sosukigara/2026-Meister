#include "foc_drive.h"

#include "foc_config.h"

// Bring-up only: driver/current-sense/sensor construction, the shared-gate-safe init order
// and the first PID limit bake. Motion, calibration and runtime safety live in their own
// translation units.

FocDrive::FocDrive(FocBoard& board, const foc_cfg::MotorPins& pins, const char* name,
                   bool invert, Print& monitor)
    : board_(board),
      pins_(pins),
      name_(name),
      invert_(invert),
      monitor_(monitor),
      driver_(pins.pwm_a, pins.pwm_b, pins.pwm_c, pins.enable),
      cs_(foc_cfg::kShuntOhms, foc_cfg::kCurrentSenseGain, pins.cs_a, pins.cs_b),
      sensor_(AS5600_I2C),
      motor_(foc_cfg::kConfirmedPolePairs > 0 ? foc_cfg::kConfirmedPolePairs
                                              : foc_cfg::kDefaultPolePairs) {
  pole_pairs_confirmed_ = foc_cfg::kConfirmedPolePairs > 0;
}

void FocDrive::printPrefix() {
  Serial.print('#');
  Serial.print(name_);
  Serial.print(' ');
}

bool FocDrive::beginDrivers() {
  // This half must run for BOTH channels before either beginMotor(): motor_.init()
  // enables the shared GPIO12 gate, and the other channel's PWM pins must already be
  // configured and at zero before that happens.
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
    printPrefix();
    Serial.println(F("ERR AS5600 CONF write failed"));
  }
  sensor_.init(&board_.i2c());
  if (sensor_ok_) motor_.linkSensor(&sensor_);

  motor_.torque_controller = TorqueControlType::foc_current;
  motor_.controller = MotionControlType::torque;
  motor_.modulation_centered = 0;

  motor_.current_limit = user_current_limit_;
  motor_.velocity_limit = foc_cfg::kVelocityLimitRps;
  motor_.voltage_limit = foc_cfg::kVoltageLimitVolts;
  motor_.voltage_sensor_align = foc_cfg::kAlignVoltageVolts;
  motor_.monitor_port = &monitor_;

  applyGains();

  printPrefix();
  Serial.print(F("SHUNT "));
  Serial.print(foc_cfg::kShuntOhms, 3);
  Serial.println(F(" ohm (verify R010; 1 mohm makes every reading 10x low)"));

  printPrefix();
  Serial.print(F("POLE_PAIRS "));
  Serial.print(motor_.pole_pairs);
  Serial.println(pole_pairs_confirmed_
                     ? F(" (confirmed)")
                     : F(" UNCONFIRMED - set P before closed loop (magnets / 2)"));

  printPrefix();
  Serial.print(F("OPEN_LOOP "));
  Serial.print(openloop_voltage_, 2);
  Serial.println(F(" V"));

  printPrefix();
  Serial.print(F("ALIGN "));
  Serial.print(motor_.voltage_sensor_align, 2);
  Serial.println(F(" V"));

  return sensor_ok_;
}

bool FocDrive::beginMotor() {
  if (board_.undervoltage()) {
    // The gate has to come before motor_.init(): alignment energises the phases, so a low
    // supply here would show up as a bogus alignment rather than as a refusal.
    printPrefix();
    Serial.print(F("ERR undervoltage, drive gated. vin="));
    Serial.print(board_.vinVolts(), 2);
    Serial.println(F(" V"));
    return false;
  }

  // init() bakes the limits into the PIDs and enables the driver. Both channels' PWM are
  // already configured because main.cpp called beginDrivers() on both first.
  motor_.init();
  motor_inited_ = true;

  if (!sensor_ok_) {
    motor_.disable();
    printPrefix();
    Serial.println(F("NOTE no sensor, closed loop gated. O open loop only."));
    return false;
  }

  if (!foc_cfg::kAutoAlignOnBoot) {
    // Alignment moves the wheels. The rover must be explicitly asked for it.
    motor_.disable();
    printPrefix();
    Serial.println(F("NOTE boot alignment off, run P (pairs) then C"));
    return false;
  }

  if (!pole_pairs_confirmed_) {
    motor_.disable();
    printPrefix();
    Serial.println(F("ERR pole pairs unconfirmed, alignment refused. Set P first."));
    return false;
  }

  const bool aligned = motor_.initFOC() != 0;
  motor_.disable();
  ready_ = aligned && pole_pairs_confirmed_;
  reportZeroElec(aligned ? "ZERO_ELEC" : "ALIGN_FAIL");
  if (!ready_) {
    printPrefix();
    Serial.println(F("ERR alignment failed, closed loop gated"));
  }
  return ready_;
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

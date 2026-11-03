#ifndef FOC_DRIVE_H
#define FOC_DRIVE_H

// Motor layer: owns one channel's driver, current sense, angle sensor and BLDCMotor, plus
// every command that can make that wheel move.

#include <Arduino.h>
#include <SimpleFOC.h>

#include "foc_board.h"
#include "foc_config.h"

class FocDrive {
 public:
  FocDrive(FocBoard& board, const foc_cfg::MotorPins& pins, const char* name, bool invert,
           Print& monitor);

  // Two-phase bring-up. Both channels' drivers must be initialized before either motor
  // because M0/M1 share ENABLE: initializing a driver does not raise the gate, but
  // motor_.init() does. See main.cpp.
  bool beginDrivers();
  bool beginMotor();

  const char* name() const { return name_; }
  bool ready() const { return ready_; }
  bool faulted() const { return fault_; }
  bool polePairsConfirmed() const { return pole_pairs_confirmed_; }

  bool calibrate();
  bool setAlignVoltage(float volts);
  bool setCurrentLimit(float amps);
  bool setPolePairs(int pairs);
  bool setOpenLoopVoltage(float volts);

  bool commandTorque(float torque);
  bool commandVelocity(float vel_rps);
  bool commandAngle(float rad);
  bool commandOpenLoopVelocity(float vel_rps);

  // TEMP D: energise the phases at the open-loop cap and report the shunt
  // currents. Tells an open winding apart from a too-weak voltage and checks the
  // current-sense gain against the configured winding resistance.
  bool diagnose();

  // User stop: disables the channel and applies the open-loop cooldown. Does NOT clear a
  // latched fault; only clearFault()/S does that.
  void stop();
  // Fault stop: stop() plus a latch that refuses every motion command until S.
  void emergencyStop();
  void clearFault();

  // Call once per loop. Keeps the sensor and shaft_velocity fresh even while disabled.
  void poll();

  bool enabled() const { return motor_.enabled != 0; }
  bool openLoopActive() const {
    return motor_.controller == MotionControlType::velocity_openloop ||
           motor_.controller == MotionControlType::angle_openloop;
  }
  uint8_t modeCode() const { return static_cast<uint8_t>(motor_.controller); }
  // Sensorless fallback: open-loop shaft_angle is simulated, so the spin check still
  // shows motion without touching the dead I2C bus.
  float angleForTelemetry() { return sensor_ok_ ? sensor_.getAngle() : motor_.shaft_angle; }
  float shaftVelocityRps() const { return motor_.shaft_velocity / _2PI; }
  // Peak measured phase current of the last poll cycle, for telemetry and #IOL.
  float phaseCurrentMax() const { return phase_current_max_; }
  float currentQ() const { return motor_.current.q; }
  float currentD() const { return motor_.current.d; }
  float voltageQ() const { return motor_.voltage.q; }
  float voltageD() const { return motor_.voltage.d; }

  // SimpleFOC 2.2.1 has no resetVelPid() and PIDController keeps its integrator state in
  // protected members, so the only way to drop a wind-up integral is to build a fresh
  // controller from the configured gains.
  void resetVelocityPid();

 private:
  bool motionAllowed();
  bool prepareClosedLoop(MotionControlType type);
  bool startOpenLoop(float target_rad_s);
  bool checkHealth(uint32_t now);
  bool checkOverCurrent(uint32_t now, float trip_amps, uint32_t trip_ms);
  bool checkCurrentSenseFeedback(uint32_t now);

  void applyGains();
  void printPrefix();
  void reportZeroElec(const char* tag);
  bool alignNow(float volts, const char* ok_tag);
  float applyInvert(float v) const { return invert_ ? -v : v; }
  // max(|ia|, |ib|, |ia+ib|) from the shunt. ic is not wired on this board, so it is
  // recovered from ia+ib+ic=0.
  float readPhaseCurrentMax();

  FocBoard& board_;
  foc_cfg::MotorPins pins_;
  const char* name_;
  bool invert_;
  Print& monitor_;
  BLDCDriver3PWM driver_;
  InlineCurrentSense cs_;
  MagneticSensorI2C sensor_;
  BLDCMotor motor_;

  bool ready_ = false;
  bool sensor_ok_ = false;
  bool motor_inited_ = false;
  bool fault_ = false;
  bool pole_pairs_confirmed_ = false;
  float target_ = 0.0f;
  uint32_t openloop_t0ms_ = 0;
  uint32_t openloop_cool_until_ = 0;
  uint32_t last_motion_cmd_ms_ = 0;
  float user_current_limit_ = foc::kCurrentLimitDefault;
  float openloop_voltage_ = foc_cfg::kOpenLoopVoltageVolts;

  bool current_over_ = false;
  uint32_t current_over_since_ms_ = 0;
  float phase_current_max_ = 0.0f;

  uint32_t last_health_ms_ = 0;
  bool vin_low_ = false;
  uint32_t vin_low_since_ms_ = 0;
  uint8_t sensor_fail_count_ = 0;

  bool cs_suspect_ = false;
  uint32_t cs_suspect_since_ms_ = 0;
};

#endif  // FOC_DRIVE_H

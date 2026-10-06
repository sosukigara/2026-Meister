#ifndef FOC_DRIVE_H
#define FOC_DRIVE_H

// Motor layer: owns the driver, current sense, angle sensor and the BLDCMotor, plus every
// command that can make the winch move.

#include <Arduino.h>
#include <SimpleFOC.h>

#include "foc_board.h"

class FocDrive {
 public:
  explicit FocDrive(FocBoard& board);

  // Whole bring-up order, including the first-time alignment. False means the drive is
  // gated and every motion command will be refused.
  bool begin();

  bool ready() const { return ready_; }

  bool calibrate();
  bool setAlignVoltage(float volts);
  bool setCurrentLimit(float amps);

  bool commandTorque(float torque);
  bool commandVelocity(float vel_rps);
  bool commandAngle(float rad);

  void stop();

  // Call once per loop. Keeps the sensor and shaft_velocity fresh even while disabled.
  void poll();

  bool enabled() const { return motor_.enabled != 0; }
  uint8_t modeCode() const { return static_cast<uint8_t>(motor_.controller); }
  float currentLimitAmps() const { return motor_.current_limit; }
  float zeroElecRad() const { return motor_.zero_electric_angle; }

  // Sensor::getAngle() is non-const in SimpleFOC 2.2.1, so this getter cannot be const.
  float shaftAngleRad() { return sensor_.getAngle(); }
  float shaftVelocityRps() const { return motor_.shaft_velocity; }
  float currentQ() const { return motor_.current.q; }
  float currentD() const { return motor_.current.d; }
  float voltageQ() const { return motor_.voltage.q; }
  float voltageD() const { return motor_.voltage.d; }

  // SimpleFOC 2.2.1 has no resetVelPid() and PIDController keeps its integrator state in
  // protected members, so the only way to drop a wind-up integral is to build a fresh
  // controller from the configured gains.
  void resetVelocityPid();

 private:
  void applyGains();
  void reportZeroElec(const char* tag);

  FocBoard& board_;
  BLDCDriver3PWM driver_;
  InlineCurrentSense cs_;
  MagneticSensorI2C sensor_;
  BLDCMotor motor_;

  bool ready_ = false;
  float torque_sp_ = 0.0f;
};

#endif  // FOC_DRIVE_H

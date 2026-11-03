#ifndef FOC_BOARD_H
#define FOC_BOARD_H

// Board-level plumbing below the motor layer: I2C bus ownership, the AS5600 fast-mode
// register write and the supply-voltage sense gate.

#include <Arduino.h>
#include <Wire.h>

class FocBoard {
 public:
  FocBoard(int bus_index, int sda_pin, int scl_pin);

  void begin();

  TwoWire& i2c() { return i2c_; }

  // The AS5600 CONF register is volatile, so this has to run again on every boot.
  bool setAs5600FastMode();

  // Address-only probe used by the runtime sensor health check. A NACK means the angle
  // read in the same loop is not trustworthy.
  bool as5600Alive();

  float vinVolts() const;
  bool undervoltage() const;

 private:
  TwoWire i2c_;
  int sda_pin_;
  int scl_pin_;
};

#endif  // FOC_BOARD_H

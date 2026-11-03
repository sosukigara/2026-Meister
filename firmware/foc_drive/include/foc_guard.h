#ifndef FOC_GUARD_H
#define FOC_GUARD_H

// Heartbeat gate guard. A hardware timer ISR drops the shared ENABLE pin if the main loop
// stops feeding it while a channel is armed. This covers a hang inside I2C/Serial/SimpleFOC
// where the software deadman in poll() cannot run.
//
// The ISR writes the GPIO output register directly; do not call digitalWrite/Serial from it.

#include <Arduino.h>

class FocGuard {
 public:
  explicit FocGuard(int enable_pin);

  void begin();
  void arm();
  void disarm();
  void feed();
  bool tripped() const { return tripped_; }
  void clearTrip();

 private:
  static void IRAM_ATTR onTimer();

  int pin_;
  volatile uint32_t ticks_ = 0;
  volatile bool armed_ = false;
  volatile bool tripped_ = false;
  bool started_ = false;
  hw_timer_t* timer_ = nullptr;
};

#endif  // FOC_GUARD_H

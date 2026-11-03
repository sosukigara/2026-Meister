#include "foc_guard.h"

#include <soc/gpio_struct.h>

#include "foc_config.h"

namespace {

FocGuard* g_guard = nullptr;

}  // namespace

FocGuard::FocGuard(int enable_pin) : pin_(enable_pin) {}

void IRAM_ATTR FocGuard::onTimer() {
  FocGuard* g = g_guard;
  if (g == nullptr || !g->armed_) return;
  if (++g->ticks_ >= (foc_cfg::kGuardTimeoutMs / 10u)) {
    // Active-high ENABLE on the MKS board. Clear the output latch directly: the gate
    // driver drops as soon as the pin goes low, whatever the main loop is doing.
    GPIO.out_w1tc = (1u << g->pin_);
    g->armed_ = false;
    g->tripped_ = true;
  }
}

void FocGuard::begin() {
  if (started_) return;
  pinMode(pin_, OUTPUT);
  digitalWrite(pin_, LOW);
  g_guard = this;
  timer_ = timerBegin(0, 80, true);  // 80 MHz / 80 = 1 MHz, 1 tick = 1 us
  timerAttachInterrupt(timer_, &FocGuard::onTimer, true);
  timerAlarmWrite(timer_, 10000, true);  // 10 ms
  timerAlarmEnable(timer_);
  started_ = true;
}

void FocGuard::arm() {
  if (tripped_) return;
  if (!armed_) {
    ticks_ = 0;
    armed_ = true;
  }
}

void FocGuard::disarm() {
  armed_ = false;
  ticks_ = 0;
}

void FocGuard::feed() {
  if (!tripped_) ticks_ = 0;
}

void FocGuard::clearTrip() {
  tripped_ = false;
  ticks_ = 0;
}

#ifndef FOC_CONSOLE_H
#define FOC_CONSOLE_H

// Line-oriented serial console: parse, dispatch to the selected channel, and emit the
// 50 Hz telemetry rows for both channels.

#include <Arduino.h>

#include "foc_board.h"
#include "foc_drive.h"
#include "foc_guard.h"
#include "foc_protocol.h"

class FocConsole {
 public:
  FocConsole(FocDrive& drive0, FocDrive& drive1, FocBoard& board);

  void begin();
  void poll();

 private:
  FocDrive& drive(uint8_t ch);
  bool prepareMotion(uint8_t ch);
  void stopBoth(bool clear_fault);
  void dispatch(const foc::Request& req);
  void serviceTelemetry();
  void printRow(uint8_t ch, FocDrive& d, uint32_t now_ms);
  void printHelp();

  FocDrive& drive0_;
  FocDrive& drive1_;
  FocBoard& board_;
  uint8_t selected_ = 0;
  uint32_t next_telemetry_ms_ = 0;
  uint32_t next_iol_ms_ = 0;
  FocGuard guard_;
  bool fault_reported_ = false;
};

#endif  // FOC_CONSOLE_H

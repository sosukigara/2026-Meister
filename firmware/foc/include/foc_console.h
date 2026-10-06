#ifndef FOC_CONSOLE_H
#define FOC_CONSOLE_H

// Line-oriented serial console: parse, dispatch, and emit the 100 Hz telemetry row.

#include <Arduino.h>

#include "foc_board.h"
#include "foc_drive.h"
#include "foc_protocol.h"

class FocConsole {
 public:
  FocConsole(FocDrive& drive, FocBoard& board);

  void begin();
  void poll();

 private:
  void dispatch(const foc::Request& req);
  void serviceTelemetry();
  void printHelp();

  FocDrive& drive_;
  FocBoard& board_;
  uint32_t next_telemetry_ms_ = 0;
};

#endif  // FOC_CONSOLE_H

#include <Arduino.h>

#include "foc_board.h"
#include "foc_config.h"
#include "foc_console.h"
#include "foc_drive.h"

namespace {

// All three live in this one translation unit and are declared in dependency order, which
// is what repo rule R11 asks for: the dependency is then readable straight from the
// declaration order. FocDrive's constructor only stores the reference, it never
// dereferences the board, so no initialisation-order question remains.
FocBoard board(foc_cfg::kI2cBusIndex);
FocDrive drive(board);
FocConsole console(drive, board);

}  // namespace

void setup() {
  Serial.begin(foc_cfg::kSerialBaud);
  Serial.setTimeout(foc_cfg::kLineTimeoutMs);

  board.begin();
  drive.begin();
  console.begin();
}

void loop() { console.poll(); }

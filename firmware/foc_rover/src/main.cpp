#include <Arduino.h>

#include "foc_board.h"
#include "foc_config.h"
#include "foc_console.h"
#include "foc_drive.h"
#include "foc_prefixed_print.h"

namespace {

// All six live in this one translation unit and are declared in dependency order, which is
// what repo rule R11 asks for: the dependency is then readable straight from the
// declaration order. FocDrive's constructor only stores the reference, it never
// dereferences the board, so no initialisation-order question remains.
//
// The monitor wrapper keeps SimpleFOC's "MOT: ..." output on the same '#' one-shot channel
// as the console diagnostics instead of injecting bare lines into the CSV stream.
PrefixedPrint sfPrint(Serial, "#SF ");

FocBoard board0(foc_cfg::kM0Pins.i2c_bus, foc_cfg::kM0Pins.i2c_sda, foc_cfg::kM0Pins.i2c_scl);
FocBoard board1(foc_cfg::kM1Pins.i2c_bus, foc_cfg::kM1Pins.i2c_sda, foc_cfg::kM1Pins.i2c_scl);
FocDrive drive0(board0, foc_cfg::kM0Pins, "M0", foc_cfg::kInvertM0, sfPrint);
FocDrive drive1(board1, foc_cfg::kM1Pins, "M1", foc_cfg::kInvertM1, sfPrint);
FocConsole console(drive0, drive1, board0);

}  // namespace

void setup() {
  Serial.begin(foc_cfg::kSerialBaud);
  Serial.setTimeout(foc_cfg::kLineTimeoutMs);

  board0.begin();
  board1.begin();
  // Both drivers must be configured before either motor_.init() raises the shared ENABLE.
  drive0.beginDrivers();
  drive1.beginDrivers();
  drive0.beginMotor();
  drive1.beginMotor();
  console.begin();
}

void loop() { console.poll(); }

#include "foc_console.h"

#include "foc_config.h"
#include "foc_protocol.h"

FocConsole::FocConsole(FocDrive& drive, FocBoard& board) : drive_(drive), board_(board) {}

void FocConsole::begin() {
  Serial.println(F("#FOE winch console ready. ? for help."));
  Serial.print(F("#VIN "));
  Serial.println(board_.vinVolts(), 2);
  Serial.println(F("#Gate: S stops. W 0 is active braking. No holding brake."));
  Serial.println(foc::kCsvHeader);
}

void FocConsole::poll() {
  if (Serial.available() > 0) {
    const String line = Serial.readStringUntil('\n');
    dispatch(foc::classify(line.c_str()));
  }
  serviceTelemetry();
}

void FocConsole::dispatch(const foc::Request& req) {
  switch (req.cmd) {
    case foc::Command::kNone:
      return;
    case foc::Command::kError:
      Serial.println(F("#ERR bad line, send ?"));
      return;
    case foc::Command::kHelp:
      printHelp();
      return;
    case foc::Command::kStop:
      drive_.stop();
      Serial.println(F("#STOP"));
      return;
    case foc::Command::kCalibrate:
      // calibrate() reports its own #CAL_OK / #CAL_FAIL, so nothing is printed here.
      drive_.calibrate();
      return;
    case foc::Command::kDiagnose:  // TEMP D
      drive_.diagnose();
      return;
    default:
      break;
  }

  if (!req.has_arg) {
    Serial.println(F("#ERR bad line, send ?"));
    return;
  }

  bool ok = false;
  switch (req.cmd) {
    case foc::Command::kSetCurrentLimit:
      if (req.arg <= 0.0f) {
        Serial.println(F("#ERR L needs a positive current"));
        return;
      }
      ok = drive_.setCurrentLimit(req.arg);
      break;
    case foc::Command::kSetAlignVoltage:
      if (req.arg <= 0.0f) {
        Serial.println(F("#ERR Z needs a positive voltage"));
        return;
      }
      ok = drive_.setAlignVoltage(req.arg);
      break;
    case foc::Command::kTorque:
      ok = drive_.commandTorque(req.arg);
      break;
    case foc::Command::kVelocity:
      ok = drive_.commandVelocity(req.arg);
      break;
    case foc::Command::kAngle:
      ok = drive_.commandAngle(req.arg);
      break;
    case foc::Command::kOpenLoop:
      ok = drive_.commandOpenLoopVelocity(req.arg);
      break;
    case foc::Command::kSine:  // TEMP
      ok = drive_.commandSine(req.arg);
      break;
    case foc::Command::kAngleOpenLoop:  // TEMP
      ok = drive_.commandAngleOpenLoop(req.arg);
      break;
    default:
      break;
  }

  // ready_ is false whenever initFOC did not run (no encoder), so the generic "gated"
  // line must only cover commands that actually need the closed loop. Open-loop commands
  // report their own reason (#BUSY or the drive's own #ERR), and appending this here would
  // be misleading noise.
  const bool needs_closed_loop =
      req.cmd == foc::Command::kTorque || req.cmd == foc::Command::kVelocity ||
      req.cmd == foc::Command::kAngle || req.cmd == foc::Command::kSetAlignVoltage;
  if (!ok && needs_closed_loop && !drive_.ready())
    Serial.println(F("#ERR closed loop gated (no encoder). Use O/N/G sensorless."));
}

void FocConsole::printHelp() {
  Serial.println(F("C          recalibrate (zero_elec + sensor direction)"));
  Serial.println(F("L <amps>   current limit, default 1.0"));
  Serial.println(F("Z <volts>  alignment voltage, default 1.0"));
  Serial.println(F("T <torque> torque mode"));
  Serial.println(F("W <rps>    velocity mode. W 0 = ACTIVE BRAKING"));
  Serial.println(F("A <rad>    angle mode"));
  Serial.println(F("O <rps>    SENSORLESS velocity (spin check only, may stall under load)"));
  Serial.println(F("N <amp>    TEMP sensorless sine spin check"));  // TEMP
  Serial.println(F("G <rad>    TEMP sensorless angle (no holding torque)"));  // TEMP
  Serial.println(F("S          STOP. This is the only stop."));
  Serial.println(F("?          this help"));
  Serial.println();
  Serial.println(F("WARN 1: opening the serial monitor resets the ESP32."));
  Serial.println(F("       Never open it while the winch is spinning."));
  Serial.println(F("WARN 2: stop with S. Never with W 0, that is active braking."));
  Serial.println(F("WARN 3: there is no holding brake. The drum free-wheels on power-down."));
  Serial.println(F("WARN 4: alignment moves the phases. Keep the supply current-limited."));
}

void FocConsole::serviceTelemetry() {
  drive_.poll();

  const uint32_t now = millis();

  // While the open loop runs, expose the measured phase current at 2 Hz so the operator
  // can see the software chop holding it near kOpenLoopCurrentAmps. '#'-prefixed, so it
  // stays out of the CSV stream.
  if (drive_.openLoopActive() && drive_.enabled() &&
      static_cast<int32_t>(now - next_iol_ms_) >= 0) {
    next_iol_ms_ = now + 500;
    Serial.print(F("#IOL "));
    Serial.print(drive_.openLoopCurrentAmps(), 2);
    Serial.println(F(" A"));
  }

  if (static_cast<int32_t>(now - next_telemetry_ms_) < 0) return;
  next_telemetry_ms_ = now + foc_cfg::kTelemetryPeriodMs;

  if (!drive_.enabled() && !foc_cfg::kTelemetryWhileDisabled) return;

  char row[foc::kCsvRowCap];
  foc::formatCsvRow(row, sizeof(row), now * 0.001f, drive_.angleForTelemetry(),
                    drive_.shaftVelocityRps(), drive_.currentQ(), drive_.modeCode(),
                    drive_.currentD(), drive_.voltageQ(), drive_.voltageD());
  Serial.println(row);
}

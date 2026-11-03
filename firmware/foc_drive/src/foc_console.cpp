#include "foc_console.h"

#include <cmath>

#include "foc_config.h"
#include "foc_protocol.h"

FocConsole::FocConsole(FocDrive& drive0, FocDrive& drive1, FocBoard& board)
    : drive0_(drive0), drive1_(drive1), board_(board),
      guard_(foc_cfg::kSharedEnablePin) {}

FocDrive& FocConsole::drive(uint8_t ch) { return ch == 1 ? drive1_ : drive0_; }

void FocConsole::begin() {
  Serial.println(F("#FOE drive console ready. ? for help."));
  Serial.print(F("#ROLE "));
  Serial.println(foc_cfg::kRoleName);
  Serial.print(F("#VERSION "));
  Serial.println(foc_cfg::kFirmwareVersion);
  Serial.print(F("#VIN "));
  Serial.println(board_.vinVolts(), 2);
  Serial.println(F("#CH 0 (M command selects the channel; M0 = left, M1 = right)"));
  Serial.println(F("#Gate: S stops BOTH channels and clears a fault. No holding brake."));
  Serial.println(F("#SAFE P=20 (40 magnets); L<=4A Z<=0.60V V<=0.60V O/W<=5rps"));
  Serial.println(F("#SAFE C is still required for closed loop; S clears a fault."));
  Serial.println(F("#SF lines are SimpleFOC diagnostics; all other '#' lines are one-shot."));
  Serial.println(foc::kCsvHeader);
  guard_.begin();
}

void FocConsole::poll() {
  if (Serial.available() > 0) {
    const String line = Serial.readStringUntil('\n');
    dispatch(foc::classify(line.c_str()));
  }
  serviceTelemetry();
}

void FocConsole::stopBoth(bool clear_fault) {
  // Disarm before any blocking call (calibrate/initFOC takes seconds). An armed guard
  // would drop the shared ENABLE mid-alignment and can leave a bogus zero_electric_angle
  // that still returns success.
  guard_.disarm();
  if (clear_fault) {
    drive0_.clearFault();
    drive1_.clearFault();
  }
  drive0_.stop();
  drive1_.stop();
}

bool FocConsole::prepareMotion(uint8_t ch) {
  if (!foc_cfg::kOneChannelAtATime) return true;
  FocDrive& other = drive(static_cast<uint8_t>(ch ^ 1));
  if (other.faulted()) {
    Serial.println(F("#ERR sibling channel fault latched, S to clear"));
    return false;
  }
  if (other.enabled()) {
    // M0/M1 share ENABLE, and stop() drives that pin low. Stopping the sibling while
    // leaving the selected channel "enabled" would silently cut the selected channel's
    // gate; the next deadman resend would then skip enable() and the motor would stop.
    // Stop both so the selected command re-enables from a known-off state.
    stopBoth(false);
  }
  return true;
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
      stopBoth(true);
      guard_.clearTrip();
      Serial.println(F("#STOP both"));
      return;
    case foc::Command::kCalibrate:
      // calibrate() reports its own CAL_OK / CAL_FAIL, so nothing is printed here.
      stopBoth(false);
      drive(selected_).calibrate();
      return;
    case foc::Command::kDiagnose:
      stopBoth(false);
      drive(selected_).diagnose();
      return;
    default:
      break;
  }

  if (!req.has_arg) {
    Serial.println(F("#ERR bad line, send ?"));
    return;
  }

  if (req.cmd == foc::Command::kSelectChannel) {
    if (req.arg != std::floor(req.arg) || req.arg < 0.0f || req.arg > 1.0f) {
      Serial.println(F("#ERR M needs 0 or 1"));
      return;
    }
    selected_ = static_cast<uint8_t>(req.arg);
    Serial.print(F("#CH "));
    Serial.println(selected_);
    return;
  }

  bool ok = false;
  switch (req.cmd) {
    case foc::Command::kSetCurrentLimit:
      if (req.arg <= 0.0f) {
        Serial.println(F("#ERR L needs a positive current"));
        return;
      }
      ok = drive(selected_).setCurrentLimit(req.arg);
      break;
    case foc::Command::kSetAlignVoltage:
      if (req.arg <= 0.0f) {
        Serial.println(F("#ERR Z needs a positive voltage"));
        return;
      }
      stopBoth(false);
      ok = drive(selected_).setAlignVoltage(req.arg);
      break;
    case foc::Command::kSetPolePairs:
      // Range-check the float before the int cast: finite 1e10 is integral but outside
      // int range, and casting it would be undefined behavior.
      if (!foc::validPolePairsArg(req.arg)) {
        Serial.println(F("#ERR P needs an integer 1-50"));
        return;
      }
      stopBoth(false);
      ok = drive(selected_).setPolePairs(static_cast<int>(req.arg));
      break;
    case foc::Command::kSetOpenLoopVoltage:
      if (req.arg <= 0.0f) {
        Serial.println(F("#ERR V needs a positive voltage"));
        return;
      }
      ok = drive(selected_).setOpenLoopVoltage(req.arg);
      break;
    case foc::Command::kTorque:
      if (!prepareMotion(selected_)) return;
      ok = drive(selected_).commandTorque(req.arg);
      break;
    case foc::Command::kVelocity:
      if (!prepareMotion(selected_)) return;
      ok = drive(selected_).commandVelocity(req.arg);
      break;
    case foc::Command::kAngle:
      if (!prepareMotion(selected_)) return;
      ok = drive(selected_).commandAngle(req.arg);
      break;
    case foc::Command::kOpenLoop:
      if (!prepareMotion(selected_)) return;
      ok = drive(selected_).commandOpenLoopVelocity(req.arg);
      break;
    default:
      break;
  }

  // ready_ is false whenever the closed loop is gated (no encoder, no P, or no C), so the
  // generic line must only cover commands that actually need it. Open-loop commands report
  // their own reason and would get misleading noise here.
  const bool needs_closed_loop =
      req.cmd == foc::Command::kTorque || req.cmd == foc::Command::kVelocity ||
      req.cmd == foc::Command::kAngle || req.cmd == foc::Command::kSetAlignVoltage ||
      req.cmd == foc::Command::kSetCurrentLimit;
  if (!ok && needs_closed_loop && !drive(selected_).ready() &&
      !drive(selected_).faulted()) {
    Serial.print(F("#ERR closed loop gated (P/C/no encoder) on "));
    Serial.println(drive(selected_).name());
  }
}

void FocConsole::printHelp() {
  // Help is one-shot output: every line starts with '#' so a CSV reader never sees a bare
  // line with commas.
  Serial.println(F("#HELP M <0|1>   select channel (0 = M0/left, 1 = M1/right)"));
  Serial.println(F("#HELP C         recalibrate selected channel (needs AS5600; P=20 preset)"));
  Serial.println(F("#HELP L <amps>  closed-loop current limit, 0.1-4.0, default 3.0"));
  Serial.println(F("#HELP Z <volts> alignment voltage, 0.10-0.60, default 0.30"));
  Serial.println(F("#HELP T <torque> torque mode (closed loop)"));
  Serial.println(F("#HELP W <rps>   velocity mode |rps|<=5. W 0 = ACTIVE BRAKING"));
  Serial.println(F("#HELP A <rad>   angle mode |rad|<=100"));
  Serial.println(F("#HELP O <rps>   SENSORLESS velocity |rps|<=5, 10 s run then 10 s cooldown"));
  Serial.println(F("#HELP P <pairs> set pole pairs 1-50; pre-set to 20 (40 magnets)"));
  Serial.println(F("#HELP V <volts> open-loop voltage, 0.10-0.60, default 0.30"));
  Serial.println(F("#HELP D         current-sense/winding diagnosis (2 s cooldown)"));
  Serial.println(F("#HELP S         STOP BOTH + clear fault. This is the only stop."));
  Serial.println(F("#HELP ?         this help"));
  Serial.println(F("#HELP"));
  Serial.println(F("#HELP WARN 1: 0.30 V start. Raise with V while watching #IOL. 0.60 V is the ceiling."));
  Serial.println(F("#HELP WARN 2: 4 A is the hard limit; open loop is bounded by voltage and #OC."));
  Serial.println(F("#HELP WARN 3: verify the shunt marking is R010 (10 mohm), not R001."));
  Serial.println(F("#HELP WARN 4: M0/M1 share ENABLE; the bench runs one channel at a time."));
  Serial.println(F("#HELP WARN 5: opening the serial monitor resets the ESP32. Never do it while spinning."));
  Serial.println(F("#HELP WARN 6: W/T/A are deadman commands; resend at >=5 Hz or the board stops."));
  Serial.println(F("#HELP WARN 7: a fault latches; S clears it. Re-motoring a short is not safe."));
}

void FocConsole::serviceTelemetry() {
  guard_.feed();
  if (drive0_.enabled() || drive1_.enabled()) {
    guard_.arm();
  } else {
    guard_.disarm();
  }

  drive0_.poll();
  drive1_.poll();

  if (guard_.tripped()) {
    drive0_.emergencyStop();
    drive1_.emergencyStop();
    if (!fault_reported_) {
      Serial.println(F("#GUARD main loop stalled, gates off. S to clear."));
      fault_reported_ = true;
    }
    guard_.clearTrip();
  }
  if (drive0_.faulted() || drive1_.faulted()) {
    if (!fault_reported_) {
      drive0_.emergencyStop();
      drive1_.emergencyStop();
      guard_.disarm();
      Serial.println(F("#FAULT latched. S to clear."));
      fault_reported_ = true;
    }
  } else if (!drive0_.enabled() && !drive1_.enabled()) {
    fault_reported_ = false;
  }

  const uint32_t now = millis();

  // While an open loop runs, expose the measured phase current at 2 Hz. There is no active
  // current limit in open loop, so this is how the operator watches the winding draw.
  if (static_cast<int32_t>(now - next_iol_ms_) >= 0) {
    next_iol_ms_ = now + 500;
    if (drive0_.openLoopActive() && drive0_.enabled()) {
      Serial.print(F("#IOL0 "));
      Serial.print(drive0_.phaseCurrentMax(), 2);
      Serial.println(F(" A"));
    }
    if (drive1_.openLoopActive() && drive1_.enabled()) {
      Serial.print(F("#IOL1 "));
      Serial.print(drive1_.phaseCurrentMax(), 2);
      Serial.println(F(" A"));
    }
  }

  if (static_cast<int32_t>(now - next_telemetry_ms_) < 0) return;
  next_telemetry_ms_ = now + foc_cfg::kTelemetryPeriodMs;

  if (drive0_.enabled() || foc_cfg::kTelemetryWhileDisabled) printRow(0, drive0_, now);
  if (drive1_.enabled() || foc_cfg::kTelemetryWhileDisabled) printRow(1, drive1_, now);
}

void FocConsole::printRow(uint8_t ch, FocDrive& d, uint32_t now_ms) {
  char row[foc::kCsvRowCap];
  foc::formatCsvRow(row, sizeof(row), now_ms * 0.001f, ch, d.angleForTelemetry(),
                    d.shaftVelocityRps(), d.currentQ(), d.modeCode(), d.currentD(),
                    d.voltageQ(), d.voltageD());
  Serial.println(row);
}

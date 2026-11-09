#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

// Single source of every tunable value in this project (repo rule R2).
// Command limits live in foc_limits.h so the console, the drive and the native tests share
// one definition. This file only aliases the defaults and holds hardware/board values.
//
// Provenance tags:
//   MKSREPO   github.com/makerbase-motor/MKS-ESP32FOC, branch MKS-ESP32-FOC-V2.0,
//             Test Code/11_close_loop_velocity_example
//   SCH       Hardware/MKS ESP32 FOC V2.0_SCH.pdf on the same branch
//   VENDOR    Eaglepower LA8308 KV90 reseller spec (not a primary datasheet)
//   TODO      not confirmed by a primary source -> listed in README.md
//
// This project drives the two wheels of one axle (M0 = left, M1 = right). It is a sibling
// of firmware/foc (the single-channel winch) and shares no code with it.

#include "foc_limits.h"

#if !defined(FOC_ROLE_FRONT) && !defined(FOC_ROLE_MID) && !defined(FOC_ROLE_REAR)
#error "Define exactly one of FOC_ROLE_FRONT / FOC_ROLE_MID / FOC_ROLE_REAR"
#endif

#if (defined(FOC_ROLE_FRONT) + defined(FOC_ROLE_MID) + defined(FOC_ROLE_REAR)) != 1
#error "Exactly one FOC_ROLE_* may be defined"
#endif

namespace foc_cfg {

// ---------------------------------------------------------------------------
// Role (the only thing that differs between the three firmware images)
// ---------------------------------------------------------------------------

#if defined(FOC_ROLE_FRONT)
constexpr char kRoleName[] = "FRONT";
#elif defined(FOC_ROLE_MID)
constexpr char kRoleName[] = "MID";
#else
constexpr char kRoleName[] = "REAR";
#endif

constexpr char kFirmwareVersion[] = "0.2.0";

// Physical mounting can turn a wheel the other way. Keep the per-role table here rather
// than flipping wires: the PC then always commands "positive = forward" on both sides.
// All false until the first bench spin proves the direction.
constexpr bool kInvertM0 = false;  // TODO(bench): set per role after the direction test
constexpr bool kInvertM1 = false;  // TODO(bench): set per role after the direction test

// ---------------------------------------------------------------------------
// Board: MKS ESP32 FOC V2.0, two channels, 12-24 V
// ---------------------------------------------------------------------------

struct MotorPins {
  int pwm_a;
  int pwm_b;
  int pwm_c;
  int enable;
  int cs_a;
  int cs_b;
  int i2c_bus;
  int i2c_sda;
  int i2c_scl;
};

// MKSREPO: BLDCDriver3PWM(32, 33, 25, 12) / InlineCurrentSense(0.01, 50.0, 39, 36)
//          TwoWire(0).begin(19, 18, 400000)
constexpr MotorPins kM0Pins{32, 33, 25, 12, 39, 36, 0, 19, 18};
// MKSREPO: BLDCDriver3PWM(26, 27, 14, 12) / InlineCurrentSense(0.01, 50.0, 35, 34)
//          TwoWire(1).begin(23, 5, 400000)
constexpr MotorPins kM1Pins{26, 27, 14, 12, 35, 34, 1, 23, 5};

// One GPIO drives both gate drivers on the official board. The guard and the stop policy
// depend on that, so make a future pin change fail at compile time.
constexpr int kSharedEnablePin = kM0Pins.enable;
static_assert(kM0Pins.enable == kM1Pins.enable,
              "M0/M1 must share ENABLE; the guard and stop policy assume it");

// M0/M1 both use GPIO12 as ENABLE on the official example (see kM0Pins/kM1Pins). A
// disable on one channel drops both gates; per-wheel isolation has to come from the power
// side. The firmware therefore stops both channels on any fault and runs one channel at a
// time during bench bring-up. false is not supported yet: the console has no dual-channel
// gate coordinator.
constexpr bool kOneChannelAtATime = true;

constexpr float kSupplyVolts = 12.0f;  // USER bench envelope: 12 V / 6 A max.
// Board accepts 12-24 V, but this build targets the 12 V rail only.
constexpr float kUndervoltageVolts = 10.0f;  // boot gate. TODO: reasoned, not measured.
// Runtime gate, deliberately lower than the boot gate so load sag does not nuisance-stop.
// TODO(undervoltage): both thresholds need a loaded 12 V measurement.
constexpr float kRuntimeUndervoltageVolts = 9.0f;
constexpr uint32_t kRuntimeUndervoltageMs = 500;

// MKSREPO: InlineCurrentSense(0.01, 50.0, ...). 6 A -> 3.0 V, 6.6 A -> ADC full scale.
// Some reseller pages claim 1 mohm / +/-33 A; SCH and the official example say 10 mohm.
// Verify the shunt marking (R010) before trusting the current numbers.
constexpr float kShuntOhms = 0.01f;        // MKSREPO (verify R010 on the board)
constexpr float kCurrentSenseGain = 50.0f;  // MKSREPO / INA240A2
constexpr float kCurrentSenseFullScaleAmps = 6.6f;

// VENDOR: Eaglepower LA8308 KV90 lists 0.186 ohm. It is used only for the low-speed
// current-sense plausibility check, never for control gains. Measure it before trusting
// the check on a different motor.
constexpr float kPhaseResistanceOhms = 0.186f;

constexpr int kVinSensePin = 13;           // MKSREPO
constexpr float kVinDividerRatio = 8.5f;   // MKSREPO

constexpr int kI2cClockHz = 400000;  // MKSREPO

// AS5600 fast mode (Infineon AS5600 datasheet Fig 21-22: CONF high 0x07 bits [1:0]
// = SF, 0x03 = 2x fastest). CONF is volatile (OTP reset at power-up), so this write
// must be repeated on every boot BEFORE the first sensor read. A NACK here is the
// per-channel sensor-absent signal: initFOC() is skipped rather than run blind.
constexpr uint8_t kAs5600Address = 0x36;
constexpr uint8_t kAs5600ConfRegister = 0x07;
constexpr uint8_t kAs5600ConfFastMask = 0x03;

// ---------------------------------------------------------------------------
// Motor: LA8308 KV90
// ---------------------------------------------------------------------------

// The LA8308 family is a 40-magnet / 20-pole-pair rotor. The vendor pages do not publish
// this, but two independent ODrive community reports and a working ODrive config for the
// Eaglepower 8308 90KV agree on 20 pole pairs:
//   https://discourse.odriverobotics.com/t/callibration-issue-when-motor-magnets-arent-alligned-with-coils/8806
//   https://discourse.odriverobotics.com/t/odrive-s1-motor-calibration-error/11326
//   https://github.com/AtomsClark/Integrated-Cycloidal-Robotic-Actuator/blob/main/firmware/odrivetool-session.md
// Count the actual rotor magnets (÷2) before trusting it on a different variant. 7 is the
// old 5010 placeholder and MUST NOT be used: SimpleFOC's alignSensor() only prints a PP
// mismatch, it still returns success, so initFOC()==1 is not evidence.
constexpr int kDefaultPolePairs = 20;
constexpr int kConfirmedPolePairs = 20;  // 0 = unconfirmed -> closed loop is gated

// Boot alignment moves the wheels. Keep it off on the rover; run `C` explicitly when the
// area is clear.
constexpr bool kAutoAlignOnBoot = false;

// Board ceiling: 12 A total peak / ~6 A per channel with cooling (reseller figure).
// The firmware cap is deliberately lower: 6 A is at ~91% of the 6.6 A ADC full scale, and
// continuous 6 A/ch has not been measured. Command limits live in foc_limits.h.
constexpr float kCurrentLimitAmps = foc::kCurrentLimitDefault;

// Open-loop backstop. At the 0.60 V ceiling and 0.186 ohm the winding draws ~3.2 A, so a
// normal bench run stays below this trip. It is for a phase short or a gross shunt error.
constexpr float kOpenLoopOcTripAmps = 4.0f;
constexpr uint32_t kOpenLoopOcTripMs = 500;

// Closed-loop hard trip. Above the 4 A command cap and below ADC saturation. A reading at
// full scale is itself a fault (the controller cannot regulate what it cannot measure).
constexpr float kClosedLoopOcTripAmps = 5.0f;
constexpr uint32_t kClosedLoopOcTripMs = 200;

// Open-loop voltage. 0.30 V is ~1.6 A at 0.186 ohm; 0.60 V is the hard ceiling (~3.2 A).
// The command is clamped by foc::clampOpenLoopVoltage().
constexpr float kOpenLoopVoltageVolts = 0.30f;
constexpr uint32_t kOpenLoopTimeoutMs = 10000;  // 10 s run
constexpr uint32_t kOpenLoopCooldownMs = 10000;  // 10 s before the next open-loop run

// Closed-loop deadman. W/T/A are not one-shot: the PC has to keep sending them (>= 5 Hz)
// or the board disables the driver. Open loop has its own 10 s timeout instead. A resend
// only updates the target; it must not re-enable the driver or reset the velocity PID.
constexpr uint32_t kCommandTimeoutMs = 500;

// Closed-loop ceiling. Full 12 V (the USER rail) is allowed because the FOC current
// loop (not this limit) is what bounds torque; the USER 6 A envelope is enforced by
// the 4 A command hard cap plus the 4/5 A OC trips. Start the first closed-loop
// test with a low `L` (e.g. 1 A).
constexpr float kVoltageLimitVolts = 12.0f;
// Alignment is voltage-mode, not current-limited. 0.30 V is ~1.6 A at 0.186 ohm. The
// command is clamped to foc::kAlignVoltageMin..Max (0.10-0.60 V).
constexpr float kAlignVoltageVolts = 0.30f;

// SimpleFOC's velocity_limit unit is rad/s despite the historical name. 5 rev/s (~31.4
// rad/s) is the bench cap; set it from the wheel diameter and gearing before driving.
constexpr float kVelocityLimitRps = foc::kClosedLoopMaxRps * 6.2831853f;

// Heartbeat guard: while a channel is enabled, the main loop must call feed() at least
// every kGuardTimeoutMs or the timer ISR drops the shared ENABLE pin. This covers an I2C
// or library hang that the software deadman cannot service.
constexpr uint32_t kGuardTimeoutMs = 500;

// Low-speed current-sense plausibility check. At standstill the winding current follows
// V/R; if the measured current is far below that, the shunt/gain is wrong or the ADC is
// disconnected. 0.25 is a wide band so normal ripple does not trip it.
constexpr bool kCurrentSenseCheckEnabled = true;
constexpr float kCsCheckMaxSpeedRad = 1.0f;
constexpr float kCsCheckMinVoltageVolts = 0.30f;
constexpr float kCsCheckMinRatio = 0.25f;
// At the 5 rev/s bench cap the back-EMF is ~3.3 V, so >6 V (half the 12 V rail)
// with no measured current cannot be a legitimate no-load point. It means the
// current loop is saturated against a dead shunt/gain. Re-tune this if
// kClosedLoopMaxRps is raised or the rail voltage changes.
constexpr float kCsLossVoltageVolts = 6.0f;
constexpr float kCsLossCurrentAmps = 0.2f;
constexpr uint32_t kCsCheckMs = 200;
constexpr uint32_t kHealthPeriodMs = 50;
constexpr uint32_t kDiagnoseCooldownMs = 2000;

constexpr float kPidCurrentP = 1.0f;  // ARTICLE (official V2.0 example 11 agrees)
constexpr float kPidCurrentI = 500.0f;  // ARTICLE (official V2.0 example 11 agrees)
constexpr float kPidVelP = 0.2f;  // ARTICLE. Official V2.0 example 11 uses 0.021 for
constexpr float kPidVelI = 2.0f;  // its 2808 motor: try those if this oscillates.
constexpr float kPidVelD = 0.0f;  // TODO(gains): bench-tune on LA8308 KV90.
constexpr float kPidAngleP = 8.0f;
constexpr float kLpfCurrentTf = 0.002f;

// ---------------------------------------------------------------------------
// Console / telemetry
// ---------------------------------------------------------------------------

constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kLineTimeoutMs = 20;
// Two rows per board (M0 and M1) at 50 Hz is ~7.5 kB/s, ~65 % of a 115200 baud link.
constexpr uint32_t kTelemetryPeriodMs = 20;
constexpr bool kTelemetryWhileDisabled = false;

}  // namespace foc_cfg

#endif  // FOC_CONFIG_H

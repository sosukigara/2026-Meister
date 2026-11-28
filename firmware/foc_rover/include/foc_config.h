#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

#include "foc_limits.h"

#if !defined(FOC_ROLE_FRONT) && !defined(FOC_ROLE_MID) && !defined(FOC_ROLE_REAR)
#error "Define exactly one of FOC_ROLE_FRONT / FOC_ROLE_MID / FOC_ROLE_REAR"
#endif

#if (defined(FOC_ROLE_FRONT) + defined(FOC_ROLE_MID) + defined(FOC_ROLE_REAR)) != 1
#error "Exactly one FOC_ROLE_* may be defined"
#endif

namespace foc_cfg {

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

constexpr MotorPins kM0Pins{32, 33, 25, 12, 39, 36, 0, 19, 18};
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

constexpr float kSupplyVolts = 12.0f;
// Board accepts 12-24 V, but this build targets the 12 V rail only.
constexpr float kUndervoltageVolts = 10.0f;  // boot gate. TODO: reasoned, not measured.
// Runtime gate, deliberately lower than the boot gate so load sag does not nuisance-stop.
// TODO(undervoltage): both thresholds need a loaded 12 V measurement.
constexpr float kRuntimeUndervoltageVolts = 9.0f;
constexpr uint32_t kRuntimeUndervoltageMs = 500;

constexpr float kShuntOhms = 0.01f;
constexpr float kCurrentSenseGain = 50.0f;
constexpr float kCurrentSenseFullScaleAmps = 6.6f;

constexpr float kPhaseResistanceOhms = 0.186f;

constexpr int kVinSensePin = 13;
constexpr float kVinDividerRatio = 8.5f;

constexpr int kI2cClockHz = 400000;

// CONF is volatile (OTP reset at power-up), so this write must be repeated on every boot
// BEFORE the first sensor read. A NACK here is the per-channel sensor-absent signal:
// initFOC() is skipped rather than run blind.
constexpr uint8_t kAs5600Address = 0x36;
constexpr uint8_t kAs5600ConfRegister = 0x07;
constexpr uint8_t kAs5600ConfFastMask = 0x03;

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
constexpr float kOpenLoopVoltageVolts = 0.30f;
constexpr uint32_t kOpenLoopTimeoutMs = 10000;
constexpr uint32_t kOpenLoopCooldownMs = 10000;

// The PC must keep sending motion commands (>= 5 Hz) or the driver disables. A resend only
// updates the target; it must not re-enable the driver or reset the velocity PID.
constexpr uint32_t kCommandTimeoutMs = 500;

// Closed-loop ceiling. Full 12 V (the USER rail) is allowed because the FOC current
// loop (not this limit) is what bounds torque; the USER 6 A envelope is enforced by
// the 4 A command hard cap plus the 4/5 A OC trips. Start the first closed-loop
// test with a low `L` (e.g. 1 A).
constexpr float kVoltageLimitVolts = 12.0f;
// Alignment is voltage-mode, not current-limited. 0.30 V is ~1.6 A at 0.186 ohm.
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

constexpr float kPidCurrentP = 1.0f;
constexpr float kPidCurrentI = 500.0f;
constexpr float kPidVelP = 0.2f;
constexpr float kPidVelI = 2.0f;
constexpr float kPidVelD = 0.0f;  // TODO(gains): bench-tune on LA8308 KV90.
constexpr float kPidAngleP = 8.0f;
constexpr float kLpfCurrentTf = 0.002f;

constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kLineTimeoutMs = 20;
// Two rows per board (M0 and M1) at 50 Hz is ~7.5 kB/s, ~65 % of a 115200 baud link.
constexpr uint32_t kTelemetryPeriodMs = 20;
constexpr bool kTelemetryWhileDisabled = false;

}  // namespace foc_cfg

#endif  // FOC_CONFIG_H

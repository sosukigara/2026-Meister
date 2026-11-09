#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

// Single source of every tunable value in this project (repo rule R2).
// No other file may define, override or fall back to any of these.
//
// Provenance tags used below:
//   MKSREPO  github.com/makerbase-motor/MKS-ESP32FOC, branch MKS-ESP32-FOC-V2.0,
//            dir "Test Code/", files 11_close_loop_velocity_example*.ino and
//            12_close_loop_position_example*.ino
//   ARTICLE  the Japanese article sketch this firmware follows (5010/24 V era gains)
//   SIMPLEFOC SimpleFOC 2.2.1 sources read at
//            firmware/foc/.pio/libdeps/foc_m1/Simple FOC/src
//   VENDOR    Eaglepower LA8308 KV90 reseller spec (OL Electronics / Amazon,
//            not a primary datasheet). KV90: 22 A continuous, 900 W, 6S-12S,
//            0.186 ohm, idle 0.6 A @24 V. R phase convention unpublished.
//   USER      bench envelope is 12 V / 6 A max. Every voltage below is sized
//            so its stall current stays under 6 A on the 0.186 ohm winding.
//   TODO      not confirmed by a primary source -> listed in README.md

namespace foc_cfg {

constexpr int kDriverPinA = 26;    // MKSREPO  BLDCDriver3PWM(26, 27, 14, 12), M1
constexpr int kDriverPinB = 27;    // MKSREPO
constexpr int kDriverPinC = 14;    // MKSREPO
constexpr int kDriverEnable = 12;  // MKSREPO  M1 gate driver ENABLE
                                  // GPIO12 is an ESP32 strapping pin: hold it in the
                                  // bootloader state with a pull-DOWN or nothing. Never
                                  // add a pull-up here, the board would enter download
                                  // mode instead of starting the sketch.
constexpr float kSupplyVolts = 12.0f;  // bench PSU. The official example targets
                                       // a 12 V pack, this build follows it now
                                       // (was 24 V in the 5010 era).

constexpr float kShuntOhms = 0.01f;      // MKSREPO  InlineCurrentSense(0.01f, 50.0f, 35, 34)
constexpr float kCurrentSenseGain = 50.0f;  // MKSREPO
constexpr int kCurrentSensePinA = 35;    // MKSREPO  M1 InlineCurrentSense(0.01, 50.0, 35, 34)
constexpr int kCurrentSensePinB = 34;    // MKSREPO

constexpr int kI2cBusIndex = 1;      // MKSREPO  TwoWire(1) for M1. M0 uses TwoWire(0).
constexpr int kI2cSdaPin = 23;       // MKSREPO  M1 begin(23, 5, 400000UL)
constexpr int kI2cSclPin = 5;        // MKSREPO
constexpr uint32_t kI2cClockHz = 400000UL;  // MKSREPO

// AS5600 fast mode. The CONF register is volatile, so this write must be repeated on
// every boot. Bit 0-1 select the digital filter (0x03 = fastest).
constexpr uint8_t kAs5600Address = 0x36;      // SIMPLEFOC sensors/MagneticSensorI2C.cpp:5
constexpr uint8_t kAs5600ConfRegister = 0x07; // AS5600 datasheet CONF register
constexpr uint8_t kAs5600ConfFastMask = 0x03; // AS5600 datasheet, CONF bit 0-1

constexpr int kVinSensePin = 13;      // MKSREPO  analogReadMilliVolts(13)
constexpr float kVinDividerRatio = 8.5f;  // MKSREPO  * 8.5 / 1000
constexpr float kUndervoltageVolts = 10.0f; // MKSREPO used 11.1 V for a 12 V pack
                                           // (3S LiPo cutoff). That leaves no sag margin
                                           // on a 12 V bench PSU, so this build gates at
                                           // 10.0 V instead. TODO(undervoltage): reasoned
                                           // pick, not measured. A loaded 12 V bench check
                                           // must confirm it before use.

// Motor: Eaglepower LA8308 KV90 (user decided). Outrunner, 8308 stator,
// 92x28.5 mm, 336 g, 6 mm shaft, 6S-12S. VENDOR (reseller, not a datasheet):
// OL Electronics LA8308 KV90 page + Amazon LA8308 listing agree on
//   KV90, 22 A continuous, 900 W, 0.186 ohm, idle 0.6 A @24 V, 36N40P.
// 12 V is below the rated 22.2 V minimum, so this build is a low-speed
// bench/winch setup, not a flight rating. R phase convention unpublished.
// Change BOTH kMotorKv and kPhaseResistanceVendorOhms together (repo rule R7).
constexpr int kMotorKv = 90;  // VENDOR + user (KV90 confirmed 2026-11-09).
                              // If the bell marking differs, change this AND the R below.
constexpr float kPhaseResistanceVendorOhms = 0.186f;  // VENDOR, KV90 row. Sizing
                                                      // only, never a control gain.
// 36N40P -> 20 pole pairs. Vendor pages do not publish the pole count; two
// independent ODrive community reports plus a working ODrive config for the
// Eaglepower 8308 90KV agree on 20 PP (same evidence as firmware/foc_drive).
// TODO(magnets): count the rotor magnets (/2) before trusting closed loop.
constexpr int kPolePairs = 20;
constexpr float kGearRatio = 6.0f;     // documented only, no control path consumes it
constexpr float kDrumRadiusM = 0.020f;  // documented only, no control path consumes it

constexpr float kCurrentLimitAmps = 1.0f;  // closed-loop torque ceiling. Far below
                                        // the motor 22 A continuous (VENDOR) and the
                                        // USER 6 A bench envelope, so the first spin
                                        // stresses neither. ARTICLE value kept.
// phase_resistance is deliberately NOT set. Setting it makes SimpleFOC derive the
// open-loop voltage as Uq = current_limit * phase_resistance
// (BLDCMotor.cpp:622-623,660-661) instead of using kOpenLoopVoltageVolts below.
// That product would be 0.186 V (1.0 A * 0.186 ohm); ~16 of the 1023 ESP32
// LEDC steps at 12 V (12/1023 ~ 0.0117 V), so the rotating field still turns,
// but the margin is thinner than on the 5010 and the reason for leaving it
// unset stands: keep the open-loop voltage explicit via kOpenLoopVoltageVolts.
// The open-loop current is bounded by physics, not by a feedback loop: at the
// 0.3 V kOpenLoopVoltageVolts below, stall is 0.3/0.186 = 1.61 A (~0.5 W).
// The old 1.0 V would be 5.38 A / ~5.4 W on this winding - uncomfortably close
// to the USER 6 A envelope, never go back there. See README "Sensorless protection".
// poll() therefore only trips on a hard fault.
constexpr float kOpenLoopOcTripAmps = 4.0f;  // hard-fault backstop (a phase short, a wrong
                                             // KV selection, or a grossly wrong shunt
                                             // reading). Normal 0.3 V open loop on the
                                             // KV90 winding draws ~1.61 A and stays
                                             // clear; a shorted winding or a wrong
                                             // low-R motor on the same 0.3 V exceeds
                                             // 4 A and trips by design. 4 A < USER 6 A
                                             // and < 6.6 A ADC full scale.
constexpr uint32_t kOpenLoopOcTripMs = 500;  // must stay over the trip this long to stop
constexpr uint32_t kOpenLoopCooldownMs = 10000;  // after any open-loop stop the next
                                                 // O/N/G is refused for 10 s: duty
                                                 // never exceeds 50 percent
constexpr float kVelocityLimitRps = 5.0f;  // bench winch cap for the KV90 winding.
                                           // SimpleFOC's velocity_limit is rad/s, not
                                           // rev/s despite this name: it bounds the
                                           // angle_openloop (G) slew and the closed-loop
                                           // W ceiling. 5 rad/s ~= 0.8 rev/s motor,
                                           // ~0.13 rev/s at the 6:1 drum.
constexpr float kVoltageLimitVolts = 1.0f;  // closed-loop saturation backstop, sized by
                                        // the USER 6 A envelope: 6 A * 0.186 ohm =
                                        // 1.116 V, so 1.0 V stalls at ~5.38 A - under
                                        // 6 A and under the 6.6 A ADC full scale, yet
                                        // above the 4 A OC trip so a saturated fault
                                        // still trips instead of sitting at the rail.
                                        // The current PI holds the current, this only
                                        // caps transients: at 5 rad/s on KV90, BEMF
                                        // ~0.53 V (Ke = 60/(90*2pi) = 0.106 V per
                                        // rad/s) plus 1 A * 0.186 ohm ~ 0.19 V =
                                        // ~0.72 V, so 1.0 V leaves ~39% headroom.
                                        // 2.0 V (BE4108 era) would be ~10.8 A stall
                                        // here - never go back there.
constexpr float kOpenLoopVoltageVolts = 0.3f;  // open-loop modes apply this as a fixed voltage
                                              // with no current feedback. Stall is 0.3/0.186
                                              // = 1.61 A (~0.5 W) on the KV90 winding.
                                              // The MKS example's 0.5 V would be ~2.7 A
                                              // here; 1.0 V (5010 era) would be ~5.4 A,
                                              // too close to the USER 6 A envelope.
                                              // The 10 s run / 10 s cooldown caps the
                                              // average. 2.0 V is what smoked the 5010
                                              // wiring - never go back there.
constexpr uint32_t kOpenLoopTimeoutMs = 10000;  // open loop always stops itself:
                                                // bounds the energy even if the
                                                // current is higher than assumed
constexpr float kSineFreqHz = 0.25f;  // TEMP: N command sine rate, delete with N
constexpr float kAlignVoltageVolts = 0.3f;  // alignment bypasses the current loop, so
                                        // this voltage lands directly as 0.3/0.186 stall
                                        // (~1.61 A on KV90). 1.0 V (ARTICLE) would be
                                        // ~5.38 A on this winding, too close to the
                                        // USER 6 A envelope. If C fails,
                                        // raise in 0.1 V steps and watch #IOL, never
                                        // jump to 1 V.

// Gains below are the ARTICLE (5010) starting point. The LA8308 KV90 has higher R
// but far lower KV than the BE4108, so the current loop may need less I and the
// velocity loop a retune.
// TODO(gains): bench-tune on LA8308 KV90 before trusting closed loop with load.
constexpr float kPidCurrentP = 1.0f;   // ARTICLE
constexpr float kPidCurrentI = 500.0f;  // ARTICLE
constexpr float kPidVelP = 0.2f;        // ARTICLE
constexpr float kPidVelI = 2.0f;        // ARTICLE
constexpr float kPidVelD = 0.0f;        // SIMPLEFOC DEF_PID_VEL_D
constexpr float kPidAngleP = 8.0f;      // ARTICLE
constexpr float kLpfCurrentTf = 0.002f;  // ARTICLE

// LA8308 KV90 healthy window for zero_elec: unmeasured. The 3.12-3.42 rad ARTICLE window
// was for the 5010 and MUST NOT be reused. Until bench data sets these, the boot
// note always warns (see FocDrive::reportZeroElec). Only used for the note, the
// firmware never clamps or rejects on it.
constexpr bool kZeroElecWindowCalibrated = false;
constexpr float kZeroElecHealthyMin = 0.0f;  // TODO(zero_elec): placeholder
constexpr float kZeroElecHealthyMax = 0.0f;  // TODO(zero_elec): placeholder

constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kLineTimeoutMs = 20;  // Arduino's 1000 ms default stalls loop() for a
                                        // whole second on a bare readStringUntil().
constexpr uint32_t kTelemetryPeriodMs = 10;  // 100 Hz
constexpr bool kTelemetryWhileDisabled = false;
  // A stopped motor does not need a 100 Hz log. Leaving it on keeps the 115200 baud line
  // busy and buries the one-shot reports the operator needs (zero_elec, current limit).

}  // namespace foc_cfg

#endif  // FOC_CONFIG_H

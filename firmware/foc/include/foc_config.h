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
//   VENDOR    DYS BE4108 reseller spec (dys.hk/product/BE4108.html etc, not a
//            primary datasheet). R/KV/current below are vendor figures.
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

// Motor: DYS BE4108 (sold as "RC BE4108"). Outrunner, Phi46x25, 113 g,
// 4 mm shaft, stator 41x8 mm, 3-6S. VENDOR dys.hk/product/BE4108.html.
// Same stator/rotor for all windings, only the winding differs:
//   KV   R[ohm]  Imax[A]  Pmax[W]
//   380  0.193   17.2     381
//   480  0.125   27       599
//   580  0.098   37       821
//   720  0.055   30       666
// R is the vendor "internal resistance" (phase convention unpublished).
// Change BOTH kMotorKv and kPhaseResistanceVendorOhms together (repo rule R7).
constexpr int kMotorKv = 580;  // VENDOR. Operator confirmed 580KV. If the bell
                               // marking differs, change this AND the R below.
constexpr float kPhaseResistanceVendorOhms = 0.098f;  // VENDOR, 580KV row. Sizing
                                                      // only, never a control gain.
// 24N22P -> 11 pole pairs. DYS truncates "Configuration ...NP" on the official
// page, so this is secondary-source only (SimpleFOC community "Most 4108 motors
// are 24N22P" + aotenjo 24n22p rewind + MKS Dual FOC BLDCMotor(11) example).
// TODO(magnets): count the rotor magnets (/2) before trusting closed loop.
constexpr int kPolePairs = 11;
constexpr float kGearRatio = 6.0f;     // documented only, no control path consumes it
constexpr float kDrumRadiusM = 0.020f;  // documented only, no control path consumes it

constexpr float kCurrentLimitAmps = 1.0f;  // closed-loop torque ceiling. Far below
                                        // both the motor Imax (17-37 A vendor) and the
                                        // board peak (~6 A/ch), so the first spin
                                        // stresses neither. ARTICLE value kept.
// phase_resistance is deliberately NOT set. Setting it makes SimpleFOC derive the
// open-loop voltage as Uq = current_limit * phase_resistance
// (BLDCMotor.cpp:622-623,660-661) instead of using kOpenLoopVoltageVolts below.
// That product would be 0.19 V (380KV) down to 0.055 V (720KV); the low end is
// ~5 of the 1023 ESP32 LEDC steps at 12 V (12/1023 ~ 0.0117 V), so the rotating
// field goes coarse and the rotor vibrates instead of turning. The 5010/24 V era
// hit the same trap at 0.015 V vs a 0.0235 V step and produced no output at all.
// The open-loop current is bounded by physics, not by a feedback loop: at the
// 0.3 V kOpenLoopVoltageVolts below, stall is 0.3/R = 1.55 A (380KV) to 5.45 A
// (720KV), 0.5-1.6 W. The old 1.0 V would be 5.2-18.2 A / 5-18 W on this motor -
// never go back there. See README "Sensorless protection".
// poll() therefore only trips on a hard fault.
constexpr float kOpenLoopOcTripAmps = 4.0f;  // hard-fault backstop (a phase short, a wrong
                                             // KV selection, or a grossly wrong shunt
                                             // reading). Normal 0.3 V open loop on the
                                             // 580KV winding draws ~3.06 A and stays
                                             // clear; a 720KV motor on the same 0.3 V
                                             // draws ~5.45 A and trips by design.
constexpr uint32_t kOpenLoopOcTripMs = 500;  // must stay over the trip this long to stop
constexpr uint32_t kOpenLoopCooldownMs = 10000;  // after any open-loop stop the next
                                                 // O/N/G is refused for 10 s: duty
                                                 // never exceeds 50 percent
constexpr float kVelocityLimitRps = 20.0f;  // MKS V2.0 example uses 20. SimpleFOC's
                                           // velocity_limit is rad/s, not rev/s despite
                                           // this name: it bounds the angle_openloop
                                           // (G) slew and the closed-loop W ceiling.
constexpr float kVoltageLimitVolts = 2.0f;  // closed-loop saturation backstop. The
                                        // current PI holds the current, this only caps
                                        // transients: at 20 rad/s on 580KV, BEMF ~0.33 V
                                        // plus 1 A * 0.098 ohm ~ 0.1 V = ~0.43 V, so 2 V
                                        // leaves ~4x headroom. 5 V (5010/24 V era) would
                                        // be 26-91 A stall on this family if saturated.
constexpr float kOpenLoopVoltageVolts = 0.3f;  // open-loop modes apply this as a fixed voltage
                                              // with no current feedback. Stall is 0.3/R:
                                              // 1.55/2.4/3.06/5.45 A for 380/480/580/720KV.
                                              // The MKS example's 0.5 V would be 2.6-9.1 A
                                              // here; 1.0 V (5010 era) would be 5.2-18.2 A.
                                              // The 10 s run / 10 s cooldown caps the
                                              // average. 2.0 V is what smoked the 5010
                                              // wiring - never go back there.
constexpr uint32_t kOpenLoopTimeoutMs = 10000;  // open loop always stops itself:
                                                // bounds the energy even if the
                                                // current is higher than assumed
constexpr float kSineFreqHz = 0.25f;  // TEMP: N command sine rate, delete with N
constexpr float kAlignVoltageVolts = 0.3f;  // alignment bypasses the current loop, so
                                        // this voltage lands directly as 0.3/R stall
                                        // (~3.06 A on 580KV). 1.0 V (ARTICLE) would be
                                        // 5.2-18.2 A on this motor family. If C fails,
                                        // raise in 0.1 V steps and watch #IOL, never
                                        // jump to 1 V.

// Gains below are the ARTICLE (5010) starting point. The BE4108 has lower R and
// higher KV, so the current loop may need less I and the velocity loop a retune.
// TODO(gains): bench-tune on BE4108 before trusting closed loop with load.
constexpr float kPidCurrentP = 1.0f;   // ARTICLE
constexpr float kPidCurrentI = 500.0f;  // ARTICLE
constexpr float kPidVelP = 0.2f;        // ARTICLE
constexpr float kPidVelI = 2.0f;        // ARTICLE
constexpr float kPidVelD = 0.0f;        // SIMPLEFOC DEF_PID_VEL_D
constexpr float kPidAngleP = 8.0f;      // ARTICLE
constexpr float kLpfCurrentTf = 0.002f;  // ARTICLE

// BE4108 healthy window for zero_elec: unmeasured. The 3.12-3.42 rad ARTICLE window
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

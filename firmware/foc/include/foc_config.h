#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

// Single source of every tunable value in this project (repo rule R2).
// No other file may define, override or fall back to any of these.
//
// Provenance tags used below:
//   MKSREPO  github.com/makerbase-motor/MKS-ESP32FOC, branch MKS-ESP32-FOC-V2.0,
//            dir "Test Code/", files 11_close_loop_velocity_example*.ino and
//            12_close_loop_position_example*.ino
//   ARTICLE  the Japanese article sketch this firmware follows
//   SIMPLEFOC SimpleFOC 2.2.1 sources read at
//            firmware/foc/.pio/libdeps/foc_m1/Simple FOC/src
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
constexpr float kSupplyVolts = 24.0f;  // ARTICLE / bench PSU. The official example targets
                                       // a 12 V pack, this build runs a 24 V supply.

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
constexpr float kUndervoltageVolts = 20.0f; // MKSREPO used 11.1 V for a 12 V pack. This
                                           // build feeds 24 V, so the official value would
                                           // never trip. TODO(undervoltage): 20.0 V is a
                                           // reasoned pick, not a measured one. A 24 V
                                           // bench check must confirm it before use.

constexpr int kPolePairs = 7;        // 5010 360KV, MKSREPO example uses 7
constexpr float kGearRatio = 6.0f;   // documented only, no control path consumes it
constexpr float kDrumRadiusM = 0.020f;  // documented only, no control path consumes it

constexpr float kCurrentLimitAmps = 1.0f;     // ARTICLE
// phase_resistance is deliberately NOT set. Setting it makes SimpleFOC derive the
// open-loop voltage as Uq = current_limit * phase_resistance
// (BLDCMotor.cpp:622-623,660-661). For this low-resistance motor that product is
// ~0.015 V, which is BELOW the ESP32 LEDC resolution at a 24 V supply (10-bit:
// _PWM_RES_BIT 10, _PWM_RES 1023 -> one step is 24/1023 ~ 0.0235 V), so the duty
// rounds to zero and the motor gets no output at all - that was the real "it will
// not spin" cause. See README "Sensorless protection".
// The open-loop current is bounded by physics, not by a feedback loop: at the 0.5 V ceiling
// (kOpenLoopVoltageVolts) this winding draws at most ~2.8 A (0.5 V / ~0.18 ohm) ~ 1.4 W.
// That is why 2.0 V is what smoked the wiring and 0.5 V is safe. An earlier voltage
// fold-back was removed: it dropped the applied voltage to ~0.05 V, which is only 2 of the
// 1023 ESP32 LEDC steps, so the rotating field was far too coarse and the rotor just
// vibrated instead of turning. poll() therefore only trips on a hard fault.
constexpr float kOpenLoopOcTripAmps = 4.0f;  // hard-fault backstop (a phase short, or a
                                             // grossly wrong shunt reading). Normal open
                                             // loop at 0.5 V never reaches this.
constexpr uint32_t kOpenLoopOcTripMs = 500;  // must stay over the trip this long to stop
constexpr uint32_t kOpenLoopCooldownMs = 10000;  // after any open-loop stop the next
                                                 // O/N/G is refused for 10 s: duty
                                                 // never exceeds 50 percent
constexpr float kVelocityLimitRps = 20.0f;  // MKS V2.0 example uses 20. SimpleFOC's
                                           // velocity_limit is rad/s, not rev/s despite
                                           // this name: it bounds the angle_openloop
                                           // (G) slew and the closed-loop W ceiling.
constexpr float kVoltageLimitVolts = 5.0f;   // ARTICLE
constexpr float kOpenLoopVoltageVolts = 0.5f;  // The MKS V2.0 open-loop example value. The
                                              // open-loop modes apply this as a fixed voltage
                                              // with no current feedback, so the current is set
                                              // by the winding: ~4 A at the ~0.13 ohm measured
                                              // here (see README). 2.0 V is what smoked the
                                              // wiring - never go back there.
constexpr uint32_t kOpenLoopTimeoutMs = 10000;  // open loop always stops itself:
                                                // bounds the energy even if the
                                                // current is higher than assumed
constexpr float kSineFreqHz = 0.25f;  // TEMP: N command sine rate, delete with N
constexpr float kAlignVoltageVolts = 1.0f;    // ARTICLE, alignment runs with the motor free

constexpr float kPidCurrentP = 1.0f;   // ARTICLE
constexpr float kPidCurrentI = 500.0f; // ARTICLE
constexpr float kPidVelP = 0.2f;       // ARTICLE
constexpr float kPidVelI = 2.0f;       // ARTICLE
constexpr float kPidVelD = 0.0f;       // SIMPLEFOC DEF_PID_VEL_D
constexpr float kPidAngleP = 8.0f;     // ARTICLE
constexpr float kLpfCurrentTf = 0.002f;  // ARTICLE

// Article's healthy window for this motor's zero_elec. Only used for the boot note,
// the firmware never clamps or rejects on it.
constexpr float kZeroElecHealthyMin = 3.12f;  // ARTICLE
constexpr float kZeroElecHealthyMax = 3.42f;  // ARTICLE

constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kLineTimeoutMs = 20;  // Arduino's 1000 ms default stalls loop() for a
                                        // whole second on a bare readStringUntil().
constexpr uint32_t kTelemetryPeriodMs = 10;  // 100 Hz
constexpr bool kTelemetryWhileDisabled = false;
  // A stopped motor does not need a 100 Hz log. Leaving it on keeps the 115200 baud line
  // busy and buries the one-shot reports the operator needs (zero_elec, current limit).

}  // namespace foc_cfg

#endif  // FOC_CONFIG_H

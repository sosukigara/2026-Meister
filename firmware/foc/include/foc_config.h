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

constexpr int kDriverPinA = 26;    // MKSREPO  BLDCDriver3PWM(26, 27, 14, 12)
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
constexpr int kCurrentSensePinA = 35;    // MKSREPO
constexpr int kCurrentSensePinB = 34;    // MKSREPO

constexpr int kI2cBusIndex = 1;      // MKSREPO  TwoWire(1) for M1. The article used
                                     // TwoWire(0); core 2.x can remap SDA/SCL on bus 0 too,
                                     // so both can work, but the official repo is the
                                     // primary source. TODO(bus-index): needs a bench check.
constexpr int kI2cSdaPin = 23;       // MKSREPO  begin(23, 5, 400000UL)
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
constexpr float kVelocityLimitRps = 130.0f;  // ARTICLE, SimpleFOC native rev/s
constexpr float kVoltageLimitVolts = 5.0f;   // ARTICLE
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

// Spin-test-only firmware: one LA8308 KV90 on M1, fixed safe envelope.
// Bench use only. Never flashed to production (that is firmware/foc_winch).
//
// What it does: open-loop spin (O), explicit alignment (C), closed-loop
// velocity (W) with a 1 A current ceiling. EVERY motion auto-stops after 10 s.
// Fixed voltages/currents only: there are no L/Z commands to get wrong.
//
// Wiring (MKS ESP32 FOC V2.0, M1): motor 3-phase to M1 output, AS5600 to
// SDA23/SCL5 (bus 1, 0x36, magnet diametral on the shaft end, 0.5-3 mm gap),
// 12 V supply current-limited with a fuse in series (no reverse protection).
// Keep the drum/wheel free: boot aligns the motor when a sensor is present.
//
// Envelope math (R = 0.186 ohm vendor, KV90):
//   open-loop/align 0.3 V -> 1.61 A stall; closed-loop ceiling 1.0 V ->
//   5.38 A stall max (under the 6 A bench envelope and 6.6 A ADC full scale);
//   closed-loop current fixed at 1.0 A; BEMF at 5 rad/s ~0.53 V + 0.19 V IR.

#include <Arduino.h>

#include <SimpleFOC.h>

namespace cfg {
// MKSREPO (makerbase-motor/MKS-ESP32FOC, branch MKS-ESP32-FOC-V2.0,
// Test Code/3_dual_as5600_test.ino + 11_close_loop_velocity_example.ino)
constexpr int kPwmA = 26, kPwmB = 27, kPwmC = 14, kGateEn = 12;
constexpr float kSupplyV = 12.0f;
constexpr float kShunt = 0.01f, kGain = 50.0f;
constexpr int kCsA = 35, kCsB = 34;
constexpr int kI2cBus = 1, kSda = 23, kScl = 5;
constexpr int kI2cHz = 400000;
// DATASHEET (Infineon AS5600 Fig 21-22): CONF high 0x07 bits[1:0] = SF.
constexpr uint8_t kAs5600Addr = 0x36, kConfReg = 0x07, kConfFast = 0x03;
// VENDOR (Eaglepower LA8308 KV90 reseller spec): 22 A / 900 W / 0.186 ohm.
constexpr int kPolePairs = 20;  // 36N40P; count magnets (/2) to confirm.
constexpr int kVinPin = 13;
constexpr float kVinRatio = 8.5f, kVinGateV = 10.0f;  // reasoned pick, not measured.
// USER bench envelope: 12 V / 6 A max.
constexpr float kOpenV = 0.3f, kAlignV = 0.3f;
constexpr float kCurLimA = 1.0f, kVoltLimV = 1.0f, kVelLimRps = 5.0f;
constexpr float kOcOlA = 4.0f;
constexpr uint32_t kOcOlMs = 500;
constexpr float kOcClA = 5.0f;
constexpr uint32_t kOcClMs = 200;
constexpr uint32_t kRunMs = 10000, kCoolMs = 10000;  // every motion stops itself.
}  // namespace cfg

BLDCDriver3PWM driver(cfg::kPwmA, cfg::kPwmB, cfg::kPwmC, cfg::kGateEn);
InlineCurrentSense cs(cfg::kShunt, cfg::kGain, cfg::kCsA, cfg::kCsB);
MagneticSensorI2C sensor(AS5600_I2C);
TwoWire i2c(cfg::kI2cBus);
BLDCMotor motor(cfg::kPolePairs);

namespace st {
bool sensor_ok = false;
bool ready = false;  // true once initFOC has succeeded with a sensor.
bool running = false;
uint32_t t0ms = 0;
uint32_t cool_until_ms = 0;
bool over = false;
uint32_t over_since_ms = 0;
float target = 0.0f;
}  // namespace st

float vinV() { return analogReadMilliVolts(cfg::kVinPin) * cfg::kVinRatio / 1000.0f; }

bool as5600FastMode() {
  i2c.beginTransmission(cfg::kAs5600Addr);
  i2c.write(cfg::kConfReg);
  if (i2c.endTransmission(false) != 0) return false;
  if (i2c.requestFrom(static_cast<int>(cfg::kAs5600Addr), 1) != 1) return false;
  const uint8_t conf = static_cast<uint8_t>(i2c.read());
  const uint8_t updated = static_cast<uint8_t>((conf & ~cfg::kConfFast) | cfg::kConfFast);
  if (updated == conf) return true;
  i2c.beginTransmission(cfg::kAs5600Addr);
  i2c.write(cfg::kConfReg);
  i2c.write(updated);
  return i2c.endTransmission(true) == 0;
}

float phaseMaxA() {
  const PhaseCurrent_s c = cs.getPhaseCurrents();
  const float ia = c.a >= 0.0f ? c.a : -c.a;
  const float ib = c.b >= 0.0f ? c.b : -c.b;
  const float icv = c.a + c.b;  // ic is not wired; ia+ib+ic=0 recovers it.
  const float ic = icv >= 0.0f ? icv : -icv;
  const float m = ia > ib ? ia : ib;
  return ic > m ? ic : m;
}

void stop(const __FlashStringHelper* why) {
  motor.disable();
  st::running = false;
  st::over = false;
  st::target = 0.0f;
  motor.voltage_limit = cfg::kVoltLimV;
  if (why != nullptr) Serial.println(why);
}

bool motionAllowed() {
  if (vinV() < cfg::kVinGateV) {
    Serial.println(F("#ERR undervoltage, motion refused"));
    return false;
  }
  return true;
}

bool align(const __FlashStringHelper* tag) {
  // initFOC() only re-runs the alignment when both fields are unset; without
  // this a second C would silently reuse the previous result (SimpleFOC logs
  // "Skip offset calib." in that case).
  motor.zero_electric_angle = NOT_SET;
  motor.sensor_direction = static_cast<int>(NOT_SET);
  motor.enable();
  const bool ok = motor.initFOC() != 0;
  motor.disable();
  st::ready = ok;
  Serial.print(tag);
  Serial.print(' ');
  Serial.println(motor.zero_electric_angle, 4);
  return ok;
}

void poll() {
  const uint32_t now = millis();
  const bool openloop = motor.controller == MotionControlType::velocity_openloop;
  if (!st::running) {
    motor.loopFOC();  // keeps shaft_velocity fresh for telemetry while idle.
    return;
  }
  if (now - st::t0ms > cfg::kRunMs) {
    stop(F("#TIMEOUT motion auto-stopped (10 s test cap)"));
    return;
  }
  const float trip_a = openloop ? cfg::kOcOlA : cfg::kOcClA;
  const uint32_t trip_ms = openloop ? cfg::kOcOlMs : cfg::kOcClMs;
  const float imag = phaseMaxA();
  if (imag > trip_a) {
    if (!st::over) {
      st::over = true;
      st::over_since_ms = now;
    } else if (now - st::over_since_ms > trip_ms) {
      stop(F("#OC overcurrent trip"));
      return;
    }
  } else {
    st::over = false;
  }
  if (openloop) {
    motor.move(st::target);
  } else {
    motor.loopFOC();
    motor.move(st::target);
  }
}

void onLine(const String& line) {
  if (line.length() == 0) return;
  const char cmd = line[0];
  const float arg = line.substring(1).toFloat();
  if (cmd == 'S') {
    if (motor.controller == MotionControlType::velocity_openloop && motor.enabled) {
      st::cool_until_ms = millis() + cfg::kCoolMs;  // open-loop duty never >50%.
    }
    stop(F("#STOP"));
    return;
  }
  if (cmd == '?') {
    Serial.println(F("O [rps] open-loop spin, fixed 0.3 V (default 1 rps, max 2)"));
    Serial.println(F("C       align with AS5600, prints zero_elec"));
    Serial.println(F("W <rps> closed-loop velocity, needs C, max 0.8 rps"));
    Serial.println(F("S       stop. Every motion auto-stops after 10 s."));
    return;
  }
  if (!motionAllowed()) return;
  if (cmd == 'O') {
    float rps = arg == 0.0f ? 1.0f : arg;
    if (rps > 2.0f) rps = 2.0f;
    if (rps < -2.0f) rps = -2.0f;
    if (static_cast<int32_t>(millis() - st::cool_until_ms) < 0) {
      Serial.println(F("#BUSY cooling, wait"));
      return;
    }
    motor.controller = MotionControlType::velocity_openloop;
    motor.current_limit = cfg::kCurLimA;
    motor.voltage_limit = cfg::kOpenV;
    st::target = rps * _2PI;
    st::t0ms = millis();
    st::running = true;
    motor.enable();
    Serial.println(F("#OPENLOOP go"));
    return;
  }
  if (cmd == 'C') {
    if (!st::sensor_ok) {
      Serial.println(F("#ERR no AS5600 (CONF NACK at boot)"));
      return;
    }
    align(F("#CAL"));
    return;
  }
  if (cmd == 'W') {
    if (!st::ready || !st::sensor_ok) {
      Serial.println(F("#ERR closed loop gated (run C with AS5600 first)"));
      return;
    }
    float rps = arg;
    if (rps > 0.8f) rps = 0.8f;  // 0.8 rps = 5.0 rad/s = kVelLimRps.
    if (rps < -0.8f) rps = -0.8f;
    motor.controller = MotionControlType::velocity;
    motor.current_limit = cfg::kCurLimA;
    motor.voltage_limit = cfg::kVoltLimV;
    st::target = rps * _2PI;
    st::t0ms = millis();
    st::running = true;
    motor.enable();
    Serial.println(F("#VELOCITY go"));
    return;
  }
  Serial.println(F("#ERR bad line, send ?"));
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);  // Arduino default 1000 ms would stall loop() a full second.
  i2c.begin(cfg::kSda, cfg::kScl, cfg::kI2cHz);
  // Fast mode must land before the first sensor read. A NACK is the
  // sensor-absent signal: closed loop stays gated, open loop still works.
  st::sensor_ok = as5600FastMode();
  sensor.init(&i2c);
  if (st::sensor_ok) motor.linkSensor(&sensor);

  driver.voltage_power_supply = cfg::kSupplyV;
  driver.init();
  motor.linkDriver(&driver);
  cs.init();
  motor.linkCurrentSense(&cs);

  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;
  motor.current_limit = cfg::kCurLimA;
  motor.velocity_limit = cfg::kVelLimRps;
  motor.voltage_limit = cfg::kVoltLimV;
  motor.voltage_sensor_align = cfg::kAlignV;
  motor.PID_current_q.P = 1.0f;  // ARTICLE (5010) baseline; bench-tune on LA8308.
  motor.PID_current_q.I = 500.0f;
  motor.PID_current_d.P = 1.0f;
  motor.PID_current_d.I = 500.0f;
  motor.LPF_current_q.Tf = 0.002f;
  motor.LPF_current_d.Tf = 0.002f;
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 2.0f;
  motor.P_angle.P = 8.0f;

  Serial.print(F("#TEST foc_test_m1 VIN "));
  Serial.print(vinV(), 2);
  Serial.println(F(" V (needs 10+ to spin)"));
  if (!st::sensor_ok) Serial.println(F("#NOTE no AS5600: O only, C/W gated"));
  motor.init();
  if (st::sensor_ok) {
    Serial.println(F("#NOTE boot aligns: keep the wheel free"));
    align(F("#ZERO_ELEC"));
    motor.disable();
  }
  Serial.println(F("#READY O/C/W/S/? (10 s auto-stop, 12 V/6 A envelope)"));
}

void loop() {
  if (Serial.available() > 0) {
    const String line = Serial.readStringUntil('\n');
    onLine(line);
  }
  poll();
}

#include <Arduino.h>

#include <SimpleFOC.h>

namespace cfg {
constexpr int kPwmA = 26, kPwmB = 27, kPwmC = 14, kGateEn = 12;
constexpr float kSupplyV = 12.0f;
constexpr float kShunt = 0.01f, kGain = 50.0f;
constexpr int kCsA = 35, kCsB = 34;
constexpr int kI2cBus = 1, kSda = 23, kScl = 5;
constexpr int kI2cHz = 400000;
constexpr uint8_t kAs5600Addr = 0x36, kConfReg = 0x07, kConfFast = 0x03;
constexpr int kPolePairs = 20;  // 36N40P; count magnets (/2) to confirm.
constexpr int kVinPin = 13;
constexpr float kVinRatio = 8.5f, kVinGateV = 10.0f;  // reasoned pick, not measured.
constexpr float kOpenV = 0.3f, kAlignV = 0.3f;  // fixed: 2.13 A measured on
                                                // 2026-11-09 (path 0.14 ohm).
constexpr float kCurLimA = 1.0f;
// L hard cap. The board is rated 12 A total / 6 A per motor, and the 10 mohm shunt with
// gain 50 puts ADC full scale at 6.6 A: 5 A is 83% of the motor rating and 76% of what the
// sense path can still resolve. I2R at 5 A is ~3.5 W on the measured 0.14 ohm path, so the
// winding is not what stops us here.
constexpr float kCurMaxA = 5.0f;
constexpr float kClVlimV = 12.0f;  // closed-loop ceiling: the FOC current loop
                                   // (not this) bounds torque. The rail caps the
                                   // speed, and above ~87 rad/s it is exhausted.
constexpr float kVelMaxRps = 14.0f;  // measured 2026-11-09: ~830 rpm (13.8 rps)
                                     // is the no-load plateau on the 12 V rail;
                                     // 16 and 18 rps commands both sat at ~87 rad/s.
constexpr float kOcOlA = 4.0f;
constexpr uint32_t kOcOlMs = 500;
// Must sit above the 5 A command plus switching ripple, and below the 6.47 A full-scale
// guard: 6 A is exactly the board's per-motor rating, so this trips at the rating, not past it.
constexpr float kOcClA = 6.0f;
constexpr uint32_t kOcClMs = 200;
constexpr float kAdcSatA = 6.47f;  // 0.98 * 6.6 A full scale: unmeasurable = fault.
// A switching transient hits both shunts at once, so one railed sample proves
// nothing. The guard has to see the reading persist before it declares the
// current sense unreadable; 10 ms is far shorter than any real overcurrent the
// 6 A/200 ms trip would miss.
constexpr uint32_t kAdcSatMs = 10;
constexpr uint32_t kRunMs = 10000, kCoolMs = 10000;
constexpr uint32_t kSpdMs = 500;
constexpr float kRampRps2 = 30.0f;  // TODO(bench): raise if spin-up feels sluggish.
constexpr uint32_t kHealthMs = 100;
constexpr float kVinRunV = 9.0f;
constexpr uint32_t kVinRunMs = 500;
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
bool sat = false;
uint32_t sat_since_ms = 0;
uint32_t last_spd_ms = 0;
uint32_t last_health_ms = 0;
uint8_t nack_count = 0;
bool vin_low = false;
uint32_t vin_low_since_ms = 0;
float target = 0.0f;
float user_current_limit = cfg::kCurLimA;
}  // namespace st

bool sensorAlive() {
  // Bare ACK ping: no register pointer change, so it never disturbs the angle reads.
  i2c.beginTransmission(cfg::kAs5600Addr);
  return i2c.endTransmission(true) == 0;
}

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

float phaseMax(const PhaseCurrent_s& c) {
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
  st::sat = false;
  st::nack_count = 0;
  st::vin_low = false;
  st::target = 0.0f;
  motor.voltage_limit = cfg::kClVlimV;
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
  // A failed initFOC still yields a zero-angle float, so without this line a
  // broken alignment is indistinguishable from a good one until W is refused.
  if (!ok) Serial.println(F("#CAL_FAIL initFOC failed, closed loop stays gated"));
  return ok;
}

void currentProbe() {
  // Mirrors the stimulus InlineCurrentSense::driverAlign() applies one phase at
  // a time at the align voltage, so these numbers explain its PASS/FAIL directly
  // instead of guessing at the shunt, the amp or the phase wiring. With all three
  // phases connected the driven phase must read ~2x the returning phase.
  motor.enable();
  delay(50);
  const PhaseCurrent_s z = cs.getPhaseCurrents();
  Serial.print(F("#CUR_OFF "));
  Serial.print(z.a, 3);
  Serial.print(' ');
  Serial.println(z.b, 3);
  for (int ph = 0; ph < 3; ++ph) {
    driver.setPwm(ph == 0 ? cfg::kAlignV : 0.0f, ph == 1 ? cfg::kAlignV : 0.0f,
                  ph == 2 ? cfg::kAlignV : 0.0f);
    delay(200);
    PhaseCurrent_s c = cs.getPhaseCurrents();
    for (int i = 0; i < 100; i++) {
      const PhaseCurrent_s c1 = cs.getPhaseCurrents();
      c.a = c.a * 0.6f + 0.4f * c1.a;
      c.b = c.b * 0.6f + 0.4f * c1.b;
      delay(3);
    }
    driver.setPwm(0, 0, 0);
    delay(100);
    Serial.print(F("#CUR_DRIVE_"));
    Serial.print(ph == 0 ? 'A' : (ph == 1 ? 'B' : 'C'));
    Serial.print(F(" ia "));
    Serial.print(c.a, 3);
    Serial.print(F(" ib "));
    Serial.print(c.b, 3);
    Serial.print(F(" ratio "));
    Serial.println(c.b != 0.0f ? fabs(c.a / c.b) : 0.0f, 3);
  }
  motor.disable();
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
  const PhaseCurrent_s cur = cs.getPhaseCurrents();
  const float imag = phaseMax(cur);
  if (imag >= cfg::kAdcSatA) {
    // Unmeasurable current: the controller is flying blind (shunt/gain/wiring).
    // One railed sample is a switching transient, so wait for it to persist and
    // print the sample that tripped: the numbers are the only way to tell the
    // two apart after the fact.
    if (!st::sat) {
      st::sat = true;
      st::sat_since_ms = now;
    } else if (now - st::sat_since_ms > cfg::kAdcSatMs) {
      Serial.print(F("#OC_SAT full scale ia "));
      Serial.print(cur.a, 2);
      Serial.print(F(" ib "));
      Serial.print(cur.b, 2);
      Serial.print(F(" ic "));
      Serial.println(cur.a + cur.b, 2);
      stop(nullptr);
      return;
    }
  } else {
    st::sat = false;
  }
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
  // While spinning, a dead sensor (frozen angle) or a sagging supply is as
  // dangerous as an overcurrent: check both at 10 Hz, stop on persistence.
  if (now - st::last_health_ms > cfg::kHealthMs) {
    st::last_health_ms = now;
    if (vinV() < cfg::kVinRunV) {
      if (!st::vin_low) {
        st::vin_low = true;
        st::vin_low_since_ms = now;
      } else if (now - st::vin_low_since_ms > cfg::kVinRunMs) {
        stop(F("#ERR supply sag, motion stopped"));
        return;
      }
    } else {
      st::vin_low = false;
    }
    if (!openloop) {
      if (!sensorAlive()) {
        if (++st::nack_count >= 2) {
          stop(F("#ERR AS5600 lost mid-run"));
          return;
        }
      } else {
        st::nack_count = 0;
      }
    }
  }
  if (openloop) {
    motor.move(st::target);
  } else {
    motor.loopFOC();
    motor.move(st::target);
  }
  if (now - st::last_spd_ms > cfg::kSpdMs) {
    st::last_spd_ms = now;
    Serial.print(F("#SPD "));
    Serial.print(motor.shaft_velocity, 1);
    Serial.print(F(" rad/s Iq "));
    Serial.println(motor.current.q, 2);
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
    Serial.print(F("L [amps] current ceiling 0.1-"));
    Serial.print(cfg::kCurMaxA, 1);
    Serial.println(F(" A (default 1.0)"));
    // Printed from the constant so the help cannot drift from the clamp.
    Serial.print(F("W <rps> closed-loop velocity, needs C, max "));
    Serial.print(cfg::kVelMaxRps, 0);
    Serial.println(F(" rps"));
    Serial.println(F("I       probe phase currents once (shunt/amp check)"));
    Serial.println(F("S       stop. O/C need a stopped motor; W retargets live."));
    Serial.println(F("Every motion auto-stops after 10 s."));
    return;
  }
  if (cmd == 'L') {
    float amps = arg == 0.0f ? cfg::kCurLimA : arg;
    if (amps < 0.1f) amps = 0.1f;
    if (amps > cfg::kCurMaxA) amps = cfg::kCurMaxA;
    st::user_current_limit = amps;
    if (st::running && motor.controller == MotionControlType::velocity) {
      // In velocity mode the I_q ceiling is PID_velocity.limit, not current_limit: init()
      // baked it once from current_limit, so writing current_limit alone leaves the loop
      // clamped at the boot value and `L` appears to do nothing under load.
      motor.current_limit = amps;
      motor.PID_velocity.limit = amps;
    }
    Serial.print(F("#CUR_LIM "));
    Serial.print(amps, 2);
    Serial.print(F(" Iq_limit "));
    Serial.println(motor.PID_velocity.limit, 2);
    return;
  }
  if (!motionAllowed()) return;
  if (cmd == 'I') {
    if (st::running) {
      Serial.println(F("#ERR stop first (S)"));
      return;
    }
    currentProbe();
    return;
  }
  if (cmd == 'O') {
    if (st::running) {
      Serial.println(F("#ERR stop first (S)"));
      return;
    }
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
    if (st::running) {
      Serial.println(F("#ERR stop first (S): never align mid-spin"));
      return;
    }
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
    if (rps > cfg::kVelMaxRps) rps = cfg::kVelMaxRps;
    if (rps < -cfg::kVelMaxRps) rps = -cfg::kVelMaxRps;
    if (st::running) {
      if (motor.controller != MotionControlType::velocity) {
        Serial.println(F("#ERR stop first (S)"));
        return;
      }
      st::target = rps * _2PI;  // retarget only: the 10 s cap keeps running.
      Serial.println(F("#VELOCITY retarget"));
      return;
    }
    motor.controller = MotionControlType::velocity;
    motor.current_limit = st::user_current_limit;
    motor.PID_velocity.limit = st::user_current_limit;  // the I_q ceiling in velocity mode
    motor.voltage_limit = cfg::kClVlimV;
    st::target = rps * _2PI;
    st::t0ms = millis();
    st::running = true;
    motor.enable();
    // Report the ceiling the velocity loop will actually use: `L` is only a request until
    // this line proves it reached PID_velocity.limit.
    Serial.print(F("#VELOCITY go Iq_limit "));
    Serial.println(motor.PID_velocity.limit, 2);
    return;
  }
  Serial.println(F("#ERR bad line, send ?"));
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);  // Arduino default 1000 ms would stall loop() a full second.
#ifdef FOC_TEST_MONITOR
  // SimpleFOC only reports why initFOC() failed through monitor_port. Build with
  // PLATFORMIO_BUILD_FLAGS="-DFOC_TEST_MONITOR=1" when C fails silently: the
  // MOT: trace separates a weak-align stall from a bad direction/PP/current-sense
  // result, and this firmware otherwise prints no diagnostic for it.
  motor.useMonitoring(Serial);
#endif
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
  motor.current_limit = st::user_current_limit;
  motor.velocity_limit = cfg::kVelMaxRps * _2PI;
  motor.voltage_limit = cfg::kClVlimV;
  motor.voltage_sensor_align = cfg::kAlignV;
  motor.PID_current_q.P = 1.0f;  // TODO(bench): tune on LA8308.
  motor.PID_current_q.I = 500.0f;
  motor.PID_current_d.P = 1.0f;
  motor.PID_current_d.I = 500.0f;
  motor.LPF_current_q.Tf = 0.002f;
  motor.LPF_current_d.Tf = 0.002f;
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 2.0f;
  motor.PID_velocity.output_ramp = cfg::kRampRps2;  // no torque step on W jumps.
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

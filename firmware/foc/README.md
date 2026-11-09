# MKS ESP32 FOC V2.0 winch driver (channel M1)

A **standalone** PlatformIO project for the MKS ESP32 FOC V2.0 board (channel **M1**)
driving a BLDC winch with [SimpleFOC](https://simplefoc.cc) 2.2.1.

It is deliberately **not** part of the sibling `firmware/main_board/` MSTE binary-protocol build. It
has its own `platformio.ini`, its own `.pio/` cache and its own serial command language.
Nothing in `firmware/main_board/src/` imports anything from here.

The behaviour follows a Japanese article sketch. Where the article and the official MKS
repository disagree, the **official repository wins** and the discrepancy is listed in
[未確認](#未確認--要確認) below.

Bring-up history (why it would not spin, why it once overheated, what the safety
guards do and do not guarantee): [docs/bringup_notes.md](docs/bringup_notes.md).

## Versions

| Package | Pin | Why |
|---|---|---|
| Arduino core | 2.0.17 (`framework-arduinoespressif32@3.20017.241212`) | The article reports runaway spin with SimpleFOC 2.4.0 on core 3.x |
| SimpleFOC | 2.2.1 | idem |
| Platform | `espressif32@6.9.0` | |

## Wiring / pin table

Every value below is defined exactly once, in `include/foc_config.h`, with a provenance tag.
Source: `github.com/makerbase-motor/MKS-ESP32FOC`, branch `MKS-ESP32-FOC-V2.0`, dir
`Test Code/`, files `11_close_loop_velocity_example*.ino` and
`12_close_loop_position_example*.ino` (channel M1). The article values match M1;

| Signal | Value | Source |
|---|---|---|
| Motor phase PWM A / B / C | GPIO 26 / 27 / 14 | MKSREPO (M1) |
| Driver enable (ENABLE) | GPIO 12 | MKSREPO |
| Current sense A / B | GPIO 35 / 34 | MKSREPO (M1) |
| Shunt / gain | 0.01 Ω / 50.0 | MKSREPO |
| I2C SDA / SCL / clock | GPIO 23 / 5 / 400 kHz | MKSREPO (M1) |
| I2C bus index | 1 (`TwoWire(1)`) | MKSREPO (M1) |
| AS5600 address | 0x36 | SimpleFOC `sensors/MagneticSensorI2C.cpp:5` |
| VIN sense | GPIO 13, `mV * 8.5 / 1000` | MKSREPO |
| Motor | Eaglepower LA8308 **KV90**, **20** pole pairs (36N40P) | VENDOR (reseller) + ODrive community (see below). **KV90 user-confirmed 2026-11-09** |
| Gear ratio / drum radius | 6.0 / 0.020 m | documented only, **no control path consumes them** |
| Supply | 12.0 V, **6 A max (USER bench envelope)** | this build. 12 V is below the motor's 22.2 V rated minimum: low-speed bench/winch only |

Motor spec ([OL Electronics LA8308 KV90](https://www.olelectronics.com/product/eaglepower-la-8308-kv90-brushless-motor), reseller spec,
not a primary datasheet; R convention unpublished):

| KV | R [Ω] | Imax [A] | Pmax [W] | 0.3 V stall [A] | note |
|---|---|---|---|---|---|
| **90** | **0.186** | **22 (continuous)** | **900** | **1.61** | **this build (KV90 confirmed)** |

12 V / 6 A envelope check on the 0.186 Ω winding: open-loop/align 0.3 V stalls at
1.61 A; closed-loop `voltage_limit` 1.0 V stalls at ~5.38 A (under 6 A, above the
4 A `#OC` trip so a saturated fault still trips); `current_limit` 1.0 A is the
normal ceiling. At 5 rad/s on KV90, BEMF ~0.53 V + 1 A * 0.186 Ω ~ 0.19 V =
~0.72 V, so 1.0 V leaves ~39% headroom.

Pole pairs: vendor pages do not publish the pole count, so 20 PP is
secondary-source only: two independent ODrive community reports plus a working
ODrive config for the Eaglepower 8308 90KV (same evidence as
`firmware/foc_drive`, see its `foc_config.h` links). Count the rotor magnets (/2)
before trusting closed loop.

> **GPIO12 is an ESP32 strapping pin.** Do **not** add a pull-up to it. A pull-up holds
> the chip in the bootloader and the sketch never starts.

## Build / flash / monitor

```bash
cd firmware/foc
pio run -e foc_m1 -t upload
pio device monitor -b 115200
```

Host-side unit test (no ESP32 needed, SimpleFOC is not linked in):

```bash
cd firmware/foc
pio test -e native
```

### Building `foc/` and `main/` in the same PlatformIO home

This project pins `espressif32@6.9.0` (core 2.0.17, `tool-esptoolpy@1.40501.0`) while
`../main/platformio.ini` leaves `platform = espressif32` unpinned (pioarduino 55.3.39, core 3.3.9,
`tool-esptoolpy@5.3.0`). Both resolve against the **same** `~/.platformio` package
directory, so building either one invalidates the other's `tool-esptoolpy` manifest and the
next build of the other reinstalls it.

Symptom: `pio run -e esp32dev` run from `firmware/main_board/` fails once, immediately after any
`espressif32@6.9.0` build, with `TypeError` at
`platforms/espressif32@src-*/builder/frameworks/arduino.py:531`
(`FRAMEWORK_DIR` resolves to `None` while the core is being reinstalled). The second
`pio run -e esp32dev` succeeds. **✗ 未検証** root cause of the reinstall loop itself.

This is not caused by anything in `foc/`'s sources — an empty scratch project pinned to
`espressif32@6.9.0` reproduces it identically. The durable fix is to give the two projects
separate `PLATFORMIO_CORE_DIR`s, which is left to the maintainer because it would discard the
already-verified package set in `firmware/foc/.pio/`.

## Commands

115200 baud, 8N1, **one command per newline-terminated line**. `Serial.setTimeout(20)` is
set in `setup()` because the Arduino default of 1000 ms would stall `loop()` for a full
second on a bare read.

| Command | Effect |
|---|---|
| `C` | Recalibrate. Clears `zero_electric_angle` / `sensor_direction`, enables, `initFOC()`, disables, prints `#CAL_OK`/`#CAL_FAIL` + the new `zero_elec`. |
| `L <amps>` | Current limit. Writes **both** `motor.current_limit` and `motor.PID_velocity.limit`, then clamps the stored torque setpoint. |
| `Z <volts>` | Set `voltage_sensor_align` and re-run `initFOC()`. Use it to confirm the alignment is stable before committing to `C`. |
| `T <torque>` | Torque mode. Enables, then `move()`. Setpoint is clamped to ±`current_limit`. |
| `W <rev/s>` | Velocity mode. Resets the velocity PI integrator **before** enabling, which is what suppresses the start-up jerk. |
| `A <rad>` | Angle mode. Enables, then `move()`. |
| `O <rps>` | Sensorless velocity (spin check only). Works without encoder; may stall under load. |
| `N <amp>` | TEMP sensorless sine spin check at `kSineFreqHz`. Delete with the N command. |
| `G <rad>` | TEMP sensorless angle (no holding torque, may skip under load). |
| `D` | TEMP winding/driver diagnosis: energises phases at the open-loop cap and prints `#DIAG` shunt currents. |
| `S` | **STOP.** `motor.disable()` + velocity-PI reset, prints `#STOP`. |
| `?` | Help plus the safety warnings. |

### Sensorless protection (and its hard limit)

In the 5010/24 V era an open-loop voltage of 2.0 V with no current limit pushed
~20 A through the winding and smoked the wiring. The MKS V2.0 example itself calls
2.0 V "excessive" and uses 0.5 V for an aircraft motor. The LA8308 KV90 winding is
0.186 Ω (vendor), so this build runs lower still: 0.3 V open-loop/align (1.61 A stall),
1.0 V closed-loop ceiling (~5.38 A stall, under the USER 6 A envelope).

What the firmware does:

1. `O`/`N`/`G` run at `kOpenLoopVoltageVolts` (0.3 V), not 2.0 V.
2. Every run stops itself after `kOpenLoopTimeoutMs` (10 s, `#TIMEOUT`), and a new
   `O`/`N`/`G` inside `kOpenLoopCooldownMs` (10 s) is refused (`#BUSY`), so duty
   never exceeds 50 percent.

What the firmware cannot do, and why:

- It cannot use SimpleFOC's built-in current control in open loop. `phase_resistance`
  is deliberately **not** set: setting it makes SimpleFOC derive the open-loop voltage
  as `current_limit x phase_resistance` (`BLDCMotor.cpp:622-623`) instead of using the
  0.3 V ceiling. That product would be 1.0 A * 0.186 Ω = 0.186 V on the KV90
  winding (~16 of the 1023 ESP32 LEDC steps at 12 V, `12/1023` ~ 0.0117 V), so the
  field still resolves, but keeping the open-loop voltage explicit avoids coupling
  the sensorless voltage to the closed-loop current limit. True FOC current control
  (`foc_current`) does limit current but needs the rotor angle, i.e. the encoder, so it
  is unavailable sensorless.

What the firmware does to bound current:

- **The 0.3 V open-loop ceiling is the limit.** SimpleFOC open loop applies a fixed
  voltage with no current feedback, so the current follows physics: stall is `0.3/R` =
  1.61 A (~0.5 W) on the KV90 winding (0.186 Ω). The old 1.0 V would be ~5.4 A /
  ~5.4 W here, too close to the USER 6 A envelope. There is deliberately **no** active current
  loop: an earlier voltage fold-back starved the motor (it pulled the applied voltage down
  to ~0.05 V, only ~4 of the 1023 ESP32 LEDC steps at 12 V, so the rotating field was far
  too coarse and the rotor only vibrated instead of turning). 0.3 V is a short-burst value;
  do not run it continuously without the encoder.
- **`#OC` trip.** poll() reads the shunt (`InlineCurrentSense::getPhaseCurrents()`) and
  stops with `#OC` if the phase current stays above `kOpenLoopOcTripAmps` (4 A) for
  `kOpenLoopOcTripMs` (500 ms) - meant for a phase short or a wrong KV selection, not a
  normal run. A shorted winding on the 0.3 V ceiling exceeds 4 A and trips by design.
- **The bench supply CC bounds the *supply* current, not the winding current.** At 0.3 V the
  PWM duty is only ~2.5 %, so the phase current is ~40x the DC input current: a 0.5 A CC only
  bites at ~20 A of phase current. It protects the wiring and the supply, but it is **not** a
  winding-heat limit at this voltage - the winding heat is `V^2 / R`.

**✗ 未検証**: `#IOL` on the LA8308 KV90 has not been measured yet. The vendor R (0.186 Ω for
KV90) is unconfirmed and its phase convention unpublished, so treat `#IOL` and the `#OC`
trip as **indicative only** until a metered spin (`O` at 0.3 V + `#IOL` + winding-meter R)
closes the loop. A sensorless spin that leaves the motor cool and the driver lukewarm is
the good state to look for.

Software cannot protect a shorted winding or shorted wiring. Never leave a
sensorless run unattended.

### Telemetry

100 Hz CSV on the same port, only while the motor is enabled, preceded by a single
`#`-prefixed header at boot:

```
#t_s,mech_rad,vel_rps,iq_a,mode,id_a,vq_v,vd_v
```

`mode` is the SimpleFOC `MotionControlType` value: `0` torque, `1` velocity, `2` angle,
`3` sensorless velocity (`O`/`N`), `4` sensorless angle (`G`).
A row is ~70 bytes, so 100 Hz uses ~7000 B/s of the 11520 B/s the link carries. Logging is
off while the motor is disabled (`kTelemetryWhileDisabled = false`) so the line stays free
for the one-shot reports the operator needs.

## Safety notes

1. **Opening the serial monitor resets the ESP32.** Never open it while the winch is
   spinning. A reset drops the gate driver and the drum free-wheels.
2. **Stop with `S`. Never with `W 0`** — `W 0` in velocity mode is **active braking**,
   which keeps torque on the phases.
3. **There is no holding brake.** On power-down the drum coasts.
4. `init()` and `C` energise the phases for alignment. Keep the 12 V supply
   **current-limited** while calibrating (alignment bypasses the current loop:
   0.3 V lands as ~1.61 A stall on the KV90 winding). Bench practice from the
   [ARTICLE](https://note.com/kawashigi_/n/n1eaf1e362ebc) (24 V rig, same board):
   PSU CC low, a ~2 A fuse in series with the rail, and a polarity check before
   connecting — the board has **no reverse-polarity protection and no OCP**,
   the PSU CC is the only current limit.
5. `L` only bounds the **torque** mode. The velocity loop is bounded by `velocity_limit`,
   the position loop by `velocity_limit` too. `L` clamps to 4 A and `Z` clamps to
   `voltage_limit` (1.0 V), so neither command can reach the 12 V / 6 A envelope.

## Implementation notes worth knowing

- **`init()` does not align.** SimpleFOC 2.2.1 `BLDCMotor::init()` (`BLDCMotor.cpp:26-65`)
  only bakes limits into the PIDs and enables the driver. `initFOC()` is a separate call and
  is what runs the alignment when `zero_electric_angle` is `NOT_SET`
  (`BLDCMotor.cpp:101`). `begin()` therefore calls both.
- **There is no `ctl_mode` / `ctrl_mode` in 2.2.1.** The field is named `controller`
  (`MotionControlType`, `common/base_classes/FOCMotor.h:180`) and the torque inner loop is
  `torque_controller` (`TorqueControlType`, same header). Any sketch that writes
  `motor.ctl_mode` is from a different SimpleFOC version.
- **There is no `resetVelPid()`.** `PIDController` keeps `integral_prev` / `error_prev` /
  `output_prev` / `timestamp_prev` in `protected` members (`common/pid.h:33-37`), so the
  wind-up integral cannot be zeroed from outside. `FocDrive::resetVelocityPid()` rebuilds a
  fresh `PIDController` from `foc_config.h`, reusing the ramp and limit the motor already had.
- **`shaft_velocity` is the field, not `shaftVelocity()`.** `shaftVelocity()`
  (`common/base_classes/FOCMotor.cpp:62`) is differential-based, so calling it twice inside
  one loop returns ~0. `move()` already refreshes `shaft_velocity`, so the telemetry reads
  the field.
- **No `analogReadResolution(12)`.** It is ineffective on core 2.0.17 with SimpleFOC 2.2.1
  and only breaks things. Not called anywhere in this project.
- **No `cs.linkDriver()`.** It does not exist in 2.2.1; `cs.init()` followed by
  `motor.linkCurrentSense(&cs)` is the whole sequence.
- **`iq_a` / `id_a` read 0.000 in sensorless mode by design.** `poll()` deliberately skips
  `motor_.loopFOC()` for `velocity_openloop` / `angle_openloop`, so `motor_.current` is never
  refreshed. A zero current column is a telemetry artifact, not a stall.
- **Limit changes come in pairs.** `BLDCMotor::init()` copies `current_limit` into
  `PID_velocity.limit` for non-voltage torque control (`BLDCMotor.cpp:53-58`). Writing only
  `current_limit` leaves the velocity loop on the old ceiling until the next `init()`.

## File map

| File | Responsibility |
|---|---|
| `include/foc_config.h` | Every tunable value with provenance. Nothing else defines any of them. |
| `include/foc_protocol.h`, `src/foc_protocol.cpp` | Command grammar + CSV row formatting. Arduino-free so the native test can link it. |
| `include/foc_board.h`, `src/foc_board.cpp` | I2C bus, AS5600 fast-mode register write, VIN sense, undervoltage gate. |
| `include/foc_drive.h`, `src/foc_drive.cpp` | Driver / current sense / sensor / motor wiring, limits, calibration, mode moves. |
| `include/foc_console.h`, `src/foc_console.cpp` | Line parsing, command dispatch, 100 Hz telemetry. |
| `src/main.cpp` | `setup()` / `loop()` and the wiring order only. |
| `test/test_foc_protocol/test_main.cpp` | Native unit test for the protocol layer. |

Dependency direction is one-way:
`main → console → drive → board`, with `protocol` free-standing below all of them.

## 未確認 / 要確認

Values that no primary source confirmed. They are in `foc_config.h` but must be
bench-checked before the firmware is trusted with load.

| Value | In config as | Why it is unconfirmed |
|---|---|---|
| Undervoltage threshold | `kUndervoltageVolts = 10.0f` | The official 11.1 V is a 3S LiPo cutoff with no sag margin on a bench PSU. 10.0 V is a reasoned pick, **not** measured. **✗ 未検証**: no loaded 12 V bench measurement yet. |
| I2C bus index | `kI2cBusIndex = 1` | The official M1 example uses `TwoWire(1)` (verified against the V2.0 branch `3_dual_as5600_test`). **✗ 未検証**: bench check pending (AS5600 now in use since 2026-11-09: confirm boot CONF ACK + `C` align; magnet must be diametral on the shaft end, 0.5–3 mm gap per [ARTICLE](https://note.com/kawashigi_/n/n1eaf1e362ebc); watch total I2C pull-up strength at 400 kHz). |
| Motor pole pairs | `kPolePairs = 20` | Secondary sources only (ODrive community 36N40P reports + working 8308 90KV config). **★要確認★**: count the rotor magnets (/2). **✗ 未検証**. |
| Motor KV / R | `kMotorKv = 90`, `kPhaseResistanceVendorOhms = 0.186` | Vendor reseller spec (KV90 user-confirmed 2026-11-09), R phase convention unpublished. **★要確認★**: bell marking + winding-meter R + no-load KV check. **✗ 未検証**. |
| Closed-loop gains | `kPidCurrent*`, `kPidVel*`, `kPidAngleP` | ARTICLE (5010) values reused as a starting point (official V2.0 example 11 uses vel P=0.021/I=0.12 for its 2808: try if ARTICLE P=0.2/I=2.0 oscillates). LA8308 KV90 retune pending. **✗ 未検証**. |
| `zero_elec` healthy window | `kZeroElecWindowCalibrated = false` | The 3.12–3.42 rad ARTICLE window was for the 5010 and is retired. Bench data must set the LA8308 window. Only used for the boot note; the firmware never clamps on it. |

## Deliberately not implemented

- **Tension (N) and wire length.** The article derives 6.71 N/A from the motor's rated
  torque. That factor is **not** calibrated against real weights, so it is not in the code.
  Converting `T <torque>` into newtons requires a measured constant for the actual winch.
- **EEPROM persistence of `zero_elec`.** Calibration is redone every boot, which is why the
  boot log prints it.
- **MSTE binary protocol / ROS bridge / web UI.** This project shares no code with them.
- **`6.71 N/A`** — see above.

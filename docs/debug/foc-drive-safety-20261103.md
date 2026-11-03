# foc_drive safety hardening (LA8308 KV90)

> Date: 2026-11-03
> Target: `firmware/foc_drive/` (3 axle roles × M0/M1)
> Not touched: `firmware/foc/` (5010 winch)
> Motor: Eaglepower LA8308 KV90 (user). Vendor spec used for sanity limits only:
> 22 A continuous, 900 W, 6S–12S, 0.186 Ω, 0.6 A idle at 24 V. Not a primary datasheet.

## Why this change

The bench firmware could energise the motor outside a safe envelope, and the shared
GPIO12 ENABLE made single-channel failures affect both channels.

| # | Hazard | Evidence |
|---|---|---|
| 1 | `Z <volts>` accepted any positive value; alignment could apply 24 V | `foc_console.cpp` only checked `>0`; SimpleFOC clamps `voltage_sensor_align` only to `motor.voltage_limit` (24 V) in `init()` |
| 2 | `L <amps>` accepted any positive value; the current PID voltage limit is 24 V | same console check; `BLDCMotor::init()` sets `PID_current_q.limit = voltage_limit` |
| 3 | No closed-loop overcurrent check | `poll()` checked current only in the open-loop branch |
| 4 | M0/M1 share ENABLE but stop was per channel | `kM0Pins/kM1Pins.enable = 12`; `stop()` on one dropped both gates while the sibling stayed `enabled` |
| 5 | Every deadman resend called `resetVelocityPid()` and `motor_.enable()` | `enable()` zeroes PWM (`BLDCDriver3PWM.cpp`); the velocity PI was reset at ≥5 Hz |
| 6 | Every `O` resend reset `openloop_t0ms_` | 10 s open-loop timeout could be bypassed indefinitely |
| 7 | `C`/`Z` could run while the sibling was energised, and `Z` did not actually re-align | `setAlignVoltage()` did not clear `zero_electric_angle`/`sensor_direction`, so `initFOC()` skipped alignment |
| 8 | Boot ran `initFOC()` automatically when the sensor was present | wheels move at power-up |
| 9 | AS5600 NACK was not checked; a failed read returns 0xFFF | `MagneticSensorI2C::read()` ignores `requestFrom` |
| 10 | A main-loop hang left the last PWM applied | no independent gate cutout; the deadman is in the same loop |
| 11 | Pole pairs were the 5010 placeholder 7 | `alignSensor()` only prints a PP mismatch; it still returns success, so `initFOC()==1` is not evidence |
| 12 | `V` max 1.0 V was above the 4 A open-loop trip | 1.0 V / 0.186 Ω ≈ 5.4 A |
| 13 | Motion targets were unbounded; cooldown used a wrap-unsafe `millis() <`; `D` bypassed the cooldown | `O 1e30`, `W 1e30`, `A 1e30`; 49.7-day wrap |

## Fixes

- New `include/foc_limits.h` (Arduino-free, native-tested): current 3.0 A default / 4.0 A
  max; align 0.10–0.60 V; open-loop 0.10–0.60 V; O/W ≤5 rev/s; angle ≤±100 rad; P 1–50;
  wrap-safe `cooldownActive()` / `timeoutElapsed()`.
- `foc_config.h`: split boot (20 V) and runtime (19 V/500 ms) undervoltage; added
  open/closed overcurrent trips (4 A/500 ms, 5 A/200 ms), ADC full-scale trip, heartbeat
  guard 500 ms, current-sense plausibility thresholds, `kConfirmedPolePairs`,
  `kAutoAlignOnBoot=false`, `kOneChannelAtATime=true`.
- `FocDrive`: two-phase bring-up (`beginDrivers` for both boards, then `beginMotor`) so the
  shared gate is never raised before both PWM stages are configured; fault latch with
  `emergencyStop()`/`clearFault()`; `ready_ = initFOC success && P confirmed`; `Z` clears
  the alignment fields and re-aligns; `P` stops the channel; `L` is capped and re-clamps a
  torque target.
- Motion: deadman resends only update the target; `enable()` and PID reset happen only on
  a mode transition. `O` resends do not extend the timeout. All targets are clamped.
  `N`/`G` removed (`O` is the sensorine check).
- Safety loop: overcurrent in all modes, ADC saturation trip, low-speed `V/R` vs measured
  current check, runtime VIN check, AS5600 health check, heartbeat guard.
- Console: one channel at a time; stop both before `C/Z/D/P`; stop both on fault; `S`
  clears faults. The guard is disarmed before any blocking `initFOC`.
- `PrefixedPrint` emits `#SF ` at the start of every SimpleFOC monitor line, preserving the
  `#` one-shot / CSV contract.
- `foc_guard`: 10 ms timer ISR drops the shared ENABLE via a direct GPIO register write
  after 500 ms without a feed.

## Pole pairs (resolved)

The vendor pages do not publish it. Three independent secondary sources for the Eaglepower
8308 90KV agree on **20 pole pairs / 40 magnets**:

- ODrive Community, "Callibration issue when motor magnets aren't alligned with coils":
  "L8308 90Kv eaglepower BLDC. it has 40 magnets (so 20 pole pairs)."
  https://discourse.odriverobotics.com/t/callibration-issue-when-motor-magnets-arent-alligned-with-coils/8806
- ODrive Community, "Odrive S1 motor calibration error": "EaglePower 8308 motor
  (20 pole pairs/ 40 magnets, 22A max continuous current)."
  https://discourse.odriverobotics.com/t/odrive-s1-motor-calibration-error/11326
- Working ODrive config for the same motor: `motor.config.pole_pairs = 20`.
  https://github.com/AtomsClark/Integrated-Cycloidal-Robotic-Actuator/blob/main/firmware/odrivetool-session.md

`kConfirmedPolePairs = 20` and `kDefaultPolePairs = 20` are set from this. The physical
magnet count on the user's unit is still the final check; if it differs, `P <magnets/2>`
and re-run `C`.

## Review blockers found after the first implementation

1. `prepareMotion()` stopped the sibling unconditionally. Because `stop()` drives the
   shared ENABLE low even for an already-disabled channel, a deadman resend could find the
   selected channel still `enabled`, skip `motor_.enable()`, and run with the gate low.
   Fixed: stop both only when the sibling is actually enabled.
2. `stopBoth()` did not disarm the heartbeat guard. `C`/`Z`/`D` then blocked in `initFOC`
   for seconds while the guard was armed; the ISR could cut alignment mid-way and leave a
   bogus `zero_electric_angle` that still returned success. Fixed: `stopBoth()` disarms
   before the blocking command.

## Second review (final HEAD)

The code-review pass found five more issues; all fixed:

1. A mode switch (`W`↔`T`↔`A`) that hit an AS5600 NACK set `ready_=false` and returned
   without disabling the already-running drive. `prepareClosedLoop()` now calls
   `emergencyStop()` if the motor is enabled.
2. `D` blocked `poll()` for ~200 ms, so its 4 A/full-scale trips could not fire.
   The sample loop now checks every sample and aborts with a latched fault on >=4 A or
   >=0.98×full scale.
3. `L` accepted 0.05 A although help/README say 0.1 A minimum. Added
   `foc::kCurrentLimitMin=0.1f` and a native test.
4. `stop()` re-armed the 10 s open-loop cooldown from the controller enum alone, so `D`
   (which leaves `velocity_openloop` selected while disabled) made the next `D` wait 10 s.
   The cooldown is now armed only when `motor_.enabled && openLoopActive()`.
5. `?` printed bare lines with commas, violating the `#` one-shot / CSV contract.
   Every help line is now `#HELP ...`.

## Verification

- TDD red: the new native tests failed before the implementation (9 assertion failures:
  limits and `N`/`G` parsing).
- `~/.local/bin/pio test -e native` — 15/15 PASS (limits + protocol).
- `~/.local/bin/pio run -e foc_front`, `-e foc_mid`, `-e foc_rear` — see commit/CI output.
- Hardware: **✗ 未検証**. No `/dev/ttyUSB*` was present. Not measured: shunt marking,
  LA8308 pole pairs, winding resistance (0.186 Ω is a vendor figure), loaded VIN
  thresholds, continuous current, spin direction, actual `#IOL`.

## Follow-ups

- Count the LA8308 magnets on the actual unit (expected 40 -> 20 pairs); adjust `P` if different.
- Fit AS5600 and read the `#SF MOT: PP check` estimate during `C`.
- Measure R and the shunt marking; tighten/loosen `kPhaseResistanceOhms` accordingly.
- Add a dual-channel gate coordinator before setting `kOneChannelAtATime=false`.
- Persist the calibration in NVS so 6 motors do not realign on every boot.

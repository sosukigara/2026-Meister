# FOC drive — 3 axle roles × 2 channels (MKS ESP32 FOC V2.0)

One source, three binaries for the 6-wheel rover: **FRONT / MID / REAR**. Each board drives
the two wheels of one axle (M0 = left, M1 = right). This is a sibling of
[`../foc/`](../foc/README.md) (the single-channel winch) and deliberately shares no code
with it, so the winch's verified behaviour is untouched.

**Status: bench bring-up. AS5600 is not fitted yet, so only the sensorless open-loop
commands are usable right now.** The closed-loop path is compiled but gated until `C`
succeeds. Pole pairs are pre-set to 20 (40 magnets). Boot alignment is off: `C` is an
explicit command.

## What is known / not known

| Item | Value | Status |
|---|---|---|
| Board | MKS ESP32 FOC V2.0, 2ch, 12–24 V | 一次情報: official repo / schematic |
| Motor | LA8308 KV90 | user |
| Motor ratings | 22 A continuous, 900 W, 6S–12S, 0.186 Ω, 24 V idle 0.6 A | **販売店スペック（一次資料ではない）** [OL Electronics](https://www.olelectronics.com/product/eaglepower-la-8308-kv90-brushless-motor) |
| Board current | 12 A total peak, ~6 A/ch with cooling | 販売店記載。連続 6 A×2 は **✗ 未検証** |
| Firmware current cap | 4.0 A hard, 3.0 A default | ADC 満尺 6.6 A に対する安全余裕。6 A は 91% で不可 |
| Current sense | INA240A2, 10 mΩ, gain 50 | 一次情報: official example / schematic |
| Shunt marking | `R010` (10 mΩ) expected | **★要確認★** 実機の刻印。`R001` なら全読みが 10 倍ずれる |
| ENABLE | M0/M1 share GPIO12 | 一次情報: official example。片 ch だけ disable 不可 |
| Pole pairs | **20 (40 magnets)** | 二次情報: ODrive コミュニティ 2 件 + 稼働中の ODrive 設定。実機の磁石数は要確認 |
| AS5600 | not fitted yet | 一時的。付いたら I2C 0/1 で自動検出 |
| Hall | board supports hall; LA8308 hall wires unknown | **★要確認★** official `8_hallsensor_test` あり |

## Firmware safety envelope (v0.2)

- `L <amps>`: 0.1–4.0 A, default 3.0 A. Closed-loop only.
- `Z <volts>`: 0.10–0.60 V, default 0.30 V. Alignment voltage is clamped at the command.
- `V <volts>`: 0.10–0.60 V, default 0.30 V. 0.30 V is ~1.6 A at 0.186 Ω.
- `O` / `W`: |rev/s| ≤ 5 (bench cap; set the real value from wheel diameter/gearing).
- Open loop: 10 s run, 10 s cooldown. **Resending `O` does not extend the timer.**
- Closed loop: `W`/`T`/`A` are deadman commands (500 ms).
- Overcurrent: open loop 4 A/500 ms; closed loop 5 A/200 ms; ADC full scale (≈6.5 A)
  trips immediately. This is a backstop, not a control loop.
- Runtime checks while enabled: VIN < 19 V for 500 ms, AS5600 NACK ×3, low-speed
  current-sense plausibility (`V/R` vs measured). Any failure latches `#FAULT`.
- Heartbeat guard: if the main loop stops feeding for 500 ms, a timer ISR drops the shared
  ENABLE pin. An I2C/library hang therefore cannot leave the last PWM applied.
- M0/M1 share ENABLE, so bench operation is **one channel at a time**. Any fault or `S`
  stops both. `kOneChannelAtATime = false` is not supported yet.
- Faults latch. Only `S` clears a fault. Re-motoring a short is not safe.

## Build

```bash
cd firmware/foc_drive
~/.local/bin/pio run -e foc_front -e foc_mid -e foc_rear
~/.local/bin/pio test -e native
```

`pio` may not be on a non-interactive PATH; the absolute path above is the one installed on
this machine.

## Flash (one board at a time)

The three boards are identical, so `/dev/ttyUSB0..2` can reorder. Record each board's
physical path once:

```bash
ls -l /dev/serial/by-path/
```

Fill the commented `upload_port` / `monitor_port` lines in `platformio.ini` for that role,
or pass `--upload-port` explicitly:

```bash
~/.local/bin/pio run -e foc_front -t upload --upload-port /dev/serial/by-path/...
```

Never flash all three with one `pio run -t upload` while they are plugged in together; the
wrong image on the wrong axle is exactly what the `#ROLE` print is meant to catch.

## First bench spin (no AS5600)

1. Wheels off the ground. Current-limited supply, fuse, guard.
2. Flash one board. Open the monitor **before** powering the 24 V rail if possible; opening
   it resets the ESP32.
3. Confirm the boot log: `#ROLE`, `#VIN` ~24 V, `#M0 SHUNT 0.010`,
   `#M0 POLE_PAIRS ... UNCONFIRMED`, `#SAFE`.
4. Pole pairs are pre-set to **20 (40 magnets)** from three consistent ODrive sources.
   Count the actual rotor magnets (÷2); if they differ, `P <magnets / 2>`. Open loop still
   turns with a wrong P, but the achieved speed scales with `P_set / P_true`.
5. Start at 0.30 V: `O 1`. Watch `#IOL0` (expected ~1.6 A at 0.186 Ω, vendor figure).
6. Stop with `S` (stops **both** channels). The open loop also self-stops after 10 s and
   refuses a restart for 10 s. Resending `O` does not extend a run.
7. If it does not turn, lower `O` to 0.5, or raise `V` in 0.05 V steps to **0.60 V max**.
   If `#OC` / `#CS_SUSPECT` appears, stop and check the R010 marking and wiring; do not
   raise the trip to make it spin.

There is no current loop in open loop. 3 A is the closed-loop default; the open-loop
protection is the voltage ceiling, the 4 A `#OC` trip, the 10 s timeout and the supply CC.

## Commands

| Command | Effect |
|---|---|
| `M 0` / `M 1` | Select M0 (left) / M1 (right) for the following commands |
| `O <rev/s>` | Sensorless open-loop velocity, \|rev/s\|≤5 (bench only) |
| `V <volts>` | Open-loop voltage, 0.10–0.60 V (default 0.30 V) |
| `P <pairs>` | Set pole pairs 1–50 for the selected channel; pre-set to 20 (40 magnets) |
| `C` | Recalibrate the selected channel (needs P + AS5600) |
| `L <amps>` | Closed-loop current limit, 0.1–4.0 A (default 3.0 A) |
| `W <rev/s>` | Closed-loop velocity, \|rev/s\|≤5 (`W 0` is active braking, not stop) |
| `T <torque>` / `A <rad>` | Closed-loop torque / angle (\|rad\|≤100) |
| `Z <volts>` | Alignment voltage, 0.10–0.60 V; re-runs alignment |
| `D` | Current-sense/winding diagnosis at the open-loop voltage |
| `S` | **STOP both channels + clear a latched fault** |
| `?` | Help |

`N` (sine) and `G` (angle open loop) were removed; `O` covers the sensorless spin check.

Telemetry is 50 Hz, two CSV rows per board:

```
#t_s,ch,mech_rad,vel_rps,iq_a,mode,id_a,vq_v,vd_v
```

`ch` is 0 (M0) or 1 (M1). `#IOL0` / `#IOL1` print the measured phase current at 2 Hz while
an open loop runs. SimpleFOC's `MOT:` diagnostics are prefixed `#SF` so they never appear
as bare lines in the CSV stream.

## Safety notes

1. **Open loop has no current control.** Current is `(V - back-EMF) / R`; raising `V`
   raises current and heat together. 0.60 V is the hard ceiling (~3.2 A at 0.186 Ω).
2. **`#OC` is a backstop, not a control loop.** If the shunt is actually 1 mΩ, the reading
   is 10× low; check the `R010` marking before trusting any current number.
3. **M0/M1 share ENABLE.** One channel's disable drops both gates. The bench runs one
   channel at a time; a fault stops the axle, not one wheel.
4. **No holding brake.** `S` disables the driver; the wheel coasts.
5. **Closed-loop commands are a deadman.** `W`/`T`/`A` must be resent at ≥5 Hz. If the board
   sees no motion command for 500 ms it disables the driver and prints `CMD_TIMEOUT`. The
   PC must also stop all three boards when any link drops.
6. **AS5600 still gates the closed loop.** `C/W/T/A` refuse until `C` succeeds. `P` is
   pre-set to 20; SimpleFOC's PP check only prints, it does not fail `initFOC()`.
7. **Opening the serial monitor resets the ESP32.** Never do it while spinning.
8. **The 24 V rail is shared by 3 boards.** Fuse and wire for the peak; do not assume
   continuous 6 A×2 without a thermal test.
9. **The heartbeat guard is not a substitute for an E-stop.** A hard E-stop must cut the
   24 V contactor.

## Relationship to the other firmware

| Project | Role |
|---|---|
| `firmware/main_board/` | MSTE binary protocol: steering servos, arm, gripper; no FOC |
| `firmware/foc/` | single-channel M1 winch, sensorless bring-up, text console |
| `firmware/foc_drive/` | this project: 3 axle roles, M0+M1, bench open loop now, AS5600 closed loop later |

`firmware/foc` is intentionally not modified. The two projects pin the same
`espressif32@6.9.0`; building them against the same `~/.platformio` still has the
`tool-esptoolpy` reinstall quirk documented in `../foc/README.md`. Use a separate
`PLATFORMIO_CORE_DIR` if that becomes painful.

## Verification status

- Native unit tests: `pio test -e native` — 15/15 PASS (limits + protocol).
- Firmware builds: `foc_front` / `foc_mid` / `foc_rear` — see the commit message / CI run.
- Hardware spin, `#IOL`, shunt marking, physical magnet count, AS5600 alignment: **✗ 未検証**.
  No board was connected (`/dev/ttyUSB*` absent) during this change.

## Next steps

- [ ] Verify the shunt marking (`R010`) and count the LA8308 magnets (expected 40).
- [ ] One wheel spins at 0.30 V; record `#IOL`, temperature, and direction.
- [ ] Fit AS5600, set P, run `C` at 0.30 V. If it fails, `Z 0.40` and retry; do not jump
      to 1.0 V on this low-resistance motor.
- [ ] Then `L 1.0`, `W 0.5` and raise slowly; `W`/`T`/`A` are deadman commands.
- [ ] Set per-role `kInvertM0/M1` from the direction test.
- [ ] Three boards: verify `#ROLE` against the by-path map before arming.
- [ ] PC side: one link per board, watchdog, all-stop on any link loss.

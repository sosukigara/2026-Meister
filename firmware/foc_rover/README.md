# FOC drive — 3 axle roles × 2 channels (MKS ESP32 FOC V2.0)

One source, three binaries for the 6-wheel rover: **FRONT / MID / REAR**. Each board drives
the two wheels of one axle (M0 = left, M1 = right). This is a sibling of [`../foc_test/`](../foc_test/README.md) (spin-test-only rig)
and deliberately shares no code with it, so bench experiments cannot leak
into the rover build.

**Status: bench bring-up. AS5600 is now in use (user 2026-11-09): each channel
auto-detects its sensor at boot via the CONF write ACK.** The closed-loop path is
compiled but gated until `C` succeeds. Pole pairs are pre-set to 20 (40 magnets).
Boot alignment is off: `C` is an explicit command.

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
| AS5600 | in use since 2026-11-09, M0 = bus 0 (SDA19/SCL18), M1 = bus 1 (SDA23/SCL5) | 起動時に CONF 書込 ACK で自動検出。磁石（径方向着磁）を軸端中心に配置し、ギャップはデータシート推奨範囲でベンチ確認 **★要確認★** |
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
- Runtime checks while enabled: VIN < 9 V for 500 ms, AS5600 NACK ×3, low-speed
  current-sense plausibility (`V/R` vs measured). Any failure latches `#FAULT`.
- Heartbeat guard: if the main loop stops feeding for 500 ms, a timer ISR drops the shared
  ENABLE pin. An I2C/library hang therefore cannot leave the last PWM applied.
- M0/M1 share ENABLE, so bench operation is **one channel at a time**. Any fault or `S`
  stops both. `kOneChannelAtATime = false` is not supported yet.
- Faults latch. Only `S` clears a fault. Re-motoring a short is not safe.

## Build

```bash
cd firmware/foc_rover
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
2. Flash one board. Open the monitor **before** powering the 12 V rail if possible; opening
   it resets the ESP32.
3. Confirm the boot log: `#ROLE`, `#VIN` ~12 V, `#M0 SHUNT 0.010`,
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
8. **The 12 V rail is shared by 3 boards (USER envelope: 12 V / 6 A max).** Fuse and wire
   for the peak; do not assume continuous 6 A×2 without a thermal test.
9. **The heartbeat guard is not a substitute for an E-stop.** A hard E-stop must cut the
   12 V contactor.

## Relationship to the other firmware

| Project | Role |
|---|---|
| `firmware/main_board/` | MSTE binary protocol: steering servos, arm, gripper; no FOC |
| `firmware/foc_rover/` | this project: 3 axle roles, M0+M1, bench open loop now, AS5600 closed loop later |
| `firmware/foc_test/` | spin-test only: fixed 0.3 V open loop + align, bounded L/W, 10 s auto-stop |

All FOC projects pin the same `espressif32@6.9.0`. Building them against one
`~/.platformio` can trigger a `tool-esptoolpy` reinstall loop: the first build
after switching projects fails with a `TypeError` (`FRAMEWORK_DIR` is `None`
while the core reinstalls) and the second build succeeds. Use a separate
`PLATFORMIO_CORE_DIR` if that becomes painful.

## 6-wheel rover plan (3 boards × 2 channels)

Moved here 2026-11-22 from the deleted `docs/design/08-rocker-bogie.md`, which was the only
copy. This is a **plan plus a bench implementation**, not a verified configuration: bandwidth
and skew are calculated, not measured.

```
PC (ROS 2)
 ├─ USB ─ FOC #1 (M0/M1) ─ front L / R
 ├─ USB ─ FOC #2 (M0/M1) ─ mid   L / R
 └─ USB ─ FOC #3 (M0/M1) ─ rear  L / R
```

- One board per axle is the natural split. Splitting by left/right would need 3 channels per
  board, which a 2-channel board cannot do.
- M0 and M1 are independent (driver, current sense, I2C bus: M0 bus 0 / SDA 19 / SCL 18,
  M1 bus 1 / SDA 23 / SCL 5). Both AS5600s use 0x36 but sit on different buses, so two per
  board do not collide.
- **ENABLE is shared between M0/M1 on GPIO 12** (the official example passes the same pin to
  both channels). You cannot disable one channel; isolate a failed wheel on the power side.
- The board is discrete AP30H80Q MOSFETs + gate driver. 6 A/ch is a **peak with cooling**
  (12 A peak for 2 ch). Continuous 6 A × 2 needs a thermal test (**★要確認★**).
- Current sense is INA240A2 + 10 mΩ shunt, gain 50 (`InlineCurrentSense(0.01, 50.0, ...)`):
  3.0 V at 6 A, ADC full scale at 6.6 A. Some listings claim 1 mΩ / ±33 A — **check the shunt
  marking**; at 1 mΩ the firmware's 0.01 is 10× off.

### Link budget

- Three USB links are independent: 3 × ~7 kB/s at 50 Hz ≈ 21 kB/s total.
- The bottleneck is each link's 115200 bps (11520 B/s at 8N1):
  - One motor line is ~70 B, so two motors per board is ~140 B. **At 100 Hz that is
    14000 B/s and does not fit.**
  - 50 Hz fits (7000 B/s, 61%). Keeping 100 Hz needs 460800 (15%). 921600 depends on the
    USB-UART bridge and cable: **✗ 未検証**.
  - Packing both motors into one line is still ~130 B, so it does not fix 100 Hz either.
- The three links share no clock. Write order gives a few ms of skew: under 1 mm at 0.3 m/s.
  One stalled write/flush does delay the others, so use one thread per link or non-blocking I/O.

### Decide before driving or closing the loop

| # | Item | Status |
|---|---|---|
| 1 | Wheel motor, gear ratio, wheel diameter | ★要確認★ `foc_config.h` is LA8308 KV90 (20 pole pairs = 40 magnets, bench limit 5 rev/s); gear ratio and wheel diameter are undetermined |
| 2 | AS5600 on all 6 wheels | ★要確認★ without it there is no closed-loop speed control or odometry |
| 3 | Drive type (skid steer / all-wheel steering) | ★要確認★ changes how commands are built |
| 4 | Roles of the 3 boards and how USB is pinned | ★要確認★ see "USB and enumeration" |
| 5 | All three boards MKS ESP32 FOC V2.0 | ★要確認★ |

### Safety (without this you cannot say "it is fine")

- **Board-side command watchdog**: ramp to 0 within 200 ms of the last valid command and
  disable at 500 ms. The PC-side `cmd_timeout` (0.5 s) alone leaves the **last velocity
  applied** if USB drops or the PC dies.
- **If any one link drops, stop all three.** One side spinning alone turns the robot.
- **Hard E-stop** that cuts the 24 V via a contactor. The firmware stop is supplementary.
- Start disabled. Opening a USB port can reset the ESP32; never open one while spinning.
- Measure continuous current and board temperature for 6 wheels (AP30H80Q, 6 A/ch is
  cooling-dependent): **✗ 未検証**.

### USB and enumeration (you will hit this with 3 boards)

- `/dev/ttyUSB0..2` order changes across reboots and replugs. If the USB-UART bridge has no
  unique serial, `by-id` collides → pin with **`/dev/serial/by-path`** (hub physical port) and
  label boards and cables front/mid/rear.
- Put the role in the board's NVS and report `#ID` at boot so a mis-plug is detectable.
  Compile-time role means 3 builds and no detection on swap.
- `pio run -t upload` with 3 boards attached can write the wrong one. Flash one at a time or
  name `/dev/serial/by-path` explicitly (same reason as the M5Stack misflash note in
  `main_board`).
- Linux `ttyUSB` `latency_timer` defaults to 16 ms. Set it to 1 ms on all 3 ports; the current
  `serial_link.cpp` does not.
- Reading 3 links in one thread lets one timeout stall the others: use one thread per link.

### Power and ground

- Three boards + 6 motors from the 24 V bus, with a fuse and bulk capacitor per board.
- Do not create two paths between PC USB GND and PSU GND: star ground. PWM current on the USB
  GND shows up as disconnects and resets.
- Whether USB 5 V and board 5 V back-feed each other is board-dependent: **★要確認★**.
- Motor inrush and regen make the 24 V bus spike: measure VIN rating and the undervoltage gate.

### Verification order (gate each step)

1. One board, one motor: `O 1` (0.30 V) → `S`, watch current and heat. After the AS5600
   arrives: `C` → `W ±`.
2. One board, both motors: I2C and current interaction, back-EMF.
3. Three boards: enumeration reproduces over 10 replugs, Hz per link, all stop when one is
   unplugged.
4. Rover lifted, 6 wheels: command direction and speed agree, E-stop.
5. On the ground for 10 continuous minutes: current, temperature, 24 V sag.

### PC side

- `umeonigiri_serial_bridge` assumes a single `serial_port`. MSTE (steering, arm) + 3 FOC
  boards is **4 links**, so splitting `/cmd_vel` into a steering path and a wheel-speed path
  needs its own node.
- The current `kinematics.cpp` is a bicycle model that sends the same speed to all 6 channels.
  Turning 6 wheels needs an inner/outer difference; closed-loop FOC can deliver it per wheel.
  Which model to use waits on the drive-type decision (item 3 above).

## Verification status

- Native unit tests: `pio test -e native` — 15/15 PASS (limits + protocol).
- Firmware builds: `foc_front` / `foc_mid` / `foc_rear` — see the commit message / CI run.
- Hardware spin, `#IOL`, shunt marking, physical magnet count, AS5600 alignment: **✗ 未検証**.
  No board was connected (`/dev/ttyUSB*` absent) during this change.

## Next steps

- [ ] Verify the shunt marking (`R010`) and count the LA8308 magnets (expected 40).
- [ ] One wheel spins at 0.30 V; record `#IOL`, temperature, and direction.
- [ ] Mount the AS5600 diametral magnet centered on each shaft end (gap per datasheet),
      confirm `#ROLE`/`#VIN`/`#SHUNT` at boot with no `#ERR AS5600`, then `P 20`,
      `C` at 0.30 V. If it fails, `Z 0.40` and retry; do not jump
      to 1.0 V on this low-resistance motor.
- [ ] Then `L 1.0`, `W 0.5` and raise slowly; `W`/`T`/`A` are deadman commands.
- [ ] Set per-role `kInvertM0/M1` from the direction test.
- [ ] Three boards: verify `#ROLE` against the by-path map before arming.
- [ ] PC side: one link per board, watchdog, all-stop on any link loss.

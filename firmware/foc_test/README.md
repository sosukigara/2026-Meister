# foc_test — spin-test-only firmware (MKS ESP32 FOC V2.0, M1)

One motor, fixed safe envelope, nothing to misconfigure. Bench use only —
production is [`../foc_winch/`](../foc_winch/README.md). This project shares no
code with it on purpose: a test rig must keep working even while production
evolves.

## Envelope (bounded commands, fixed voltages)

| Item | Value | Sizing |
|---|---|---|
| Supply | 12 V, 6 A max bench | USER envelope; VIN gate 10 V, sag gate 9 V/500 ms while running |
| Open loop `O` | fixed 0.3 V → **2.13 A measured** (2026-11-09) | 0.3 V / 0.14 Ω; 0.186 Ω is line-to-line |
| Align `C` | fixed 0.3 V | same as above |
| Closed loop `W` | `L` 0.1–3.0 A (default 1.0), 12 V ceiling, `W` capped at 14 rps | **ceiling measured 2026-11-09: ~830 rpm (87 rad/s) no-load**; 16 and 18 rps commands both sat there, so the 12 V rail — not the cap — is the limit |
| Overcurrent | open loop 4 A/500 ms, closed loop 5 A/200 ms, ADC full scale trips after **10 ms** | hard-fault backstops; the full-scale guard needs persistence because a switching transient rails both shunts at once |
| Run cap | **every motion auto-stops after 10 s**; open loop then cools down 10 s | test-only rule |
| Motion guards | `O`/`C` need a stopped motor; `W` retargets a live session; accel ramp 30 rad/s²; AS5600 loss (2×NACK) stops a run | mid-run faults cannot hide behind a frozen angle |

Motor: LA8308 KV90, 20 pole pairs (36N40P — count the magnets ÷2 to confirm).
Sensor: AS5600 on bus 1 (SDA23/SCL5, 0x36, fast mode set every boot).
`O` works without a sensor; `C`/`W` need it.

## Wiring

Motor 3-phase to the M1 output. AS5600: diametral magnet centered on the shaft
end, 0.5–3 mm gap, module on bus 1. 12 V supply current-limited with a fuse in
series — the board has no reverse-polarity protection. Keep the wheel free:
boot aligns the motor when a sensor answers.

## Use

```bash
cd firmware/foc_test
~/.local/bin/pio run -e foc_test_m1 -t upload --upload-port /dev/serial/by-id/<board>
~/.local/bin/pio device monitor -b 115200
```

```
O       open-loop spin at 1 rps (arg optional, max 2 rps)
C       align with AS5600, prints #ZERO_ELEC
L [amps] current ceiling 0.1-3.0 A (default 1.0)
W <rps> closed-loop velocity in rps (needs C, max 14 rps)
I       probe phase currents once (wiring/shunt check)
S       stop. ? shows this list. Every motion auto-stops after 10 s.
```

If `C` prints `#CAL` but `W` stays gated, rebuild with
`PLATFORMIO_BUILD_FLAGS="-DFOC_TEST_MONITOR=1"` to see SimpleFOC's own `MOT:`
trace. `MOT: Align error!` means the phase-current 2:1 split test failed, and
`I` then shows which phase is not carrying its share: with all three phases
connected, driving one must read ~2x the returning one. A ratio near 1 or a
much weaker third phase is a wiring fault, not a setting.

High-speed run-up: `C`, then `L 1.0`, `W 1`, `W 3`, `W 5`, `W 8`, `W 12`,
watching `#SPD` (rad/s) and supply current at each step. Each `W` stops
itself after 10 s. Above `W 12` the speed stops rising (~87 rad/s on 12 V):
that is the rail, so a faster spin needs a higher supply, not a bigger number.
If `#OC`/`#OC_SAT` trips, stop: check the magnet gap, phase wiring, and the
shunt marking before retrying.

Expected first spin: `O` turns the free motor. `I` then shows ~2.1 A on the
driven phase splitting ~2:1 into the returning one; a ratio near 1 means a phase
wire is not landed and SimpleFOC will refuse closed loop (`MOT: Align error!`).
If `C` fails, check the magnet gap/direction and the SDA/SCL wiring —
not the voltages (they are fixed).

## Deliberately missing

`L`/`Z` configuration, CSV telemetry, deadman commands, multi-channel support,
native unit tests (the whole sketch needs Arduino + SimpleFOC; the bench spin
is the test). Anything beyond a spin check belongs in `foc_winch`/`foc_rover`.

# foc_test — spin-test-only firmware (MKS ESP32 FOC V2.0, M1)

One motor, fixed safe envelope, nothing to misconfigure. Bench use only —
production is [`../foc_winch/`](../foc_winch/README.md). This project shares no
code with it on purpose: a test rig must keep working even while production
evolves.

## Envelope (bounded commands, fixed voltages)

| Item | Value | Sizing |
|---|---|---|
| Supply | 12 V, 6 A max bench | USER envelope; VIN gate 10 V, sag gate 9 V/500 ms while running |
| Open loop `O` | fixed 0.3 V → 1.61 A stall | 0.3/0.186 Ω |
| Align `C` | fixed 0.3 V | same as above |
| Closed loop `W` | `L` 0.1–3.0 A (default 1.0), 12 V ceiling, max 10 rps (~600 rpm) | current loop bounds torque; BEMF at 10 rps ~6.7 V so 12 V leaves margin |
| Overcurrent | open loop 4 A/500 ms, closed loop 5 A/200 ms, ADC full scale trips now | hard-fault backstops |
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
W <rps> closed-loop velocity in rps (needs C, max 10 rps)
S       stop. ? shows this list. Every motion auto-stops after 10 s.
```

High-speed run-up: `C`, then `L 1.0`, `W 1`, `W 3`, `W 5`, `W 8`, `W 10`,
watching `#SPD` (rad/s) and supply current at each step. Each `W` stops
itself after 10 s. If `#OC`/`#OC_SAT` trips, stop: check the magnet gap,
phase wiring, and the shunt marking before retrying.

Expected first spin: `O` turns the free motor, telemetry shows `#IOL` ≈ 1.6 A
or less. If `C` fails, check the magnet gap/direction and the SDA/SCL wiring —
not the voltages (they are fixed).

## Deliberately missing

`L`/`Z` configuration, CSV telemetry, deadman commands, multi-channel support,
native unit tests (the whole sketch needs Arduino + SimpleFOC; the bench spin
is the test). Anything beyond a spin check belongs in `foc_winch`/`foc_rover`.

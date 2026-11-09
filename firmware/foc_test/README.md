# foc_test — spin-test-only firmware (MKS ESP32 FOC V2.0, M1)

One motor, fixed safe envelope, nothing to misconfigure. Bench use only —
production is [`../foc_winch/`](../foc_winch/README.md). This project shares no
code with it on purpose: a test rig must keep working even while production
evolves.

## Fixed envelope (no L/Z commands exist)

| Item | Value | Sizing |
|---|---|---|
| Supply | 12 V, 6 A max bench | USER envelope; VIN gate 10 V |
| Open loop `O` | fixed 0.3 V → 1.61 A stall | 0.3/0.186 Ω |
| Align `C` | fixed 0.3 V | same as above |
| Closed loop `W` | fixed 1.0 A, 1.0 V ceiling → 5.38 A stall max | under 6 A and 6.6 A ADC full scale |
| Overcurrent | open loop 4 A/500 ms, closed loop 5 A/200 ms | hard-fault backstop |
| Run cap | **every motion auto-stops after 10 s**; open loop then cools down 10 s | test-only rule |

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
W 0.5   closed-loop velocity in rps (needs C, max 0.8 rps)
S       stop. ? shows this list.
```

Expected first spin: `O` turns the free motor, telemetry shows `#IOL` ≈ 1.6 A
or less. If `C` fails, check the magnet gap/direction and the SDA/SCL wiring —
not the voltages (they are fixed).

## Deliberately missing

`L`/`Z` configuration, CSV telemetry, deadman commands, multi-channel support,
native unit tests (the whole sketch needs Arduino + SimpleFOC; the bench spin
is the test). Anything beyond a spin check belongs in `foc_winch`/`foc_rover`.

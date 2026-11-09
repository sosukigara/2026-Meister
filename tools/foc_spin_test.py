#!/usr/bin/env python3
"""Bench spin test driver for firmware/foc_test over serial.

Sends the bring-up sequence and logs every line the board prints, so the
operator watches #IOL/#SPD/supply current instead of typing commands.
The firmware itself enforces the envelope (10 s auto-stop, OC trips);
this script only paces the commands.

Usage:
    python3 tools/foc_spin_test.py --port /dev/serial/by-id/<board>
    python3 tools/foc_spin_test.py --port /dev/ttyUSB0 --hold 6 --ramp 1,3,5,8,10

Requires pyserial (pip install pyserial). Board must already run foc_test.
"""

from __future__ import annotations

import argparse
import sys
import time

import serial


def expect(ser: serial.Serial, token: str, timeout_s: float) -> bool:
    """Drain lines until one contains token or the timeout elapses."""
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        line = ser.readline().decode("ascii", "replace").strip()
        if line:
            print(f"< {line}", flush=True)
        if token in line:
            return True
    return False


def send(ser: serial.Serial, cmd: str) -> None:
    print(f"> {cmd}", flush=True)
    ser.write((cmd + "\n").encode("ascii"))


def run(ser: serial.Serial, hold_s: float, ramp: list[float]) -> int:
    if not expect(ser, "#READY", 5.0):
        print("ERR: no #READY from foc_test (wrong firmware or baud?)", file=sys.stderr)
        return 1
    send(ser, "?")
    time.sleep(1.0)

    send(ser, "O")
    if not expect(ser, "#OPENLOOP go", 3.0):
        print("ERR: open loop refused", file=sys.stderr)
        return 1
    time.sleep(hold_s)
    send(ser, "S")
    time.sleep(12.0)  # open-loop 10 s cooldown before the next motion.

    send(ser, "C")
    if not expect(ser, "#CAL", 10.0):
        print("ERR: alignment failed (magnet gap / wiring?)", file=sys.stderr)
        return 1

    for rps in ramp:
        send(ser, f"W {rps}")
        if not expect(ser, "#VELOCITY go", 3.0):
            print(f"ERR: W {rps} refused", file=sys.stderr)
            return 1
        time.sleep(hold_s)
        send(ser, "S")
        time.sleep(1.0)
    print("OK: sequence complete, check #SPD/#IOL above", flush=True)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="foc_test bench spin sequence")
    ap.add_argument("--port", required=True, help="serial device, e.g. /dev/serial/by-id/... ")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--hold", type=float, default=6.0, help="seconds to hold each speed")
    ap.add_argument("--ramp", default="1,3,5,8,10", help="W setpoints in rps")
    args = ap.parse_args()
    ramp = [float(x) for x in args.ramp.split(",") if x.strip()]
    if not ramp or any(abs(x) > 10.0 for x in ramp):
        print("ERR: ramp must be within +-10 rps (firmware clamp)", file=sys.stderr)
        return 1
    ser = serial.Serial(args.port, args.baud, timeout=1.0)
    ser.reset_input_buffer()
    try:
        return run(ser, args.hold, ramp)
    finally:
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())

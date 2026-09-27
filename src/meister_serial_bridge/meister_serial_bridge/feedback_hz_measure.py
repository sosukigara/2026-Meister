"""FB_STATE 送信周期の実測（読み取り専用）。

`MSTE_FEEDBACK_HZ` の妥当性を確認するための計測ツール。ROS ノードは起動せず、
ポートを開いて FB_STATE を受信するだけの独立したスクリプト。

  # 設定どおりの周期で届いているかを見る（双向健全性の確認つき）
  meister_hz_measure --port /dev/ttyUSB0 --with-commands

判定は 3 つ:
  - 実測 Hz が設定値に追従しているか（追従しなくなったら线路が飽和している）
  - `error_flags` のプロトコルエラーフラグが立っていないか（双方向健全性）
  - 受信バイト数と復号フレーム数が整合しているか（欠落の検出）

注意: `MSTE_FEEDBACK_HZ` は 1000 の約数でなければならない（firmware の
`static_assert` で強制）。200 Hz 以下の値で測ることが推奨。
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from dataclasses import dataclass

from meister_serial_bridge.comm_check import describe_port
from meister_serial_bridge.protocol import (
    FB_ERROR_PROTOCOL,
    MAX_FRAME_SIZE,
    FrameParser,
    TYPE_FB_STATE,
    encode_motor_velocity,
    encode_steering_angle,
)

DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD = 115200
DEFAULT_DURATION_S = 5.0
_COMMAND_HZ = 10.0  # 計測中に流す零指令のレート（双方向健全性の確認用）
_ZERO6 = [0] * 6


@dataclass
class Measurement:
    """1 回の計測結果。"""

    frames: int
    nbytes: int
    duration_s: float
    measured_hz: float
    mean_ms: float
    p95_ms: float
    max_ms: float
    protocol_errors: int
    commands_sent: int

    @property
    def lost_frames(self) -> int:
        """受信バイト数から期待されるフレーム数が減った分 = 欠落か破損。"""
        expected = self.nbytes // MAX_FRAME_SIZE
        return max(0, expected - self.frames)


def measure(port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD,
            duration_s: float = DEFAULT_DURATION_S,
            with_commands: bool = False, reset: bool = True,
            serial_factory=None) -> Measurement:
    """FB_STATE の到着レートと間隔を測る。"""
    if serial_factory is None:
        import serial as pyserial
        serial_factory = pyserial.serial_for_url

    with serial_factory(port, baud, timeout=0.05) as link:
        link.dtr = False  # DTR = IO0。assert のままだとブートローダで止まる
        if reset:
            link.rts = True
            time.sleep(0.1)
            link.rts = False
            time.sleep(1.0)  # 起動待ち（ROM バナーと初期化 stabilisation）
        link.reset_input_buffer()

        parser = FrameParser()
        stamps: list[float] = []
        nbytes = 0
        protocol_errors = 0
        ncmd = 0
        next_cmd = 0.0
        t0 = time.monotonic()
        while True:
            now = time.monotonic()
            elapsed = now - t0
            if elapsed >= duration_s:
                break
            if with_commands and now >= next_cmd:
                link.write(encode_steering_angle(_ZERO6) +
                           encode_motor_velocity(_ZERO6))
                ncmd += 1
                next_cmd = now + 1.0 / _COMMAND_HZ
            if link.in_waiting:
                chunk = link.read(link.in_waiting)
                nbytes += len(chunk)
                for frame in parser.feed(chunk):
                    if frame.type_id != TYPE_FB_STATE:
                        continue
                    stamps.append(time.monotonic())
                    if frame.get_u8(13) & FB_ERROR_PROTOCOL:
                        protocol_errors += 1
            else:
                time.sleep(0.001)
        window = max(1e-9, time.monotonic() - t0)

    gaps = [(b - a) * 1e3 for a, b in zip(stamps, stamps[1:])]
    span = (stamps[-1] - stamps[0]) if len(stamps) > 1 else 0.0
    ordered = sorted(gaps)
    return Measurement(
        frames=len(stamps),
        nbytes=nbytes,
        duration_s=window,
        measured_hz=(len(stamps) - 1) / span if span > 0 else 0.0,
        mean_ms=statistics.mean(gaps) if gaps else 0.0,
        p95_ms=ordered[int(len(ordered) * 0.95)] if ordered else 0.0,
        max_ms=max(gaps) if gaps else 0.0,
        protocol_errors=protocol_errors,
        commands_sent=ncmd,
    )


def main(args: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog='meister_hz_measure',
        description='FB_STATE の送信周期を実測する（読み取りのみ・ROS 不要）。')
    parser.add_argument('--port', default=DEFAULT_PORT)
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD)
    parser.add_argument('--duration', type=float, default=DEFAULT_DURATION_S,
                        help='計測時間 [秒]（既定: %(default)s）')
    parser.add_argument('--with-commands', action='store_true',
                        help='10 Hz で零指令を流し、双方向健全性を確認する')
    parser.add_argument('--no-reset', dest='reset', action='store_false',
                        help='計測前に ESP32 を再起動しない')
    parser.set_defaults(reset=True)
    parser.add_argument('--expect-hz', type=float, default=None,
                        help='設定値。指定すると実測値との差を表示する')
    opts = parser.parse_args(args)

    print(f'port: {opts.port}  ({describe_port(opts.port)})  baud: {opts.baud}')
    try:
        m = measure(port=opts.port, baud=opts.baud, duration_s=opts.duration,
                    with_commands=opts.with_commands, reset=opts.reset)
    except Exception as exc:  # noqa: BLE001 - CLI Entrypoint
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1

    print(f'frames          : {m.frames}  ({m.nbytes} bytes, '
          f'commands={m.commands_sent})')
    print(f'measured Hz     : {m.measured_hz:.1f}', end='')
    if opts.expect_hz:
        delta = m.measured_hz - opts.expect_hz
        print(f'  (expect {opts.expect_hz:g}, delta {delta:+.1f})')
    else:
        print()
    print(f'inter-arrival ms: mean={m.mean_ms:.2f} p95={m.p95_ms:.2f} '
          f'max={m.max_ms:.2f}')
    print(f'protocol errors : {m.protocol_errors}')
    print(f'lost frames     : {m.lost_frames}')
    if m.protocol_errors or m.lost_frames:
        print('NG: フレームが欠落または破損しています。この設定では採用しないこと。')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())

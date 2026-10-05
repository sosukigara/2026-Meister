"""ESP32 ⇔ PC 通信のベンチ確認 CLI.

モータやサーボを動かさずに、プロトコル通信が成立しているかを 1 コマンドで検証する。
`firmware/main/platformio.ini` の `esp32dev_usbuart` 環境（プロトコル UART = USB/UART0）
へ書き込んだ ESP32 に対して使う。

  1. uplink    ESP32 からの FB_STATE をチェックサム検証付きで受け取れるか
  2. downlink  4 種の有効なコマンドを送り、FB_STATE にプロトコルエラーフラグが
               立たないこと（フレーム長が ESP32 側と一致しているかの検証）
  3. err_flag  意図的に壊したフレームを 1 個送り、プロトコルエラーフラグが立つこと
               （手順 2 の判定基準がESP32 側で実際に機能していることの裏取り）

手順 2 は ESP32 側の受信エラーが再起動でしか消えないことに依存する。既にフラグが
立っている場合は手順 2 が判定できないので、--reset で再起動してから実行する。
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Callable

from meister_serial_bridge.protocol import (
    FB_ERROR_PROTOCOL,
    FrameParser,
    TYPE_FB_STATE,
    encode_arm_angle,
    encode_gripper,
    encode_motor_velocity,
    encode_steering_angle,
)

DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD = 115200
DEFAULT_TIMEOUT_S = 2.0

_POLL_S = 0.002  # 受信待ちのポーリング間隔 [秒]

# 通信確認専用。中立/停止の値だけを流す（モータ・サーボを動かさない）。
COMMAND_FRAMES: tuple[tuple[str, bytes], ...] = (
    ('CMD_MOTOR_VELOCITY', encode_motor_velocity([0] * 6)),
    ('CMD_STEERING_ANGLE', encode_steering_angle([0] * 6)),
    ('CMD_ARM_ANGLE', encode_arm_angle([0] * 4)),
    ('CMD_GRIPPER', encode_gripper(2)),  # 2 = 停止
)


class CommError(RuntimeError):
    """通信確認が成立しなかった（使用者向けの理由）。"""


def describe_port(port: str) -> str:
    """ポートの /dev/serial/by-id 名を返す。

    複数の ESP32 を同時接続しているときに誤接続を見分けるため。
    """
    try:
        real = os.path.realpath(port)
        for path in glob.glob('/dev/serial/by-id/*'):
            if os.path.realpath(path) == real:
                return path
    except OSError:
        pass
    return port


@dataclass(frozen=True)
class Feedback:
    """FB_STATE 1 フレームのデコード結果。"""

    encoders: tuple[int, ...]
    state: int
    error_flags: int

    @property
    def has_protocol_error(self) -> bool:
        return bool(self.error_flags & FB_ERROR_PROTOCOL)

    def describe(self) -> str:
        # enc[6] は実エンコーダではなく firmware/main/src/main.cpp の時間由来ダミー値
        # (sendFeedbackIfDue)。誤解を避けるため placeholder と明示する。
        return (f'enc(placeholder)={list(self.encoders)} '
                f'state=0x{self.state:02x} error=0x{self.error_flags:02x}')


@dataclass
class Step:
    """確認項目の結果。"""

    name: str
    ok: bool
    detail: str


@dataclass
class CheckResult:
    port: str
    steps: list[Step] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return bool(self.steps) and all(step.ok for step in self.steps)


def _drain(link) -> bytes:
    """受信バッファを非ブロッキングで抜く。

    pyserial の read(size) は size バイトがそろうか timeout するまでブロックする。
    read(size) で回すと 1 回の呼び出しで timeout 分を失い、受信レートを観測できなく
    なるため、in_waiting を見てから読む。
    """
    n = link.in_waiting
    return link.read(n) if n else b''


def _collect_feedback(link, parser: FrameParser, deadline: float,
                      want: int = 1) -> list[Feedback]:
    """deadline まで FB_STATE を最大 want 個読む。他種別は無視する。"""
    found: list[Feedback] = []
    while time.monotonic() < deadline and len(found) < want:
        chunk = _drain(link)
        if not chunk:
            time.sleep(_POLL_S)
            continue
        for frame in parser.feed(chunk):
            if frame.type_id != TYPE_FB_STATE:
                continue
            found.append(Feedback(
                encoders=tuple(frame.get_int16(i * 2) for i in range(6)),
                state=frame.get_u8(12),
                error_flags=frame.get_u8(13),
            ))
    return found


def _wait_protocol_error(link, parser: FrameParser, deadline: float) -> bool:
    """プロトコルエラーフラグが立った FB_STATE が届くか。"""
    return any(fb.has_protocol_error
               for fb in _collect_feedback(link, parser, deadline, want=32))


def _reset_esp32(link) -> None:
    """EN 線 (RTS) をパルスして ESP32 を再起動する。

    受信エラーは再起動でしか消えないため手順 2 の前提にする。
    標準オートプログラム回路はエミッタ接地（active-LOW）なので、
    RTS=1 で EN=Low になってリセットし、RTS=0 で通常起動する。
    IO0 (DTR) は Low だと ROM ダウンロードモードで落ちるので、
    RTS をトグルする前に DTR を解放しておく（run_check が行う）。
    """
    link.rts = True
    time.sleep(0.1)
    link.rts = False
    time.sleep(0.5)


def run_check(port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD,
              timeout: float = DEFAULT_TIMEOUT_S, reset: bool = True,
              serial_factory: Callable | None = None) -> CheckResult:
    """ESP32 との通信を検証して結果を返す。失敗時は CommError を投げる。"""
    if serial_factory is None:
        import serial as pyserial  # ROS 実行時だけ要求する（テストでは注入する）
        serial_factory = pyserial.serial_for_url

    result = CheckResult(port=port)
    with serial_factory(port, baud, timeout=0) as link:
        # pyserial はポートを開いた直後に DTR を assert する。標準オートプログラム
        # 回路では DTR = IO0 なので、assert されたままではリセット時に
        # ブートローダで起動して FB_STATE が返らない。先に解放しておく。
        link.dtr = False
        if reset:
            _reset_esp32(link)
        link.reset_input_buffer()
        parser = FrameParser()

        # --- 1. 上り (ESP32 -> PC) ---
        uplink = _collect_feedback(link, parser, time.monotonic() + timeout)
        if not uplink:
            raise CommError(
                f'FB_STATE を受信できませんでした ({describe_port(port)} @ {baud}bps)。'
                'esp32dev_usbuart ビルドが書き込まれているか、'
                'ポートとボーレートが正しいかを確認してください。')
        result.steps.append(Step('uplink', True, uplink[0].describe()))

        if uplink[0].has_protocol_error:
            raise CommError(
                'FB_STATE に既にプロトコルエラーフラグが立っています。'
                'このフラグは再起動でしか消えないため、--reset（既定）付きで'
                '再実行してください。')

        # --- 2. 下り (PC -> ESP32) ---
        # FB_STATE はコマンドへの応答ではなく 10 Hz 周期送信なので、
        # 4 種を送った直後の最初の FB_STATE がそれに対する応答になる。
        # フラグが立たなければ ESP32 側は全フレームを正常に受理している。
        names = ', '.join(name for name, _ in COMMAND_FRAMES)
        link.write(b''.join(frame for _, frame in COMMAND_FRAMES))
        link.flush()
        reply = _collect_feedback(link, parser, time.monotonic() + timeout, want=1)
        if not reply:
            raise CommError(
                f'{names} を送ったあとの FB_STATE が返ってきませんでした。')
        if reply[0].has_protocol_error:
            raise CommError(
                f'{names} に対してプロトコルエラーフラグが立ちました '
                f'({reply[0].describe()})。フレーム長が ESP32 側と一致していない可能性があります。')
        result.steps.append(Step('downlink', True,
                                 f'{names} -> {reply[0].describe()}'))

        # --- 3. 手順 2 の判定基準が機能していることの裏取り ---
        # チェックサムを壊したフレームを送り、フラグが立つことを確認する。
        good = encode_motor_velocity([0] * 6)
        broken = bytearray(good)
        broken[-1] ^= 0xFF
        link.reset_input_buffer()
        link.write(bytes(broken))
        link.flush()
        flagged = _wait_protocol_error(link, parser, time.monotonic() + timeout)
        if not flagged:
            raise CommError(
                '意図的に壊したフレームでプロトコルエラーフラグが立ちませんでした。'
                '手順 2 の判定基準が機能していないため結果は採用できません。')
        result.steps.append(Step('err_flag', True,
                                 f'corrupt frame -> error_flags |= 0x{FB_ERROR_PROTOCOL:02x}'))

    return result


def main(args: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog='meister_comm_check',
        description='ESP32 とのプロトコル通信を検証する（モータ/サーボは動かさない）。')
    parser.add_argument('--port', default=DEFAULT_PORT,
                        help=f'シリアルポート（既定: {DEFAULT_PORT}）')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD,
                        help=f'ボーレート（既定: {DEFAULT_BAUD}）')
    parser.add_argument('--timeout', type=float, default=DEFAULT_TIMEOUT_S,
                        help='各項目の待ち時間 [秒]（既定: %(default)s）')
    parser.add_argument('--no-reset', dest='reset', action='store_false',
                        help='開始時に ESP32 を再起動しない')
    parser.set_defaults(reset=True)
    opts = parser.parse_args(args)

    print(f'port: {opts.port}  ({describe_port(opts.port)})  baud: {opts.baud}')
    try:
        result = run_check(port=opts.port, baud=opts.baud,
                           timeout=opts.timeout, reset=opts.reset)
    except CommError as exc:
        print(f'FAIL: {exc}', file=sys.stderr)
        return 1
    except Exception as exc:  # noqa: BLE001 - CLI なので原因を，采取して終了する
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1

    for step in result.steps:
        print(f'  [{"OK" if step.ok else "NG"}] {step.name:<9} {step.detail}')
    print('PASS: PC <-> ESP32 通信を確認しました。')
    return 0


if __name__ == '__main__':
    sys.exit(main())

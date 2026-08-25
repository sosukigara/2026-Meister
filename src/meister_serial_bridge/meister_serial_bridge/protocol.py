"""Meister UART プロトコル — Python 実装 (PC 側).

firmware/include/meister_protocol.h と同一フォーマットのエンコーダ/パーサ。
固定長バイナリフレーム + XOR チェックサム、リトルエンディアン。

  [0]      ヘッダ     0xA5
  [1]      種別       TypeId
  [2..L-2] ペイロード (種別ごとに固定長)
  [L-1]    チェックサム (先頭からペイロード末尾までの XOR)
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

HEADER = 0xA5

TYPE_MOTOR_VELOCITY = 0x01  # int16 vel[6]  -1000..+1000 (千分率)
TYPE_STEERING_ANGLE = 0x02  # int16 ang[6]  -900..+900  (0.1 度)
TYPE_ARM_ANGLE = 0x03       # int16 ang[4]  0..1800     (0.1 度)
TYPE_GRIPPER = 0x04         # uint8 cmd     0=閉 1=開 2=停止
TYPE_FB_STATE = 0x81        # int16 enc[6] + uint8 state + uint8 error_flags
TYPE_FB_ERROR = 0x82        # uint8 error_code

NUM_DRIVE_MOTORS = 6
NUM_STEERING_SERVOS = 6
NUM_ARM_SERVOS = 4

MIN_VELOCITY = -1000
MAX_VELOCITY = 1000
MIN_STEERING = -900
MAX_STEERING = 900
MIN_ARM_ANGLE = 0
MAX_ARM_ANGLE = 1800

_PAYLOAD_SIZES = {
    TYPE_MOTOR_VELOCITY: NUM_DRIVE_MOTORS * 2,
    TYPE_STEERING_ANGLE: NUM_STEERING_SERVOS * 2,
    TYPE_ARM_ANGLE: NUM_ARM_SERVOS * 2,
    TYPE_GRIPPER: 1,
    TYPE_FB_STATE: NUM_DRIVE_MOTORS * 2 + 2,
    TYPE_FB_ERROR: 1,
}

MAX_PAYLOAD_SIZE = _PAYLOAD_SIZES[TYPE_FB_STATE]
MAX_FRAME_SIZE = 1 + 1 + MAX_PAYLOAD_SIZE + 1


def compute_checksum(data: bytes) -> int:
    """バイト列の XOR チェックサム。"""
    c = 0
    for b in data:
        c ^= b
    return c


def _clamp(value: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, value))


def _frame(type_id: int, payload: bytes) -> bytes:
    head = bytes((HEADER, type_id)) + payload
    return head + bytes((compute_checksum(head),))


def encode_motor_velocity(vel: list[int]) -> bytes:
    """CMD_MOTOR_VELOCITY フレームを生成する (6ch, 千分率)。"""
    if len(vel) != NUM_DRIVE_MOTORS:
        raise ValueError(f"expected {NUM_DRIVE_MOTORS} values, got {len(vel)}")
    payload = struct.pack(
        "<6h", *(_clamp(int(v), MIN_VELOCITY, MAX_VELOCITY) for v in vel)
    )
    return _frame(TYPE_MOTOR_VELOCITY, payload)


def encode_steering_angle(angle: list[int]) -> bytes:
    """CMD_STEERING_ANGLE フレームを生成する (6ch, 0.1 度)。"""
    if len(angle) != NUM_STEERING_SERVOS:
        raise ValueError(f"expected {NUM_STEERING_SERVOS} values, got {len(angle)}")
    payload = struct.pack(
        "<6h", *(_clamp(int(a), MIN_STEERING, MAX_STEERING) for a in angle)
    )
    return _frame(TYPE_STEERING_ANGLE, payload)


def encode_arm_angle(angle: list[int]) -> bytes:
    """CMD_ARM_ANGLE フレームを生成する (4ch, 0.1 度)。"""
    if len(angle) != NUM_ARM_SERVOS:
        raise ValueError(f"expected {NUM_ARM_SERVOS} values, got {len(angle)}")
    payload = struct.pack(
        "<4h", *(_clamp(int(a), MIN_ARM_ANGLE, MAX_ARM_ANGLE) for a in angle)
    )
    return _frame(TYPE_ARM_ANGLE, payload)


def encode_gripper(cmd: int) -> bytes:
    """CMD_GRIPPER フレームを生成する (0=閉 1=開 2=停止)。"""
    return _frame(TYPE_GRIPPER, bytes((_clamp(int(cmd), 0, 2),)))


@dataclass
class Frame:
    type_id: int
    payload: bytes

    def get_int16(self, offset: int) -> int:
        return struct.unpack_from("<h", self.payload, offset)[0]

    def get_u8(self, offset: int) -> int:
        return self.payload[offset]


class FrameParser:
    """ストリームから固定長フレームを抽出するインクリメンタルパーサ。

    ESP32 側 main.cpp の rx ステートマシンと同じ挙動。
    """

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[Frame]:
        """受信バイトを投入し、完了したフレームのリストを返す。"""
        self._buf.extend(data)
        frames: list[Frame] = []
        while True:
            # ヘッダ同期
            while self._buf and self._buf[0] != HEADER:
                self._buf.pop(0)
            if len(self._buf) < 2:
                break
            type_id = self._buf[1]
            payload_size = _PAYLOAD_SIZES.get(type_id, 0)
            if payload_size == 0:
                # 未知種別 → ヘッダ破棄して再同期
                self._buf.pop(0)
                continue
            frame_size = 2 + payload_size + 1
            if len(self._buf) < frame_size:
                break
            frame_bytes = bytes(self._buf[:frame_size])
            del self._buf[:frame_size]
            if compute_checksum(frame_bytes) != 0:
                continue  # チェックサム不一致 → 破棄
            frames.append(
                Frame(type_id=type_id, payload=frame_bytes[2 : 2 + payload_size])
            )
        return frames

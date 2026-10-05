"""comm_check.py のテスト: 検証手順が実際の判定として機能すること。

テストダブルは firmware/main/src/main.cpp の受信・送信の挙動（有効なフレームは
受理してエラー無しの FB_STATE を返す、壊れたフレームはエラーフラグを立てる、
フラグは再起動まで保持）と、IO0(DTR) が Low のままだとブートローダで止まる
という実際の挙動を再現する。
"""

import pytest

from meister_serial_bridge import protocol as p
from meister_serial_bridge.comm_check import CommError, run_check


class FakeEsp32:
    """ESP32 ファームウェアの最小のふるまいを再現するテストダブル。"""

    def __init__(self, *, reply=True, reject_valid=False, notice_corrupt=True,
                 dtr_stuck=False):
        self._parser = p.FrameParser()
        self._rx = bytearray()
        self._reply = reply
        self._reject_valid = reject_valid
        self._notice_corrupt = notice_corrupt
        self._dtr = True          # pyserial は open 時に DTR を assert する
        self._dtr_stuck = dtr_stuck
        self._app_running = False
        self.error_flags = 0
        self.written = bytearray()
        self.rts = True

    # ---- pyserial.Serial 相当のインターフェース ----
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    @property
    def dtr(self):
        return self._dtr

    @dtr.setter
    def dtr(self, value):
        if not self._dtr_stuck:
            self._dtr = bool(value)

    def _tick(self):
        """アプリが起動しているか。IO0=Low のままならブートローダで停止。"""
        if not self._app_running:
            if self._dtr:
                return False
            self._app_running = True
        return self._app_running

    @property
    def in_waiting(self):
        if not self._tick():
            return 0
        if self._reply and not self._rx:
            # FB_STATE は指令への応答ではなく周期送信（実機と同じ）。
            self._rx.extend(p.encode_state([0] * 6, 0, self.error_flags))
        return len(self._rx)

    def read(self, size):
        if not self._tick():
            return b''
        chunk = bytes(self._rx[:size])
        del self._rx[:size]
        return chunk

    def write(self, data):
        if not self._tick():
            return
        self.written.extend(data)
        frames = self._parser.feed(bytes(data))
        consumed = sum(len(f.payload) + 3 for f in frames)
        if consumed != len(data):            # 破棄された = 壊れたフレームがある
            if self._notice_corrupt:
                self.error_flags |= p.FB_ERROR_PROTOCOL
        elif frames and self._reject_valid:  # 有効フレームを拒否する firmware
            self.error_flags |= p.FB_ERROR_PROTOCOL
        if self._reply:
            self._rx.extend(p.encode_state([0] * 6, 0, self.error_flags))

    def flush(self):
        pass

    def reset_input_buffer(self):
        self._rx.clear()


def _factory(spy):
    def factory(port, baud, timeout):
        return spy
    return factory


def _check(spy, **kwargs):
    kwargs.setdefault('port', 'fake')
    kwargs.setdefault('reset', False)
    kwargs.setdefault('timeout', 0.2)
    return run_check(serial_factory=_factory(spy), **kwargs)


def test_passes_when_esp32_answers_both_directions():
    """FB_STATE が届き、4 種の指令が受理され、破損検知も作動する。"""
    result = _check(FakeEsp32())
    assert result.ok
    assert [step.name for step in result.steps] == ['uplink', 'downlink', 'err_flag']


def test_fails_when_esp32_never_answers():
    """応答が返らない場合は uplink 手順で失敗する。"""
    with pytest.raises(CommError, match='FB_STATE を受信できませんでした'):
        _check(FakeEsp32(reply=False))


def test_fails_when_valid_command_is_flagged_as_error():
    """有効な指令が ESP32 側で破損扱いされたら、その時点で失敗する。"""
    with pytest.raises(CommError, match='プロトコルエラーフラグが立ちました'):
        _check(FakeEsp32(reject_valid=True))


def test_fails_when_corrupt_frame_is_not_acknowledged():
    """破損検知が働かない firmware では downlink の判定基準を信用できない。"""
    with pytest.raises(CommError, match='意図的に壊したフレーム'):
        _check(FakeEsp32(notice_corrupt=False))


def test_sends_only_neutral_commands():
    """実機で実行しても動かないよう、指令は全 0 / グリッパー停止のみ。"""
    esp = FakeEsp32()
    _check(esp)
    # 手順 3 の壊したフレームはパーサが破棄するので、残るのは有効な指令だけ
    frames = p.FrameParser().feed(bytes(esp.written))
    assert [f.type_id for f in frames] == [
        p.TYPE_MOTOR_VELOCITY, p.TYPE_STEERING_ANGLE,
        p.TYPE_ARM_ANGLE, p.TYPE_GRIPPER,
    ]
    for frame in frames[:3]:
        assert all(frame.get_int16(i * 2) == 0
                   for i in range(len(frame.payload) // 2))
    assert frames[3].get_u8(0) == 2  # 2 = 停止


def test_releases_dtr_before_any_io():
    """DTR は IO0 に直結している。assert されたままだとブートローダで止まる。"""
    assert _check(FakeEsp32()).ok
    with pytest.raises(CommError, match='FB_STATE を受信できませんでした'):
        _check(FakeEsp32(dtr_stuck=True))

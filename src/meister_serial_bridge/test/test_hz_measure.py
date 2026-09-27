"""feedback_hz_measure.py のテスト: 計測がフレーム数・破損フラグ・欠落を正しく数えること。"""

from meister_serial_bridge import protocol as p
from meister_serial_bridge.feedback_hz_measure import measure


class ScriptedEsp32:
    """ あらかじめ用意したバイト列を返すだけのテストダブル。"""

    def __init__(self, chunks):
        self._chunks = list(chunks)
        self._buf = bytearray()
        self.dtr = True
        self.rts = True
        self.written = bytearray()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    @property
    def in_waiting(self):
        if not self._buf and self._chunks:
            self._buf.extend(self._chunks.pop(0))
        return len(self._buf)

    def read(self, size):
        chunk = bytes(self._buf[:size])
        del self._buf[:size]
        return chunk

    def write(self, data):
        self.written.extend(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        self._buf.clear()


def _factory(spy):
    def factory(port, baud, timeout):
        return spy
    return factory


def _state(error_flags=0):
    return p.encode_state([0] * 6, 0, error_flags)


def _run(spy, **kwargs):
    kwargs.setdefault('port', 'fake')
    kwargs.setdefault('duration_s', 0.15)
    kwargs.setdefault('reset', False)
    return measure(serial_factory=_factory(spy), **kwargs)


def test_counts_every_frame_without_loss():
    spy = ScriptedEsp32([b''.join(_state() for _ in range(20))])
    m = _run(spy)
    assert m.frames == 20
    assert m.lost_frames == 0
    assert m.protocol_errors == 0
    assert m.commands_sent == 0


def test_counts_protocol_error_flag_frames():
    spy = ScriptedEsp32([
        b''.join(_state() for _ in range(10)),
        _state(p.FB_ERROR_PROTOCOL),
    ])
    m = _run(spy)
    assert m.frames == 11
    assert m.protocol_errors == 1


def test_detects_frame_dropped_on_the_wire():
    """壊れた（または UART 側で失われた）フレームはバイト数だけ増えて
    復号数が増えないので、欠落として検出できる。"""
    corrupt = bytearray(_state())
    corrupt[-1] ^= 0xFF
    spy = ScriptedEsp32([b''.join(_state() for _ in range(10)) + bytes(corrupt)])
    m = _run(spy)
    assert m.frames == 10
    assert m.lost_frames == 1


def test_sends_commands_when_requested():
    spy = ScriptedEsp32([b''.join(_state() for _ in range(20))])
    m = _run(spy, with_commands=True)
    assert m.commands_sent >= 1
    # 零指令（速度 0 / 舵角 0）だけを送る
    frames = p.FrameParser().feed(bytes(spy.written))
    assert frames, '指令が送られていない'
    for frame in frames:
        assert all(frame.get_int16(i * 2) == 0
                   for i in range(len(frame.payload) // 2))

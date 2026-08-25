"""protocol.py のテスト: フレーム形式が firmware/include/meister_protocol.h と一致すること。"""

from meister_serial_bridge import protocol as p


def test_motor_velocity_frame_layout():
    """ヘッダ/種別/ペイロード/チェックサムのレイアウトとリトルエンディアンを検証。"""
    frame = p.encode_motor_velocity([1000, 500, 0, -500, -1000, 321])
    assert len(frame) == 15  # 1 + 1 + 12 + 1
    assert frame[0] == 0xA5
    assert frame[1] == p.TYPE_MOTOR_VELOCITY
    # リトルエンディアンの int16 (例: 1000 = 0x03E8 -> E8 03)
    assert frame[2:4] == b'\xe8\x03'
    assert frame[4:6] == b'\xf4\x01'   # 500
    assert frame[6:8] == b'\x00\x00'   # 0
    assert frame[8:10] == b'\x0c\xfe'  # -500
    # チェックサム: 全バイト XOR == 0
    assert p.compute_checksum(frame) == 0


def test_steering_frame_clamps():
    frame = p.encode_steering_angle([900, -900, 2000, -2000, 123, -123])
    assert len(frame) == 15
    assert frame[1] == p.TYPE_STEERING_ANGLE
    f = p.FrameParser().feed(frame)[0]
    assert [f.get_int16(i * 2) for i in range(6)] == [900, -900, 900, -900, 123, -123]


def test_parser_roundtrip_and_resync():
    parser = p.FrameParser()
    f1 = p.encode_motor_velocity([100, -200, 300, -400, 500, -600])
    f2 = p.encode_steering_angle([10, 20, 30, 40, 50, 60])
    # 1 バイトずつばらして送っても復元できる
    frames = []
    for b in (f1 + f2):
        frames.extend(parser.feed(bytes([b])))
    assert len(frames) == 2
    assert frames[0].type_id == p.TYPE_MOTOR_VELOCITY
    assert [frames[0].get_int16(i * 2) for i in range(6)] == [100, -200, 300, -400, 500, -600]
    assert frames[1].type_id == p.TYPE_STEERING_ANGLE


def test_parser_drops_corrupt_frame():
    parser = p.FrameParser()
    good = p.encode_motor_velocity([1, 2, 3, 4, 5, 6])
    corrupt = bytearray(good)
    corrupt[5] ^= 0xFF  # ペイロードを壊す
    frames = parser.feed(bytes(corrupt) + good)
    # 壊れたフレームは捨てられ、正常フレームだけ復元される
    assert len(frames) == 1
    assert frames[0].get_int16(0) == 1


def test_arm_gripper_frames():
    arm = p.encode_arm_angle([0, 900, 1800, 2500])
    assert len(arm) == 11
    assert arm[1] == p.TYPE_ARM_ANGLE
    gripper = p.encode_gripper(1)
    assert gripper == bytes([0xA5, 0x04, 0x01, 0xA5 ^ 0x04 ^ 0x01])

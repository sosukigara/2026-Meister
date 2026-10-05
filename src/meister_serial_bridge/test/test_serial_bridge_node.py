"""serial_bridge_node.py のテスト: アーム/グリッパーの下行経路。

pytest が PYTHONPATH=. だけで走る前提（このリポジトリの他のテストと同じ）なので、
rclpy とメッセージ型は差し替える。差し替えるのは Node の器だけ（パラメータ、時刻、
購読/公開/タイマーの登録、ログ）。検証対象の _on_arm_joint / _on_gripper /
_on_watchdog / _write_frames と protocol.encode_arm_angle はそのまま通る。
"""

import importlib
import sys
import time
import types

import pytest

from meister_serial_bridge import protocol as p


class _Log:
    def __init__(self):
        self.records = []

    def _emit(self, level, msg):
        self.records.append((level, msg))

    def info(self, msg, **kwargs):
        self._emit('info', msg)

    def warn(self, msg, **kwargs):
        self._emit('warn', msg)

    def error(self, msg, **kwargs):
        self._emit('error', msg)

    def debug(self, msg, **kwargs):
        self._emit('debug', msg)


class _Time:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def __sub__(self, other):
        return _Time(self.nanoseconds - other.nanoseconds)


class _Clock:
    """テストから明示的に進められる単調時計。"""

    def __init__(self):
        self._ns = 0

    def now(self):
        return _Time(self._ns)

    def advance(self, seconds):
        self._ns += int(seconds * 1e9)


class _Param:
    def __init__(self, value):
        self.value = value


class _Entity:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class Node:
    """rclpy.node.Node のうち本ノードが触る面だけを再現する。"""

    def __init__(self, name):
        self._name = name
        self._params = {}
        self._clock = _Clock()
        self._logger = _Log()
        self.subscriptions = []
        self.publishers = []
        self.timers = []

    def declare_parameter(self, name, value):
        self._params[name] = value

    def get_parameter(self, name):
        return _Param(self._params[name])

    def get_clock(self):
        return self._clock

    def get_logger(self):
        return self._logger

    def create_subscription(self, msg_type, topic, callback, depth):
        sub = _Entity(msg_type=msg_type, topic=topic, callback=callback, depth=depth)
        self.subscriptions.append(sub)
        return sub

    def create_publisher(self, msg_type, topic, depth):
        pub = _Entity(msg_type=msg_type, topic=topic, depth=depth)
        self.publishers.append(pub)
        return pub

    def create_timer(self, period, callback):
        timer = _Entity(period=period, callback=callback)
        self.timers.append(timer)
        return timer

    def destroy_node(self):
        pass


class Int16MultiArray:
    def __init__(self):
        self.data = []


class UInt8:
    def __init__(self):
        self.data = 0


class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


_STATE = {'ok': True}


def _install_ros_stubs():
    """rclpy / geometry_msgs / std_msgs を記憶中的な偽物に差し替える。"""
    class ExternalShutdownException(Exception):
        pass

    layout = {
        'rclpy': {
            'ok': lambda: _STATE['ok'],
            'init': lambda *a, **k: None,
            'shutdown': lambda *a, **k: None,
            'spin': lambda *a, **k: None,
        },
        'rclpy.executors': {'ExternalShutdownException': ExternalShutdownException},
        'rclpy.node': {'Node': Node},
        'geometry_msgs': {},
        'geometry_msgs.msg': {'Twist': Twist},
        'std_msgs': {},
        'std_msgs.msg': {'Int16MultiArray': Int16MultiArray, 'UInt8': UInt8},
    }
    made = {}
    for name, attrs in layout.items():
        module = types.ModuleType(name)
        for attr, value in attrs.items():
            setattr(module, attr, value)
        made[name] = module
    # import 文は親モジュールの属性もに引きにくるので、両方に出しておく
    made['rclpy'].executors = made['rclpy.executors']
    made['rclpy'].node = made['rclpy.node']
    made['geometry_msgs'].msg = made['geometry_msgs.msg']
    made['std_msgs'].msg = made['std_msgs.msg']
    sys.modules.update(made)


class FakeSerial:
    """pyserial.Serial のうち本ノードが使う面だけを再現する。"""

    def __init__(self):
        self.written = bytearray()
        self.flushes = 0
        self._rx = bytearray()

    @property
    def in_waiting(self):
        return len(self._rx)

    def read(self, size):
        # 受信スレッドは daemon で回し続ける。何も返さないとホットループに
        # なるので、実機の timeout 相当だけ休ませる。
        time.sleep(0.005)
        chunk = bytes(self._rx[:size])
        del self._rx[:size]
        return chunk

    def write(self, data):
        self.written.extend(data)

    def flush(self):
        self.flushes += 1

    def close(self):
        pass


@pytest.fixture(scope='module')
def node_cls():
    """差し替えを適用してノード本体を import し、後で sys.modules を戻す。"""
    saved = dict(sys.modules)
    _STATE['ok'] = True
    _install_ros_stubs()
    try:
        yield importlib.import_module(
            'meister_serial_bridge.serial_bridge_node').SerialBridgeNode
    finally:
        _STATE['ok'] = False       # 残った受信スレッドを止める
        for name in [n for n in sys.modules if n not in saved]:
            del sys.modules[name]
        sys.modules.update(saved)


@pytest.fixture
def bridge(node_cls, monkeypatch):
    """実 UART の代わりに FakeSerial をつないだ生ノードを 1 組返す。"""
    serial = pytest.importorskip('serial')
    spy = FakeSerial()
    monkeypatch.setattr(serial, 'serial_for_url', lambda *a, **k: spy)
    node = node_cls()
    assert node._serial is spy, 'テスト用の偽物が注入されていない'
    try:
        yield node, spy
    finally:
        node._serial = None
        node.destroy_node()


def _types(spy):
    return [f.type_id for f in p.FrameParser().feed(bytes(spy.written))]


def _arm_msg(*values):
    msg = Int16MultiArray()
    msg.data = list(values)
    return msg


def _gripper_msg(value):
    msg = UInt8()
    msg.data = value
    return msg


def test_arm_message_writes_one_arm_frame(bridge):
    """関節角 1 通で CMD_ARM_ANGLE が 1 フレームだけ出る。"""
    node, spy = bridge
    node._on_arm_joint(_arm_msg(0, 900, 1800, 450))
    frames = p.FrameParser().feed(bytes(spy.written))
    assert [f.type_id for f in frames] == [p.TYPE_ARM_ANGLE]
    assert [frames[0].get_int16(i * 2) for i in range(4)] == [0, 900, 1800, 450]
    # フレーム長は encode_arm_angle の戻り値と一致する (11 = 1 + 1 + 8 + 1)
    assert len(bytes(spy.written)) == len(p.encode_arm_angle([0, 900, 1800, 450]))
    assert spy.flushes == 1


def test_gripper_message_writes_one_gripper_frame(bridge):
    """グリッパー 1 通で CMD_GRIPPER が 1 フレームだけ出る。"""
    node, spy = bridge
    node._on_gripper(_gripper_msg(1))
    frames = p.FrameParser().feed(bytes(spy.written))
    assert [f.type_id for f in frames] == [p.TYPE_GRIPPER]
    assert frames[0].get_u8(0) == 1
    assert len(bytes(spy.written)) == len(p.encode_gripper(1))
    assert spy.flushes == 1


def test_out_of_range_joint_does_not_kill_the_node(bridge):
    """範囲外の関節角でもコールバックは例外を投げずノードは生き残る。

    最終的な権威は firmware の ClampArm() なのでノードは丸めない。ただし
    送らずに壊れるのも困るので、フレームが 1 枚出てパースできることは
    確かめる。値は encode_arm_angle が有限の長さへ収める。境界は
    Int16MultiArray が表せる両端 (-32768 / 32767) を使う。
    """
    node, spy = bridge
    node._on_arm_joint(_arm_msg(-32768, -900, 1801, 32767))
    frames = p.FrameParser().feed(bytes(spy.written))
    assert len(frames) == 1
    assert [frames[0].get_int16(i * 2) for i in range(4)] == [0, 0, 1800, 1800]
    # 例外が投げられていないので、次の指令も処理できる
    node._on_arm_joint(_arm_msg(0, 0, 0, 0))
    assert _types(spy) == [p.TYPE_ARM_ANGLE, p.TYPE_ARM_ANGLE]


def test_wrong_joint_count_is_ignored(bridge):
    """軸数が違うメッセージは固定長フレームに翻訳できないので無視する。"""
    node, spy = bridge
    node._on_arm_joint(_arm_msg(0, 900))
    assert _types(spy) == []


def test_arm_command_timeout_holds_last_target(bridge):
    """アームの指令が途絶えても停止フレームは送らない（最終角度を保持する）。

    CMD_ARM_ANGLE は絶対位置なので、途絶えた時点のアームはすでに最終目標で
    静止している。0 を送るのは停止ではなく原点への移動になる。ウォッチドッグ
    が実際に動いていることを車体側の停止フレームで確かめつつ、
    アームを追加送信しないことを見る。
    """
    node, spy = bridge
    node._on_arm_joint(_arm_msg(300, 300, 300, 300))
    before = _types(spy).count(p.TYPE_ARM_ANGLE)

    node._clock.advance(30.0)
    node._on_watchdog()
    node._on_watchdog()

    types_after = _types(spy)
    # ウォッチドッグは車体を停止させた (arm は増やさない)
    assert p.TYPE_MOTOR_VELOCITY in types_after
    assert p.TYPE_STEERING_ANGLE in types_after
    assert types_after.count(p.TYPE_ARM_ANGLE) == before
    assert node._arm_idle_logged is True

    # 新たなアーム指令が来れば保持を解除し、また書き始める
    node._on_arm_joint(_arm_msg(600, 600, 600, 600))
    assert _types(spy).count(p.TYPE_ARM_ANGLE) == before + 1
    assert node._arm_idle_logged is False


def test_arm_and_gripper_subscriptions_follow_cmd_vel_style(bridge):
    """cmd_vel と同じ cmd_ 規約・同じ QoS 深さで購読している。"""
    node, _ = bridge
    topics = {sub.topic: sub for sub in node.subscriptions}
    assert set(topics) == {'/cmd_vel', '/cmd_arm_joint', '/cmd_gripper'}
    assert all(sub.depth == 10 for sub in topics.values())
    assert topics['/cmd_arm_joint'].callback == node._on_arm_joint
    assert topics['/cmd_gripper'].callback == node._on_gripper

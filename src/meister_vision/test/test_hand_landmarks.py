"""hand_landmark_detector / hand_landmarks_node の pytest。

実の手 keypoint モデルは CI に無い (.gitignore の *.onnx) ので、
onnxruntime のセッションだけを偽物に差し替えて前処理・復号・空結果を
確かめる。「モデルが無い」経路は実ファイルを作らずに、パラメータと
環境変数だけで起こす。
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from meister_vision import hand_landmarks_node  # noqa: E402
from meister_vision.hand_landmark_detector import (  # noqa: E402
    MODEL_ENV_VAR, MODEL_FILENAME, HandLandmarkDetector, HandLandmarks,
    resolve_hand_model_path,
)
from meister_vision.hand_landmarks_node import HandLandmarksNode  # noqa: E402
from sensor_msgs.msg import Image as ImageMsg  # noqa: E402

SOURCE_ROOT = Path(__file__).resolve().parents[1]


class _FakeTensorInfo:
    def __init__(self, name: str, shape=None) -> None:
        self.name = name
        self.shape = shape


class _FakeSession:
    """onnxruntime.InferenceSession の必要な面だけを持つ偽物。"""

    def __init__(self, output: np.ndarray,
                 input_shape=(1, 224, 224, 3)) -> None:
        self._output = np.asarray(output, dtype=np.float32)
        self._input_shape = input_shape
        self.run_count = 0

    def get_inputs(self):
        return [_FakeTensorInfo("input_1", shape=self._input_shape)]

    def get_outputs(self):
        return [_FakeTensorInfo("output_1")]

    def set_output(self, output) -> None:
        """途中のフレームから検出結果を入れ替える (手あり / 手なし の切替用)。"""
        self._output = np.asarray(output, dtype=np.float32)

    def run(self, output_names, feed):
        self.run_count += 1
        self.last_feed = feed
        return [self._output]


def _landmarks_output(score=0.9, num_landmarks=21, spread=0.5):
    """(1, K, 3) の出力を作る。中央から scatter した x, y と score。"""
    out = np.zeros((1, num_landmarks, 3), dtype=np.float32)
    base_x = np.linspace(0.5 - spread / 2, 0.5 + spread / 2, num_landmarks)
    out[0, :, 0] = base_x
    out[0, :, 1] = 0.5
    out[0, :, 2] = score
    return out


def _plain_image(height=480, width=640):
    return np.full((height, width, 3), 128, dtype=np.uint8)


class TestNoHand:
    """手が映っていない画像が例外でなく「空結果」になること。"""

    def test_low_score_returns_none(self):
        session = _FakeSession(_landmarks_output(score=0.01))
        det = HandLandmarkDetector(session=session, score_threshold=0.5)
        assert det.detect(_plain_image()) is None

    def test_no_hand_image_does_not_raise(self):
        """実際のモデルが無いため同一画像でも空結果になり、例外は出ない。"""
        session = _FakeSession(_landmarks_output(score=0.0))
        det = HandLandmarkDetector(session=session, score_threshold=0.2)
        assert det.detect(_plain_image()) is None

    def test_score_equal_to_threshold_is_accepted(self):
        """境界は「未満で捨てる」。yolo_detector の scores >= conf と揃える。"""
        session = _FakeSession(_landmarks_output(score=0.5))
        det = HandLandmarkDetector(session=session, score_threshold=0.5)
        result = det.detect(_plain_image())
        assert result is not None
        assert result.confidence == pytest.approx(0.5)

    def test_weakest_landmark_decides(self):
        """1 点だけ崩れた landmark で全体が不採用になること。"""
        out = _landmarks_output(score=0.95)
        out[0, 7, 2] = 0.10
        det = HandLandmarkDetector(
            session=_FakeSession(out), score_threshold=0.5)
        assert det.detect(_plain_image()) is None

    def test_session_actually_ran(self):
        session = _FakeSession(_landmarks_output(score=0.01))
        HandLandmarkDetector(session=session).detect(_plain_image())
        assert session.run_count == 1


class TestDecode:
    def test_landmarks_are_pixels_inside_image(self):
        det = HandLandmarkDetector(
            session=_FakeSession(_landmarks_output(score=0.9)),
            score_threshold=0.5)
        result = det.detect(_plain_image(480, 640))
        assert result is not None
        assert len(result.landmarks) == 21
        for x, y in result.landmarks:
            assert 0.0 <= x <= 640.0
            assert 0.0 <= y <= 480.0

    def test_confidence_is_reported(self):
        det = HandLandmarkDetector(
            session=_FakeSession(_landmarks_output(score=0.83)),
            score_threshold=0.5)
        result = det.detect(_plain_image())
        assert isinstance(result, HandLandmarks)
        assert result.confidence == pytest.approx(0.83)

    def test_output_shape_contract_is_enforced(self):
        """契約と違う形は推測で読まず ValueError で落とす (R4)。"""
        bad = np.zeros((1, 63), dtype=np.float32)
        det = HandLandmarkDetector(
            session=_FakeSession(bad), score_threshold=0.2)
        with pytest.raises(ValueError) as excinfo:
            det.detect(_plain_image())
        assert "(1, 21, 3)" in str(excinfo.value)

    def test_unsupported_input_shape_is_rejected(self):
        """入力が静的 4 次元でなければ作らない。"""
        session = _FakeSession(_landmarks_output(), input_shape=[1, "h", "w", 3])
        with pytest.raises(ValueError):
            HandLandmarkDetector(session=session)

    def test_nchw_input_is_supported(self):
        session = _FakeSession(_landmarks_output(score=0.9),
                               input_shape=(1, 3, 224, 224))
        det = HandLandmarkDetector(session=session, score_threshold=0.5)
        blob = det._preprocess(_plain_image())[0]
        assert blob.shape == (1, 3, 224, 224)

    def test_bad_image_shape_raises(self):
        det = HandLandmarkDetector(session=_FakeSession(_landmarks_output()))
        with pytest.raises(ValueError):
            det.detect(np.zeros((10, 10), dtype=np.uint8))


class TestModelResolution:
    """モデルが見つからない時に握り潰さず例外にすること。"""

    def test_explicit_missing_path_raises(self, tmp_path, monkeypatch):
        monkeypatch.delenv(MODEL_ENV_VAR, raising=False)
        missing = str(tmp_path / "absent.onnx")
        with pytest.raises(FileNotFoundError) as excinfo:
            HandLandmarkDetector(model_path=missing)
        text = str(excinfo.value)
        assert missing in text
        assert MODEL_FILENAME in text

    def test_missing_env_path_raises(self, tmp_path, monkeypatch):
        missing = str(tmp_path / "from_env.onnx")
        monkeypatch.setenv(MODEL_ENV_VAR, missing)
        with pytest.raises(FileNotFoundError) as excinfo:
            HandLandmarkDetector()
        assert missing in str(excinfo.value)

    def test_env_var_wins_over_candidates(self, tmp_path, monkeypatch):
        target = tmp_path / "hand.onnx"
        target.write_bytes(b"")
        monkeypatch.setenv(MODEL_ENV_VAR, str(target))
        assert resolve_hand_model_path() == str(target)

    @pytest.mark.skipif(
        (SOURCE_ROOT / "models" / MODEL_FILENAME).exists(),
        reason="モデルが配置されると解決が成功するため")
    def test_source_tree_candidate_reported_when_unresolvable(
            self, monkeypatch):
        monkeypatch.delenv(MODEL_ENV_VAR, raising=False)
        assert resolve_hand_model_path().endswith(
            "models/" + MODEL_FILENAME)


@pytest.fixture()
def ros_context():
    import rclpy
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


class _RecordingPublisher:
    """購読者が 1 人いる Publisher の代替。

    実際の DDS 購読を待たせると取りこぼしが発生して
    購読者ゼロ短絡で変換前に返ってしまう。購読数を宣言的に固定して、
    コールバックが最後まで到達することを保証する。
    """

    def __init__(self) -> None:
        self.messages = []

    def get_subscription_count(self) -> int:
        return 1

    def publish(self, msg) -> None:
        self.messages.append(msg)


def _node_with(session_output, score=0.9, threshold=0.5):
    session = _FakeSession(session_output)
    det = HandLandmarkDetector(session=session, score_threshold=threshold)
    return HandLandmarksNode(detector=det), session


def _image_msg(height=480, width=640, encoding="bgr8", frame_id="camera_link"):
    msg = ImageMsg()
    msg.height = height
    msg.width = width
    msg.encoding = encoding
    msg.step = width * 3
    msg.data = np.zeros((height, width, 3), dtype=np.uint8).tobytes()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = 1234
    return msg


class TestNodeCallback:
    def test_hand_publishes_one_pose_per_landmark(self, ros_context):
        node, _ = _node_with(_landmarks_output(score=0.9))
        node._pub = _RecordingPublisher()
        try:
            node._image_callback(_image_msg())
        finally:
            node.destroy_node()
        published = node._pub.messages[0]
        assert len(published.poses) == 21
        first = published.poses[0]
        assert 0.0 <= first.position.x <= 640.0
        assert 0.0 <= first.position.y <= 480.0
        assert first.position.z == 0.0
        assert first.orientation.w == 1.0

    def test_unsupported_encoding_does_not_crash_callback(self, ros_context):
        """不正エンコーディングでも例外を呼叫出しへ出さないこと。"""
        node, session = _node_with(_landmarks_output(score=0.9))
        node._pub = _RecordingPublisher()
        try:
            node._image_callback(_image_msg(encoding="invalid_xyz"))
        finally:
            node.destroy_node()
        assert session.run_count == 0
        assert node._pub.messages == []

    def test_rate_limit_skips_second_frame(self, ros_context):
        node, session = _node_with(_landmarks_output(score=0.9))
        node._pub = _RecordingPublisher()
        try:
            node._min_interval = 1e9
            node._last_process_time = 0.0
            node._image_callback(_image_msg())
        finally:
            node.destroy_node()
        assert session.run_count == 0
        assert node._pub.messages == []

    def test_no_subscriber_short_circuits(self, ros_context):
        """購読者ゼロなら推論も配信もしないこと。"""
        node, session = _node_with(_landmarks_output(score=0.9))
        try:
            assert node._pub.get_subscription_count() == 0
            node._image_callback(_image_msg())
        finally:
            node.destroy_node()
        assert session.run_count == 0


class _FakeClock:
    """time.monotonic の差し替え。フレームの合間に時間を明示的に進める。

    実時間で sleep すると猶予 0.5 秒の検証が数秒安定する。初期値は 1024.0
    に取る。1024 台の倍数に 2 のべきの分数を足した値は 2 進小数で厳密に
    表せるので、境界値 (猶予ちょうど) の比較が丸め誤差で flaky にならない。
    """

    def __init__(self, start: float = 1024.0) -> None:
        self.now = start

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> float:
        self.now += seconds
        return self.now


@pytest.fixture()
def clock(monkeypatch):
    """ノードが読む時刻を固定し、フレーム単位で操作できるようにする。"""
    fake = _FakeClock()
    monkeypatch.setattr(hand_landmarks_node.time, "monotonic", fake)
    return fake


def _feed(node, clock, seconds):
    """seconds 進めた時刻で 1 フレーム処理する。"""
    clock.advance(seconds)
    node._image_callback(_image_msg())


def _grace_node(session_output, threshold=0.5):
    """猶予と消失だけを検証するノードを返す。

    _min_interval を 0 にするのは、レート制限が別テスト
    (test_rate_limit_skips_second_frame) の担当であり、ここでは固定の
    フレーム間隔で「脱落が何フレーム分许されるか」を数えたいから。
    """
    node, session = _node_with(session_output, threshold=threshold)
    node._min_interval = 0.0
    node._last_process_time = 0.0
    node._pub = _RecordingPublisher()
    return node, session


class TestNoHandIsSilent:
    """契約 (a): 手が映っていないフレームでは配信件数が増えないこと。"""

    def test_hand_timeout_sec_default(self, ros_context):
        node, _ = _node_with(_landmarks_output(score=0.9))
        try:
            assert node._hand_timeout_sec == pytest.approx(0.5)
        finally:
            node.destroy_node()

    def test_no_hand_publishes_nothing(self, ros_context, clock):
        """手が映っていないフレームでは何も publish しないこと。"""
        node, session = _grace_node(_landmarks_output(score=0.02))
        try:
            _feed(node, clock, 0.25)
        finally:
            node.destroy_node()
        assert session.run_count == 1
        assert node._pub.messages == []

    def test_never_seen_hand_never_publishes(self, ros_context, clock):
        """一度も手が映っていないなら、猶予がいくら経過しても無音。"""
        node, session = _grace_node(_landmarks_output(score=0.02))
        try:
            for _ in range(10):
                _feed(node, clock, 0.5)
        finally:
            node.destroy_node()
        assert session.run_count == 10
        assert node._pub.messages == []

    def test_missed_frames_within_grace_are_silent(self, ros_context, clock):
        """猶予内の連続脱落は何も出さないこと。"""
        node, session = _grace_node(_landmarks_output(score=0.9))
        try:
            _feed(node, clock, 0.0)
            session.set_output(_landmarks_output(score=0.01))
            _feed(node, clock, 0.125)
            _feed(node, clock, 0.125)
        finally:
            node.destroy_node()
        assert session.run_count == 3
        assert len(node._pub.messages) == 1

    def test_cleared_pose_array_after_timeout(self, ros_context, clock):
        """猶予切れで空配列を 1 度だけ出すこと。"""
        node, session = _grace_node(_landmarks_output(score=0.9))
        try:
            _feed(node, clock, 0.0)
            session.set_output(_landmarks_output(score=0.01))
            _feed(node, clock, 0.75)
        finally:
            node.destroy_node()
        assert len(node._pub.messages) == 2
        cleared = node._pub.messages[1]
        assert len(cleared.poses) == 0
        assert cleared.header.frame_id == "camera_link"
        assert cleared.header.stamp.sec == 1234

    def test_count_does_not_grow_while_hand_absent(self, ros_context, clock):
        """手が消えたあとも配信件数が増えないこと。不変条件の本体。"""
        node, session = _grace_node(_landmarks_output(score=0.9))
        try:
            _feed(node, clock, 0.0)
            session.set_output(_landmarks_output(score=0.01))
            _feed(node, clock, 0.75)
            for _ in range(20):
                _feed(node, clock, 0.125)
        finally:
            node.destroy_node()
        assert session.run_count == 22
        assert len(node._pub.messages) == 2

    def test_grace_boundary_is_inclusive(self, ros_context, clock):
        """猶予ちょうどで確定すること (未満なら保持)。"""
        node, session = _grace_node(_landmarks_output(score=0.9))
        try:
            _feed(node, clock, 0.0)
            session.set_output(_landmarks_output(score=0.01))
            _feed(node, clock, 0.5)
        finally:
            node.destroy_node()
        assert len(node._pub.messages) == 2

    def test_reacquired_hand_rearms_the_clear(self, ros_context, clock):
        """再検出の後は、次の消失でまた 1 度だけ通知すること。"""
        node, session = _grace_node(_landmarks_output(score=0.9))
        try:
            _feed(node, clock, 0.0)
            session.set_output(_landmarks_output(score=0.01))
            _feed(node, clock, 0.75)
            session.set_output(_landmarks_output(score=0.9))
            _feed(node, clock, 0.125)
            session.set_output(_landmarks_output(score=0.01))
            _feed(node, clock, 0.75)
        finally:
            node.destroy_node()
        assert len(node._pub.messages) == 4
        assert len(node._pub.messages[2].poses) == 21
        assert len(node._pub.messages[3].poses) == 0

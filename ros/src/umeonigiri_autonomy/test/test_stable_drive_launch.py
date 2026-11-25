"""新しいヘッドレス統合 launch と安定性テスト CLI の契約テスト。"""
import importlib.util
import json
import os
from pathlib import Path

from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

PKG = Path(__file__).resolve().parents[1]
LAUNCH = PKG / "launch"


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def unwrap(value):
    """launch の TextSubstitution / tuple を人間が読める値へ戻す。"""
    while hasattr(value, "text"):
        value = value.text
    if isinstance(value, (list, tuple)):
        if len(value) == 1:
            return unwrap(value[0])
        return tuple(unwrap(item) for item in value)
    if isinstance(value, str) and "\n...\n" in value:
        return value.split("\n...\n", 1)[0]
    return value


def launch_file_name(entity):
    source = entity.__dict__["_IncludeLaunchDescription__launch_description_source"]
    return os.path.basename(str(unwrap(source.__dict__["_LaunchDescriptionSource__location"])))


def included_names(module):
    return {launch_file_name(entity)
            for entity in module.generate_launch_description().entities
            if isinstance(entity, IncludeLaunchDescription)}


def test_stable_drive_launch_includes_headless_navigation_and_web():
    module = load(LAUNCH / "stable_drive_vision.launch.py", "stable_drive_launch")
    names = included_names(module)
    assert {
        "robot_state_publisher.launch.py",
        "simulation.launch.py",
        "slam.launch.py",
        "navigation.launch.py",
        "umeonigiri_webnav.launch.py",
    } <= names
    nodes = [entity for entity in module.generate_launch_description().entities
             if isinstance(entity, Node)]
    assert any("yolo_grasp_node" in repr(node._Node__node_executable) for node in nodes)


def test_existing_headless_launch_is_unchanged_in_shape():
    module = load(LAUNCH / "arm_vision_slam.launch.py", "arm_vision_slam_launch")
    names = included_names(module)
    assert {"simulation.launch.py", "slam.launch.py"} <= names
    assert "navigation.launch.py" not in names
    nodes = [entity for entity in module.generate_launch_description().entities
             if isinstance(entity, Node)]
    assert all("webnav" not in repr(node) for node in nodes)


def test_shared_vision_node_disables_detection_gate():
    module = load(LAUNCH / "vision_node_common.py", "vision_node_common")
    node = module.arm_vision_node()
    assert "yolo_grasp_node" in repr(node._Node__node_executable)
    parameters = {}
    for group in node._Node__parameters:
        for key, value in group.items():
            parameters[unwrap(key)] = unwrap(value)
    assert parameters["detection_fps"] == 0.0
    assert parameters["image_topic"] == "/arm_camera/image_raw"


def test_default_goals_are_normalized(monkeypatch, tmp_path):
    """既定の 2 要素 GOALS でも 3 要素として展開され走行できる。"""
    script = load(PKG / "scripts" / "stability_test.py", "stability_default")

    class Response:
        def __init__(self, payload):
            self.payload = payload

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

        def read(self):
            return json.dumps(self.payload).encode()

    def fake_urlopen(request, timeout=5):
        endpoint = str(getattr(request, "full_url", request)).rsplit("/", 1)[-1]
        if endpoint == "status":
            return Response({"state": "succeeded"})
        if endpoint == "pose":
            return Response({"has_pose": True, "x": 0.0, "y": 0.0})
        return Response({"ok": True})

    monkeypatch.setattr(script.urllib.request, "urlopen", fake_urlopen)
    monkeypatch.setattr(script.time, "sleep", lambda seconds: None)
    out = tmp_path / "stability.json"
    code = script.main(["--base", "http://127.0.0.1:1", "--out", str(out),
                        "--timeout", "0.1", "--post-delay", "0"])
    assert code == 0
    assert len(json.loads(out.read_text(encoding="utf-8"))) == len(script.GOALS)


def test_stability_test_writes_custom_output(monkeypatch, tmp_path):
    script = load(PKG / "scripts" / "stability_test.py", "stability_script")

    class Response:
        def __init__(self, payload):
            self.payload = payload

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

        def read(self):
            return json.dumps(self.payload).encode()

    def fake_urlopen(request, timeout=5):
        url = getattr(request, "full_url", request)
        endpoint = str(url).rsplit("/", 1)[-1]
        if endpoint == "status":
            return Response({"state": "succeeded"})
        if endpoint == "pose":
            return Response({"has_pose": True, "x": 1.0, "y": 2.0})
        return Response({"ok": True})

    monkeypatch.setattr(script.urllib.request, "urlopen", fake_urlopen)
    monkeypatch.setattr(script.time, "sleep", lambda seconds: None)
    out = tmp_path / "stability.json"
    goals = tmp_path / "goals.json"
    goals.write_text("[[1.0, 2.0], [1.0, 2.0]]", encoding="utf-8")
    code = script.main(["--base", "http://127.0.0.1:1", "--out", str(out),
                        "--goals", str(goals), "--timeout", "0.1", "--post-delay", "0"])
    assert code == 0
    results = json.loads(out.read_text(encoding="utf-8"))
    assert len(results) == 2
    assert all(result["state"] == "succeeded" and result["err_m"] == 0.0
               for result in results)

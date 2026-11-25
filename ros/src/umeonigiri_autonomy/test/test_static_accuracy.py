"""Contracts for the static-accuracy regression helper and goal CLI."""
import importlib.util
import json
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
REPO = PKG.parents[2]


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_iou_and_evaluation_are_independent_of_model_output():
    helper = load(REPO / "docs" / "perf" / "measure_static_accuracy.py", "static_accuracy")
    a = {"x0": 0, "y0": 0, "x1": 10, "y1": 10}
    b = {"x0": 5, "y0": 0, "x1": 15, "y1": 10}
    assert abs(helper.iou(a, a) - 1.0) < 1e-6
    assert abs(helper.iou(a, b) - 1.0 / 3.0) < 1e-6
    predictions = [dict(a, class_id=0, score=0.9)]
    gt = [["person", 0, 0, 10, 10]]
    result = helper.evaluate(predictions, gt, ["person", "bus"])
    assert result["class_accuracy"] == 1.0
    assert result["false_positive_count"] == 0
    wrong = helper.evaluate([dict(a, class_id=1, score=0.9)], gt, ["person", "bus"])
    assert wrong["class_accuracy"] == 0.0
    assert wrong["matched_count"] == 1  # IoU だけ独立に見る


def test_static_accuracy_evidence_declares_known_object_regression():
    helper = load(REPO / "docs" / "perf" / "measure_static_accuracy.py", "static_accuracy")
    assert "Known-object regression" in helper.GROUND_TRUTH or \
        "KNOWN-OBJECT REGRESSION" in helper.__doc__
    assert len(helper.GROUND_TRUTH["bus.jpg"]) >= 2  # person と bus
    assert {gt[0] for gt in helper.GROUND_TRUTH["bus.jpg"]} >= {"person", "bus"}


def test_stability_goal_file_supports_yaw(monkeypatch, tmp_path):
    script = load(PKG / "scripts" / "stability_test.py", "stability_yaw")

    class Response:
        def __init__(self, payload):
            self.payload = payload

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

        def read(self):
            return json.dumps(self.payload).encode()

    posted = []

    def fake_urlopen(request, timeout=5):
        url = getattr(request, "full_url", request)
        endpoint = str(url).rsplit("/", 1)[-1]
        if endpoint == "status":
            return Response({"state": "succeeded"})
        if endpoint == "pose":
            return Response({"has_pose": True, "x": 1.0, "y": 2.0})
        if endpoint == "nav":
            posted.append(json.loads(request.data))
        return Response({"ok": True})

    monkeypatch.setattr(script.urllib.request, "urlopen", fake_urlopen)
    monkeypatch.setattr(script.time, "sleep", lambda seconds: None)
    goals = tmp_path / "goals.json"
    goals.write_text("[[1.0, 2.0], [1.0, 2.0, 1.57]]", encoding="utf-8")
    out = tmp_path / "stability.json"
    code = script.main(["--base", "http://127.0.0.1:1", "--out", str(out),
                        "--goals", str(goals), "--timeout", "0.1", "--post-delay", "0"])
    assert code == 0
    assert posted == [
        {"waypoints": [{"x": 1.0, "y": 2.0, "yaw": 0.0}]},
        {"waypoints": [{"x": 1.0, "y": 2.0, "yaw": 1.57}]},
    ]
    assert json.loads(out.read_text(encoding="utf-8"))[1]["yaw"] == 1.57


def test_nav_override_is_scoped_and_does_not_change_default_params():
    nav = load(PKG / "launch" / "navigation.launch.py", "nav_launch")
    names = [str(entity.name) for entity in nav.generate_launch_description().entities
             if type(entity).__name__ == "DeclareLaunchArgument"]
    assert "nav_overlay" in names
    overlay = PKG / "config" / "stable_drive_nav_overlay.yaml"
    assert overlay.is_file()
    assert "0.9" in overlay.read_text(encoding="utf-8")
    default = PKG / "config" / "mapping_nav_params.yaml"
    assert "max_accel: [1.0, 0.0, 3.2]" in default.read_text(encoding="utf-8")
    stable = (PKG / "launch" / "stable_drive_vision.launch.py").read_text(encoding="utf-8")
    assert "stable_drive_nav_overlay.yaml" in stable
    assert "nav_overlay" in stable
    assert "use_smoother_path" in stable  # controller 出力を smoother へ回す
    assert "'use_smoother_path', default_value='false'" in (
        (PKG / "launch" / "navigation.launch.py").read_text(encoding="utf-8"))

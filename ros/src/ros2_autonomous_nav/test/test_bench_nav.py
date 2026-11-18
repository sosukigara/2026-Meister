import json
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "scripts"))

from bench_nav import load_goals, validate_goals  # noqa: E402


def _write(tmp_path, doc):
    p = tmp_path / "goals.json"
    p.write_text(json.dumps(doc))
    return str(p)


def test_load_goals_reads_world_and_points(tmp_path):
    doc = {"world": "warehouse", "repeats": 2,
           "goals": [{"x": 1.0, "y": 2.0, "yaw": 0.0}]}
    out = load_goals(_write(tmp_path, doc))
    assert out["world"] == "warehouse"
    assert out["repeats"] == 2
    assert out["goals"][0]["x"] == pytest.approx(1.0)


def test_validate_rejects_goal_inside_obstacle():
    doc = {"world": "warehouse", "repeats": 1,
           "goals": [{"x": 1.0, "y": 2.0, "yaw": 0.0}]}
    problems = validate_goals(doc, lambda x, y: False)
    assert problems == ["goal 0 (1.0, 2.0) is not in free space"]


def test_validate_accepts_free_space_goals():
    doc = {"world": "warehouse", "repeats": 1,
           "goals": [{"x": 1.0, "y": 2.0, "yaw": 0.0}]}
    assert validate_goals(doc, lambda x, y: True) == []


def test_validate_rejects_empty_goal_list():
    doc = {"world": "warehouse", "repeats": 1, "goals": []}
    assert validate_goals(doc, lambda x, y: True) == ["no goals defined"]

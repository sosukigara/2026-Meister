import math
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "scripts"))

from bench_metrics import (  # noqa: E402
    min_footprint_clearance,
    path_length,
    success_rate,
    summarize,
)


def test_path_length_sums_segments():
    assert path_length([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)]) == pytest.approx(2.0)


def test_path_length_of_single_pose_is_zero():
    assert path_length([(0.0, 0.0, 0.0)]) == pytest.approx(0.0)


def test_clearance_is_distance_to_nearest_occupied_cell():
    poses = [(0.0, 0.0, 0.0)]
    assert min_footprint_clearance(poses, [(1.0, 0.0)], 0.30, 0.20) == pytest.approx(0.70)


def test_clearance_accounts_for_yaw():
    poses = [(0.0, 0.0, math.pi / 2)]
    assert min_footprint_clearance(poses, [(0.0, 0.95)], 0.30, 0.20) == pytest.approx(0.65)


def test_clearance_is_infinite_without_obstacles():
    assert min_footprint_clearance([(0.0, 0.0, 0.0)], [], 0.30, 0.20) == math.inf


def test_success_rate_counts_successes():
    assert success_rate([True, True, False, True]) == pytest.approx(0.75)


def test_summarize_reports_counts_and_worst_case():
    runs = [
        {"success": True, "duration_s": 10.0, "path_length_m": 5.0,
         "final_error_xy": 0.05, "final_error_yaw": 0.02, "min_clearance_m": 0.50},
        {"success": False, "duration_s": 30.0, "path_length_m": 9.0,
         "final_error_xy": 0.40, "final_error_yaw": 0.30, "min_clearance_m": 0.08},
    ]
    out = summarize(runs)
    assert out["total"] == 2
    assert out["success_rate"] == pytest.approx(0.5)
    assert out["max_final_error_xy"] == pytest.approx(0.40)
    assert out["min_clearance_m"] == pytest.approx(0.08)

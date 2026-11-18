import math
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "scripts"))

from bench_metrics import (  # noqa: E402
    cell_cost,
    contact_count,
    interval_stats,
    min_footprint_clearance,
    path_length,
    path_length_ratio,
    success_rate,
    summarize,
)


class _Vec:
    def __init__(self, x=0.0, y=0.0):
        self.x, self.y = x, y


class _Info:
    def __init__(self, resolution, width, height, ox, oy):
        self.resolution, self.width, self.height = resolution, width, height
        self.origin = type("_Pose", (), {"position": _Vec(ox, oy)})()


class _Grid:
    def __init__(self, data, width, height, resolution=0.5, ox=0.0, oy=0.0):
        self.data = data
        self.info = _Info(resolution, width, height, ox, oy)


def test_path_length_sums_segments():
    assert path_length([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)]) == pytest.approx(2.0)


def test_path_length_of_single_pose_is_zero():
    assert path_length([(0.0, 0.0, 0.0)]) == pytest.approx(0.0)


def test_clearance_is_distance_to_nearest_occupied_cell():
    assert min_footprint_clearance([(0.0, 0.0, 0.0)], [(1.0, 0.0)], 0.30, 0.20) == pytest.approx(0.70)


def test_clearance_accounts_for_yaw():
    assert min_footprint_clearance([(0.0, 0.0, math.pi / 2)], [(0.0, 0.95)],
                                   0.30, 0.20) == pytest.approx(0.65)


def test_clearance_uses_euclidean_distance_on_diagonal():
    # hypot(0.70, 0.80) — an axis-max implementation would return 0.80
    assert min_footprint_clearance([(0.0, 0.0, 0.0)], [(1.0, 1.0)],
                                   0.30, 0.20) == pytest.approx(1.0630145)


def test_clearance_is_zero_for_a_point_inside_the_footprint():
    assert min_footprint_clearance([(0.0, 0.0, 0.0)], [(0.10, 0.10)],
                                   0.30, 0.20) == pytest.approx(0.0)


def test_clearance_picks_the_nearest_of_several_cells():
    # (0, 0.5) -> 0.30 ; (2, 0) -> 1.70
    assert min_footprint_clearance([(0.0, 0.0, 0.0)], [(2.0, 0.0), (0.0, 0.5)],
                                   0.30, 0.20) == pytest.approx(0.30)


def test_clearance_takes_the_worst_pose_not_the_first():
    # first pose: 10.20 ; second pose: 0.20
    assert min_footprint_clearance([(0.0, 0.0, 0.0), (10.0, 0.0, 0.0)], [(10.5, 0.0)],
                                   0.30, 0.20) == pytest.approx(0.20)


def test_clearance_is_infinite_without_obstacles():
    assert min_footprint_clearance([(0.0, 0.0, 0.0)], [], 0.30, 0.20) == math.inf


def test_contact_count_counts_poses_closer_than_threshold():
    poses = [(0.0, 0.0, 0.0), (5.0, 0.0, 0.0)]
    cells = [(0.31, 0.0)]  # clearance 0.01 from the first pose only
    assert contact_count(poses, cells, 0.30, 0.20, threshold=0.02) == 1


def test_contact_count_is_zero_when_everything_is_far():
    assert contact_count([(0.0, 0.0, 0.0)], [(1.0, 0.0)], 0.30, 0.20, threshold=0.02) == 0


def test_success_rate_counts_successes():
    assert success_rate([True, True, False, True]) == pytest.approx(0.75)


def test_success_rate_of_no_runs_is_zero():
    assert success_rate([]) == 0.0


def test_path_length_ratio_divides_travelled_by_planned():
    assert path_length_ratio(6.0, 5.0) == pytest.approx(1.2)


def test_path_length_ratio_is_infinite_without_a_plan():
    assert path_length_ratio(6.0, 0.0) == math.inf


def test_interval_stats_reports_hz_and_p99():
    times = [i * 0.05 for i in range(21)]
    stats = interval_stats(times)
    assert stats["effective_hz"] == pytest.approx(20.0)
    assert stats["p99_ms"] == pytest.approx(50.0)


def test_interval_stats_p99_catches_a_stall():
    times = [i * 0.05 for i in range(20)] + [0.95 + 0.20]  # last gap 0.20 s
    stats = interval_stats(times)
    assert stats["p99_ms"] == pytest.approx(200.0)


def test_interval_stats_of_short_series_is_empty():
    assert interval_stats([0.0]) == {"effective_hz": 0.0, "p99_ms": 0.0}


def test_cell_cost_reads_the_grid_cell_under_a_point():
    grid = _Grid([0, 100, 0, 254], width=2, height=2, resolution=0.5, ox=0.0, oy=0.0)
    assert cell_cost(grid, 0.1, 0.1) == 0
    assert cell_cost(grid, 0.6, 0.1) == 100
    assert cell_cost(grid, 0.1, 0.6) == 0
    assert cell_cost(grid, 0.6, 0.6) == 254


def test_cell_cost_outside_the_grid_is_none():
    grid = _Grid([0, 0, 0, 0], width=2, height=2, resolution=0.5)
    assert cell_cost(grid, -0.1, 0.1) is None
    assert cell_cost(grid, 0.1, 5.0) is None


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
    assert out["max_final_error_yaw"] == pytest.approx(0.30)
    assert out["min_clearance_m"] == pytest.approx(0.08)
    assert out["mean_duration_s"] == pytest.approx(20.0)


def test_summarize_ignores_runs_without_measurements():
    runs = [
        {"success": False, "duration_s": 0.0, "path_length_m": 0.0,
         "final_error_xy": None, "final_error_yaw": None, "min_clearance_m": None},
        {"success": True, "duration_s": 10.0, "path_length_m": 5.0,
         "final_error_xy": 0.05, "final_error_yaw": 0.02, "min_clearance_m": 0.50},
    ]
    out = summarize(runs)
    assert out["total"] == 2
    assert out["measured"] == 1
    assert out["min_clearance_m"] == pytest.approx(0.50)
    assert out["max_final_error_xy"] == pytest.approx(0.05)


def test_summarize_of_no_runs_is_all_none():
    out = summarize([])
    assert out["total"] == 0
    assert out["success_rate"] == 0.0
    assert out["min_clearance_m"] is None

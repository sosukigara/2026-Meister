"""ナビゲーション評価の指標。ROS に依存しない純関数だけを置く。

設計書 docs/superpowers/specs/2026-11-18-navigation-design.md の §8.1 が
定義する合格基準を、そのまま計算できる形にしたもの。
"""

import math
from typing import Dict, Iterable, Sequence, Tuple

Pose = Sequence[float]  # (x, y, yaw)
Cell = Tuple[float, float]


def path_length(poses: Iterable[Pose]) -> float:
    """軌跡の累積距離 [m]。点が1つ以下なら 0。"""
    points = [(float(p[0]), float(p[1])) for p in poses]
    total = 0.0
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def min_footprint_clearance(
    poses: Iterable[Pose],
    occupied: Sequence[Cell],
    half_length: float,
    half_width: float,
) -> float:
    """機体矩形と障害物セルの最小距離 [m]。障害物が無ければ inf。

    矩形は機体座標で x 方向に half_length、y 方向に half_width。
    各 pose で世界座標の点を機体座標へ回してから、軸ごのはみ出し量を合成する。
    """
    best = math.inf
    for pose in poses:
        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        for ox, oy in occupied:
            dx, dy = float(ox) - x, float(oy) - y
            bx = dx * cos_yaw + dy * sin_yaw
            by = -dx * sin_yaw + dy * cos_yaw
            over_x = max(0.0, abs(bx) - half_length)
            over_y = max(0.0, abs(by) - half_width)
            best = min(best, math.hypot(over_x, over_y))
    return best


def success_rate(results: Sequence[bool]) -> float:
    """成功率 [0, 1]。"""
    if not results:
        return 0.0
    return sum(1 for r in results if r) / len(results)


def summarize(runs: Sequence[Dict[str, float]]) -> Dict[str, float]:
    """run の並びから合否に使う代表値を出す。"""
    return {
        "total": len(runs),
        "success_rate": success_rate([bool(r["success"]) for r in runs]),
        "max_final_error_xy": max(float(r["final_error_xy"]) for r in runs),
        "max_final_error_yaw": max(float(r["final_error_yaw"]) for r in runs),
        "min_clearance_m": min(float(r["min_clearance_m"]) for r in runs),
        "mean_duration_s": sum(float(r["duration_s"]) for r in runs) / len(runs),
    }

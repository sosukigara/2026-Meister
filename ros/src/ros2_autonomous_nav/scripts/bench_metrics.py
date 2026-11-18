"""ナビゲーション評価の指標。ROS に依存しない純関数だけを置く。

設計書 docs/superpowers/specs/2026-11-18-navigation-design.md の §8.1 が
定義する合格基準を、そのまま計算できる形にしたもの。

障害物セルは「地図の占有セル」を渡すこと。コストマップの /costmap は
0..100 に変換済みで膨張リングを含むため、そのまま渡すと距離が実際より
小さく（安全側に）出る。呼び出し側は costmap_raw の LETHAL 以上を使う。
"""

import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

Pose = Sequence[float]  # (x, y, yaw)
Cell = Tuple[float, float]


def path_length(poses: Iterable[Pose]) -> float:
    """軌跡の累積距離 [m]。点が1つ以下なら 0。"""
    points = [(float(p[0]), float(p[1])) for p in poses]
    total = 0.0
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def _clearance_at(x: float, y: float, yaw: float, occupied: Sequence[Cell],
                  half_length: float, half_width: float) -> float:
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    best = math.inf
    for ox, oy in occupied:
        dx, dy = float(ox) - x, float(oy) - y
        bx = dx * cos_yaw + dy * sin_yaw
        by = -dx * sin_yaw + dy * cos_yaw
        over_x = max(0.0, abs(bx) - half_length)
        over_y = max(0.0, abs(by) - half_width)
        best = min(best, math.hypot(over_x, over_y))
    return best


def min_footprint_clearance(poses: Iterable[Pose], occupied: Sequence[Cell],
                            half_length: float, half_width: float) -> float:
    """機体矩形と障害物セルの最小距離 [m]。障害物が無ければ inf。

    矩形は機体座標で x 方向に half_length、y 方向に half_width。
    各 pose で世界座標の点を機体座標へ回してから、軸ごとの はみ出し量を合成する。
    """
    best = math.inf
    for pose in poses:
        best = min(best, _clearance_at(float(pose[0]), float(pose[1]), float(pose[2]),
                                       occupied, half_length, half_width))
    return best


def contact_count(poses: Iterable[Pose], occupied: Sequence[Cell],
                  half_length: float, half_width: float,
                  threshold: float = 0.02) -> int:
    """機体矩形が障害物から threshold 未満まで近づいた pose の数。"""
    return sum(
        1 for pose in poses
        if _clearance_at(float(pose[0]), float(pose[1]), float(pose[2]),
                         occupied, half_length, half_width) < threshold
    )


def path_length_ratio(travelled: float, planned: float) -> float:
    """走行距離 ÷ 計画経路長。計画が無ければ inf。"""
    if planned <= 0.0:
        return math.inf
    return travelled / planned


def interval_stats(times: Sequence[float]) -> Dict[str, float]:
    """連続メッセージの時刻列から実行周波数と周期 p99 [ms] を出す。"""
    if len(times) < 2:
        return {"effective_hz": 0.0, "p99_ms": 0.0}
    intervals = [b - a for a, b in zip(times, times[1:]) if b - a > 0.0]
    if not intervals:
        return {"effective_hz": 0.0, "p99_ms": 0.0}
    total = sum(intervals)
    ordered = sorted(intervals)
    index = min(len(ordered) - 1, int(round(0.99 * (len(ordered) - 1))))
    return {
        "effective_hz": len(intervals) / total if total > 0.0 else 0.0,
        "p99_ms": ordered[index] * 1000.0,
    }


def cell_cost(grid, x: float, y: float) -> Optional[int]:
    """OccupancyGrid のセル値。-1 は未知、範囲外は None。

    grid は .data / .info.resolution / .info.width / .info.height /
    .info.origin.position.{x,y} を持つもの（nav_msgs/OccupancyGrid と同じ形）。
    """
    info = grid.info
    col = math.floor((x - info.origin.position.x) / info.resolution)
    row = math.floor((y - info.origin.position.y) / info.resolution)
    if col < 0 or row < 0 or col >= info.width or row >= info.height:
        return None
    return int(grid.data[row * info.width + col])


def success_rate(results: Sequence[bool]) -> float:
    """成功率 [0, 1]。"""
    if not results:
        return 0.0
    return sum(1 for r in results if r) / len(results)


def _measured(runs: Sequence[Dict[str, object]]) -> List[Dict[str, object]]:
    """測定値が揃っている run だけを返す（失敗や即時終了の run を除く）。"""
    return [r for r in runs if r.get("min_clearance_m") is not None]


def summarize(runs: Sequence[Dict[str, object]]) -> Dict[str, object]:
    """run の並びから合否に使う代表値を出す。測定不能な run は集計から除く。"""
    measured = _measured(runs)
    if not measured:
        return {
            "total": len(runs), "measured": 0, "success_rate": success_rate(
                [bool(r["success"]) for r in runs]),
            "max_final_error_xy": None, "max_final_error_yaw": None,
            "min_clearance_m": None, "mean_duration_s": None,
        }
    return {
        "total": len(runs),
        "measured": len(measured),
        "success_rate": success_rate([bool(r["success"]) for r in runs]),
        "max_final_error_xy": max(float(r["final_error_xy"]) for r in measured),
        "max_final_error_yaw": max(float(r["final_error_yaw"]) for r in measured),
        "min_clearance_m": min(float(r["min_clearance_m"]) for r in measured),
        "mean_duration_s": sum(float(r["duration_s"]) for r in measured) / len(measured),
    }

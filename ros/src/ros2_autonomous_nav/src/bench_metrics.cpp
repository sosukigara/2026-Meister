// SPDX-License-Identifier: MIT
#include "ros2_autonomous_nav/bench_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <limits>

namespace ros2_autonomous_nav {
namespace {

double Hypot(double a, double b) { return std::hypot(a, b); }

}  // namespace

double PathLength(const std::vector<Pose2>& poses) {
  double total = 0.0;
  for (std::size_t i = 1; i < poses.size(); ++i) {
    total += Hypot(poses[i].x - poses[i - 1].x, poses[i].y - poses[i - 1].y);
  }
  return total;
}

double ClearanceAt(const Pose2& pose, const std::vector<Cell>& occupied,
                   double half_length, double half_width) {
  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);
  double best = std::numeric_limits<double>::infinity();
  for (const auto& [ox, oy] : occupied) {
    const double dx = ox - pose.x;
    const double dy = oy - pose.y;
    // 世界座標の点を機体座標へ回す。
    const double body_x = dx * cos_yaw + dy * sin_yaw;
    const double body_y = -dx * sin_yaw + dy * cos_yaw;
    const double over_x = std::max(0.0, std::abs(body_x) - half_length);
    const double over_y = std::max(0.0, std::abs(body_y) - half_width);
    best = std::min(best, Hypot(over_x, over_y));
  }
  return best;
}

double MinFootprintClearance(const std::vector<Pose2>& poses,
                             const std::vector<Cell>& occupied,
                             double half_length, double half_width) {
  double best = std::numeric_limits<double>::infinity();
  for (const auto& pose : poses) {
    best = std::min(best, ClearanceAt(pose, occupied, half_length, half_width));
  }
  return best;
}

int ContactCount(const std::vector<Pose2>& poses, const std::vector<Cell>& occupied,
                 double half_length, double half_width, double threshold) {
  int count = 0;
  for (const auto& pose : poses) {
    if (ClearanceAt(pose, occupied, half_length, half_width) < threshold) {
      ++count;
    }
  }
  return count;
}

double PathLengthRatio(double travelled, double planned) {
  if (planned <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return travelled / planned;
}

IntervalStats ComputeIntervalStats(const std::vector<double>& times) {
  IntervalStats stats;
  if (times.size() < 2) {
    return stats;
  }
  std::vector<double> intervals;
  intervals.reserve(times.size() - 1);
  for (std::size_t i = 1; i < times.size(); ++i) {
    const double delta = times[i] - times[i - 1];
    if (delta > 0.0) {
      intervals.push_back(delta);
    }
  }
  if (intervals.empty()) {
    return stats;
  }
  double total = 0.0;
  for (const double value : intervals) {
    total += value;
  }
  std::sort(intervals.begin(), intervals.end());
  const auto last = static_cast<double>(intervals.size() - 1);
  const auto index = static_cast<std::size_t>(
      std::min(last, std::round(0.99 * last)));
  stats.effective_hz = total > 0.0 ? static_cast<double>(intervals.size()) / total : 0.0;
  stats.p99_ms = intervals[index] * 1000.0;
  return stats;
}

std::optional<int> CellCost(const GridView& grid, double x, double y) {
  if (grid.resolution <= 0.0 || grid.width <= 0 || grid.height <= 0) {
    return std::nullopt;
  }
  const int col = static_cast<int>(std::floor((x - grid.origin_x) / grid.resolution));
  const int row = static_cast<int>(std::floor((y - grid.origin_y) / grid.resolution));
  if (col < 0 || row < 0 || col >= grid.width || row >= grid.height) {
    return std::nullopt;
  }
  return static_cast<int>(grid.data[static_cast<std::size_t>(row) * grid.width + col]);
}

namespace {

double Percentile(std::vector<double> values, double q) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const auto last = static_cast<double>(values.size() - 1);
  const auto index = static_cast<std::size_t>(std::min(last, std::round(q * last)));
  return values[index];
}

}  // namespace

std::vector<std::string> ValidateGoals(const std::vector<Goal>& goals,
                                       const GridView& cost_grid, int free_cost_max) {
  std::vector<std::string> problems;
  if (goals.empty()) {
    problems.emplace_back("no goals defined");
    return problems;
  }
  for (std::size_t i = 0; i < goals.size(); ++i) {
    const auto cost = CellCost(cost_grid, goals[i].x, goals[i].y);
    if (!cost.has_value() || *cost < 0 || *cost >= free_cost_max) {
      char text[128];
      std::snprintf(text, sizeof(text), "goal %zu (%.1f, %.1f) is not in free space", i,
                    goals[i].x, goals[i].y);
      problems.emplace_back(text);
    }
  }
  return problems;
}

double SuccessRate(const std::vector<bool>& results) {
  if (results.empty()) {
    return 0.0;
  }
  const auto successes = std::count(results.begin(), results.end(), true);
  return static_cast<double>(successes) / static_cast<double>(results.size());
}

RunSummary Summarize(const std::vector<RunRecord>& runs) {
  RunSummary summary;
  summary.total = static_cast<int>(runs.size());
  std::vector<bool> results;
  results.reserve(runs.size());
  for (const auto& run : runs) {
    results.push_back(run.success);
  }
  summary.success_rate = SuccessRate(results);

  double max_xy = -std::numeric_limits<double>::infinity();
  double max_yaw = -std::numeric_limits<double>::infinity();
  double min_clearance = std::numeric_limits<double>::infinity();
  double duration_sum = 0.0;
  int measured = 0;
  for (const auto& run : runs) {
    if (!run.min_clearance_m.has_value()) {
      continue;
    }
    ++measured;
    duration_sum += run.duration_s;
    min_clearance = std::min(min_clearance, *run.min_clearance_m);
    if (run.final_error_xy.has_value()) {
      max_xy = std::max(max_xy, *run.final_error_xy);
    }
    if (run.final_error_yaw.has_value()) {
      max_yaw = std::max(max_yaw, *run.final_error_yaw);
    }
  }
  std::vector<double> planner_latencies;
  double max_control_p99 = -std::numeric_limits<double>::infinity();
  for (const auto& run : runs) {
    if (run.planner_latency_ms.has_value()) {
      planner_latencies.push_back(*run.planner_latency_ms);
    }
    if (run.control_p99_ms.has_value()) {
      max_control_p99 = std::max(max_control_p99, *run.control_p99_ms);
    }
  }
  if (!planner_latencies.empty()) {
    summary.planner_latency_p95_ms = Percentile(planner_latencies, 0.95);
  }
  if (!std::isinf(max_control_p99)) {
    summary.max_control_p99_ms = max_control_p99;
  }

  summary.measured = measured;
  if (measured > 0) {
    summary.min_clearance_m = min_clearance;
    summary.mean_duration_s = duration_sum / measured;
    if (!std::isinf(max_xy)) {
      summary.max_final_error_xy = max_xy;
    }
    if (!std::isinf(max_yaw)) {
      summary.max_final_error_yaw = max_yaw;
    }
  }
  return summary;
}

std::vector<Goal> SelectFreeGoals(const GridView& free_grid,
                                  const std::vector<Cell>& occupied, int count,
                                  double min_spacing, double min_clearance,
                                  double half_length, double half_width) {
  std::vector<Goal> selected;
  if (count <= 0) {
    return selected;
  }
  for (int row = 0; row < free_grid.height; ++row) {
    for (int col = 0; col < free_grid.width; ++col) {
      const int value = free_grid.data[static_cast<std::size_t>(row) * free_grid.width + col];
      if (value < 0 || value >= kFreeCostMax) {
        continue;
      }
      const double x = free_grid.origin_x + (static_cast<double>(col) + 0.5) * free_grid.resolution;
      const double y = free_grid.origin_y + (static_cast<double>(row) + 0.5) * free_grid.resolution;
      bool too_close = false;
      for (const auto& chosen : selected) {
        if (Hypot(x - chosen.x, y - chosen.y) < min_spacing) {
          too_close = true;
          break;
        }
      }
      if (too_close) {
        continue;
      }
      if (ClearanceAt(Pose2{x, y, 0.0}, occupied, half_length, half_width) < min_clearance) {
        continue;
      }
      selected.push_back(Goal{x, y, 0.0});
      if (static_cast<int>(selected.size()) >= count) {
        return selected;
      }
    }
  }
  return selected;
}

}  // namespace ros2_autonomous_nav

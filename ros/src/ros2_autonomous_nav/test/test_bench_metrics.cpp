// SPDX-License-Identifier: MIT
/// bench_metrics の単体テスト。ROS グラフも rclcpp::init も要らない。
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "ros2_autonomous_nav/bench_metrics.hpp"

namespace ros2_autonomous_nav {
namespace {

constexpr double kPi = 3.14159265358979323846;

GridView MakeGrid(int width, int height, double resolution, double origin_x,
                  double origin_y, int16_t value) {
  GridView grid;
  grid.width = width;
  grid.height = height;
  grid.resolution = resolution;
  grid.origin_x = origin_x;
  grid.origin_y = origin_y;
  grid.data.assign(static_cast<std::size_t>(width) * height, value);
  return grid;
}

TEST(BenchMetrics, PathLengthSumsSegments) {
  EXPECT_DOUBLE_EQ(PathLength({{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}}), 2.0);
}

TEST(BenchMetrics, PathLengthOfSinglePoseIsZero) {
  EXPECT_DOUBLE_EQ(PathLength({{0.0, 0.0, 0.0}}), 0.0);
}

TEST(BenchMetrics, ClearanceIsDistanceToNearestOccupiedCell) {
  EXPECT_NEAR(MinFootprintClearance({{0.0, 0.0, 0.0}}, {{1.0, 0.0}}, 0.30, 0.20), 0.70, 1e-9);
}

TEST(BenchMetrics, ClearanceAccountsForYaw) {
  EXPECT_NEAR(MinFootprintClearance({{0.0, 0.0, kPi / 2}}, {{0.0, 0.95}}, 0.30, 0.20),
              0.65, 1e-9);
}

TEST(BenchMetrics, ClearanceUsesEuclideanDistanceOnDiagonal) {
  // hypot(0.70, 0.80)。軸ごとの最大を返す実装なら 0.80 になって落ちる。
  EXPECT_NEAR(MinFootprintClearance({{0.0, 0.0, 0.0}}, {{1.0, 1.0}}, 0.30, 0.20),
              1.0630145, 1e-6);
}

TEST(BenchMetrics, ClearanceIsZeroForPointInsideFootprint) {
  EXPECT_NEAR(MinFootprintClearance({{0.0, 0.0, 0.0}}, {{0.10, 0.10}}, 0.30, 0.20), 0.0, 1e-9);
}

TEST(BenchMetrics, ClearancePicksNearestOfSeveralCells) {
  EXPECT_NEAR(MinFootprintClearance({{0.0, 0.0, 0.0}}, {{2.0, 0.0}, {0.0, 0.5}}, 0.30, 0.20),
              0.30, 1e-9);
}

TEST(BenchMetrics, ClearanceTakesWorstPoseNotFirst) {
  // 1 点目だけ見る実装なら 10.20 になって落ちる。
  EXPECT_NEAR(MinFootprintClearance({{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}}, {{10.5, 0.0}},
                                    0.30, 0.20),
              0.20, 1e-9);
}

TEST(BenchMetrics, ClearanceIsInfiniteWithoutObstacles) {
  EXPECT_TRUE(std::isinf(MinFootprintClearance({{0.0, 0.0, 0.0}}, {}, 0.30, 0.20)));
}

TEST(BenchMetrics, ContactCountCountsPosesCloserThanThreshold) {
  const std::vector<Pose2> poses{{0.0, 0.0, 0.0}, {5.0, 0.0, 0.0}};
  EXPECT_EQ(ContactCount(poses, {{0.31, 0.0}}, 0.30, 0.20), 1);
}

TEST(BenchMetrics, ContactCountIsZeroWhenEverythingIsFar) {
  EXPECT_EQ(ContactCount({{0.0, 0.0, 0.0}}, {{1.0, 0.0}}, 0.30, 0.20), 0);
}

TEST(BenchMetrics, SuccessRateCountsSuccesses) {
  EXPECT_DOUBLE_EQ(SuccessRate({true, true, false, true}), 0.75);
}

TEST(BenchMetrics, SuccessRateOfNoRunsIsZero) {
  EXPECT_DOUBLE_EQ(SuccessRate({}), 0.0);
}

TEST(BenchMetrics, PathLengthRatioDividesTravelledByPlanned) {
  EXPECT_DOUBLE_EQ(PathLengthRatio(6.0, 5.0), 1.2);
}

TEST(BenchMetrics, PathLengthRatioIsInfiniteWithoutPlan) {
  EXPECT_TRUE(std::isinf(PathLengthRatio(6.0, 0.0)));
}

TEST(BenchMetrics, IntervalStatsReportsHzAndP99) {
  std::vector<double> times;
  for (int i = 0; i < 21; ++i) {
    times.push_back(i * 0.05);
  }
  const auto stats = ComputeIntervalStats(times);
  EXPECT_NEAR(stats.effective_hz, 20.0, 1e-9);
  EXPECT_NEAR(stats.p99_ms, 50.0, 1e-9);
}

TEST(BenchMetrics, IntervalStatsP99CatchesAStall) {
  std::vector<double> times;
  for (int i = 0; i < 20; ++i) {
    times.push_back(i * 0.05);
  }
  times.push_back(1.15);  // 最後の間隔が 0.20 s
  EXPECT_NEAR(ComputeIntervalStats(times).p99_ms, 200.0, 1e-9);
}

TEST(BenchMetrics, IntervalStatsOfShortSeriesIsEmpty) {
  const auto stats = ComputeIntervalStats({0.0});
  EXPECT_DOUBLE_EQ(stats.effective_hz, 0.0);
  EXPECT_DOUBLE_EQ(stats.p99_ms, 0.0);
}

TEST(BenchMetrics, CellCostReadsTheGridCellUnderAPoint) {
  auto grid = MakeGrid(2, 2, 0.5, 0.0, 0.0, 0);
  grid.data[1] = 100;
  grid.data[3] = 254;
  EXPECT_EQ(CellCost(grid, 0.1, 0.1), 0);
  EXPECT_EQ(CellCost(grid, 0.6, 0.1), 100);
  EXPECT_EQ(CellCost(grid, 0.1, 0.6), 0);
  EXPECT_EQ(CellCost(grid, 0.6, 0.6), 254);
}

TEST(BenchMetrics, CellCostOutsideTheGridIsEmpty) {
  const auto grid = MakeGrid(2, 2, 0.5, 0.0, 0.0, 0);
  EXPECT_FALSE(CellCost(grid, -0.1, 0.1).has_value());
  EXPECT_FALSE(CellCost(grid, 0.1, 5.0).has_value());
}

TEST(BenchMetrics, SummarizeReportsCountsAndWorstCase) {
  std::vector<RunRecord> runs(2);
  runs[0].success = true;
  runs[0].duration_s = 10.0;
  runs[0].path_length_m = 5.0;
  runs[0].final_error_xy = 0.05;
  runs[0].final_error_yaw = 0.02;
  runs[0].min_clearance_m = 0.50;
  runs[1].success = false;
  runs[1].duration_s = 30.0;
  runs[1].path_length_m = 9.0;
  runs[1].final_error_xy = 0.40;
  runs[1].final_error_yaw = 0.30;
  runs[1].min_clearance_m = 0.08;

  const auto summary = Summarize(runs);
  EXPECT_EQ(summary.total, 2);
  EXPECT_EQ(summary.measured, 2);
  EXPECT_DOUBLE_EQ(summary.success_rate, 0.5);
  EXPECT_NEAR(*summary.max_final_error_xy, 0.40, 1e-9);
  EXPECT_NEAR(*summary.max_final_error_yaw, 0.30, 1e-9);
  EXPECT_NEAR(*summary.min_clearance_m, 0.08, 1e-9);
  EXPECT_NEAR(*summary.mean_duration_s, 20.0, 1e-9);
}

TEST(BenchMetrics, SummarizeSeparatesDrivingClearanceFromStartPose) {
  // 初期姿勢は「走って行った位置」ではない。走行中の最小値と混ぜない。
  std::vector<RunRecord> runs(2);
  for (auto& run : runs) {
    run.success = true;
    run.duration_s = 5.0;
    run.min_clearance_m = 0.10;
    run.min_clearance_driving_m = 0.45;
  }

  const auto summary = Summarize(runs);

  EXPECT_NEAR(*summary.min_clearance_m, 0.10, 1e-9);
  EXPECT_NEAR(*summary.min_clearance_driving_m, 0.45, 1e-9);
}

TEST(BenchMetrics, SummarizeOmitsDrivingClearanceWhenUnmeasured) {
  std::vector<RunRecord> runs(1);
  runs[0].success = true;
  runs[0].duration_s = 5.0;
  runs[0].min_clearance_m = 0.10;

  const auto summary = Summarize(runs);

  EXPECT_FALSE(summary.min_clearance_driving_m.has_value());
}

TEST(BenchMetrics, SummarizeIgnoresRunsWithoutMeasurements) {
  std::vector<RunRecord> runs(2);
  runs[0].success = false;  // 測定不能（拒否・軌跡なし）
  runs[1].success = true;
  runs[1].duration_s = 10.0;
  runs[1].final_error_xy = 0.05;
  runs[1].final_error_yaw = 0.02;
  runs[1].min_clearance_m = 0.50;

  const auto summary = Summarize(runs);
  EXPECT_EQ(summary.total, 2);
  EXPECT_EQ(summary.measured, 1);
  EXPECT_NEAR(*summary.min_clearance_m, 0.50, 1e-9);
  EXPECT_NEAR(*summary.max_final_error_xy, 0.05, 1e-9);
}

TEST(BenchMetrics, SummarizeOfNoRunsIsEmpty) {
  const auto summary = Summarize({});
  EXPECT_EQ(summary.total, 0);
  EXPECT_EQ(summary.measured, 0);
  EXPECT_DOUBLE_EQ(summary.success_rate, 0.0);
  EXPECT_FALSE(summary.min_clearance_m.has_value());
}

TEST(BenchMetrics, SelectFreeGoalsRespectsSpacing) {
  const auto grid = MakeGrid(8, 8, 0.5, 0.0, 0.0, 0);
  const auto goals = SelectFreeGoals(grid, {}, 5, 1.0, 0.0, 0.30, 0.20);
  ASSERT_EQ(goals.size(), 5u);
  for (std::size_t i = 0; i < goals.size(); ++i) {
    for (std::size_t j = i + 1; j < goals.size(); ++j) {
      EXPECT_GE(std::hypot(goals[i].x - goals[j].x, goals[i].y - goals[j].y), 1.0);
    }
  }
}

TEST(BenchMetrics, SelectFreeGoalsIsDeterministic) {
  const auto grid = MakeGrid(8, 8, 0.5, 0.0, 0.0, 0);
  const auto first = SelectFreeGoals(grid, {}, 4, 1.0, 0.0, 0.30, 0.20);
  const auto second = SelectFreeGoals(grid, {}, 4, 1.0, 0.0, 0.30, 0.20);
  ASSERT_EQ(first.size(), second.size());
  for (std::size_t i = 0; i < first.size(); ++i) {
    EXPECT_DOUBLE_EQ(first[i].x, second[i].x);
    EXPECT_DOUBLE_EQ(first[i].y, second[i].y);
  }
}

TEST(BenchMetrics, SelectFreeGoalsHonoursMinClearance) {
  const auto grid = MakeGrid(8, 8, 0.5, 0.0, 0.0, 0);
  const std::vector<Cell> occupied{{0.25, 3.0}};
  const auto goals = SelectFreeGoals(grid, occupied, 4, 0.5, 1.0, 0.30, 0.20);
  ASSERT_FALSE(goals.empty());
  for (const auto& goal : goals) {
    EXPECT_GT(ClearanceAt({goal.x, goal.y, 0.0}, occupied, 0.30, 0.20), 1.0 - 1e-9);
  }
}

TEST(BenchMetrics, SelectFreeGoalsSkipsOccupiedCells) {
  const auto grid = MakeGrid(4, 4, 0.5, 0.0, 0.0, 100);
  EXPECT_TRUE(SelectFreeGoals(grid, {}, 3, 0.5, 0.0, 0.30, 0.20).empty());
}

TEST(BenchMetrics, ValidateGoalsRejectsEmptyList) {
  const auto grid = MakeGrid(4, 4, 0.5, 0.0, 0.0, 0);
  const auto problems = ValidateGoals({}, grid);
  ASSERT_EQ(problems.size(), 1u);
  EXPECT_EQ(problems[0], "no goals defined");
}

TEST(BenchMetrics, ValidateGoalsRejectsGoalInsideObstacle) {
  auto grid = MakeGrid(4, 4, 0.5, 0.0, 0.0, 0);
  grid.data[0] = 100;  // セル (0,0) が占有
  const auto problems = ValidateGoals({{0.1, 0.1, 0.0}}, grid);
  ASSERT_EQ(problems.size(), 1u);
  EXPECT_EQ(problems[0], "goal 0 (0.1, 0.1) is not in free space");
}

TEST(BenchMetrics, ValidateGoalsAcceptsFreeGoals) {
  const auto grid = MakeGrid(4, 4, 0.5, 0.0, 0.0, 0);
  EXPECT_TRUE(ValidateGoals({{1.75, 1.75, 0.0}}, grid).empty());
}

TEST(BenchMetrics, ValidateGoalsRejectsGoalOutsideTheGrid) {
  const auto grid = MakeGrid(4, 4, 0.5, 0.0, 0.0, 0);
  EXPECT_EQ(ValidateGoals({{9.0, 9.0, 0.0}}, grid).size(), 1u);
}

TEST(BenchMetrics, SummarizeAggregatesPlannerP95AndWorstControlP99) {
  std::vector<RunRecord> runs(3);
  for (int i = 0; i < 3; ++i) {
    runs[i].success = true;
    runs[i].min_clearance_m = 0.5;
    runs[i].final_error_xy = 0.05;
    runs[i].final_error_yaw = 0.02;
  }
  runs[0].planner_latency_ms = 10.0;
  runs[1].planner_latency_ms = 20.0;
  runs[2].planner_latency_ms = 100.0;
  runs[0].control_p99_ms = 50.0;
  runs[1].control_p99_ms = 70.0;
  runs[2].control_p99_ms = 40.0;

  const auto summary = Summarize(runs);
  ASSERT_TRUE(summary.planner_latency_p95_ms.has_value());
  EXPECT_NEAR(*summary.planner_latency_p95_ms, 100.0, 1e-9);
  ASSERT_TRUE(summary.max_control_p99_ms.has_value());
  EXPECT_NEAR(*summary.max_control_p99_ms, 70.0, 1e-9);
}

}  // namespace
}  // namespace ros2_autonomous_nav

// SPDX-License-Identifier: MIT
/// ナビゲーション評価の指標。ROS に依存しない純関数だけを置く。
///
/// 設計書 docs/superpowers/specs/2026-11-18-navigation-design.md §8.1 の合格基準を
/// そのまま計算できる形にしたもの。障害物セルは「地図の占有セル」を渡すこと。
/// /global_costmap/costmap は 0..100 に変換済みで内接リングを含むため、
/// そのまま渡すと距離が実際より小さく出る。呼び出し側は costmap_raw の
/// LETHAL 以上を使う。
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace ros2_autonomous_nav {

/// 合格判定に使う定数。
inline constexpr int kFreeCostMax = 50;      ///< ゴールを自由とみなすコストマップ値の上限
inline constexpr int kLethalRaw = 254;       ///< nav2_msgs/Costmap の LETHAL_OBSTACLE
inline constexpr double kContactThreshold = 0.02;  ///< 接触とみなす footprint 距離 [m]

/// 機体寸法 [m]。設計書 §3 が 0.6 x 0.4 と定めている。
/// URDF（半径 0.25）と Nav2 の robot_radius（0.35）はこの値より古い記述で、
/// 更新は設計書 PR4（自己位置推定の入れ替え）で行う。
inline constexpr double kFootprintHalfLength = 0.30;
inline constexpr double kFootprintHalfWidth = 0.20;

struct Pose2 {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

using Cell = std::pair<double, double>;

/// nav_msgs/OccupancyGrid と同じ並びの読み取り専用ビュー。
struct GridView {
  int width{0};
  int height{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  // OccupancyGrid の -1..100 と costmap_raw の 0..255 の両方を保持する。
  std::vector<int16_t> data;
};

struct Goal {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct IntervalStats {
  double effective_hz{0.0};
  double p99_ms{0.0};
};

/// 1 回のゴール実行の記録。測定できなかった項目は nullopt。
struct RunRecord {
  bool success{false};
  double duration_s{0.0};
  double path_length_m{0.0};
  double planned_length_m{0.0};
  std::optional<double> path_length_ratio;
  std::optional<double> final_error_xy;
  std::optional<double> final_error_yaw;
  std::optional<double> min_clearance_m;
  /// 初期姿勢を除いた最小クリアランス。走行で近づいた量だけを見る。
  std::optional<double> min_clearance_driving_m;
  std::optional<int> contact_count;
  std::optional<double> planner_latency_ms;
  std::optional<double> control_hz;
  std::optional<double> control_p99_ms;
};

struct RunSummary {
  int total{0};
  int measured{0};
  double success_rate{0.0};
  std::optional<double> max_final_error_xy;
  std::optional<double> max_final_error_yaw;
  std::optional<double> min_clearance_m;
  /// 初期姿勢を除いた最小クリアランス。走行で近づいた量だけを見る。
  std::optional<double> min_clearance_driving_m;
  std::optional<double> mean_duration_s;
  /// 設計書 §8.1「計画 p95 < 100 ms」の判定に使う。
  std::optional<double> planner_latency_p95_ms;
  /// 設計書 §8.1「制御 p99 < 60 ms」の判定に、最悪の run で使う。
  std::optional<double> max_control_p99_ms;
};

/// 軌跡の累積距離 [m]。点が 1 つ以下なら 0。
double PathLength(const std::vector<Pose2>& poses);

/// 1 姿勢での機体矩形と障害物セルの最短距離 [m]。障害物が無ければ infinity。
double ClearanceAt(const Pose2& pose, const std::vector<Cell>& occupied,
                   double half_length, double half_width);

/// 全姿勢のうちで最も近い距離 [m]。
double MinFootprintClearance(const std::vector<Pose2>& poses,
                             const std::vector<Cell>& occupied,
                             double half_length, double half_width);

/// 機体矩形が threshold 未満まで近づいた姿勢の数。
int ContactCount(const std::vector<Pose2>& poses, const std::vector<Cell>& occupied,
                 double half_length, double half_width,
                 double threshold = kContactThreshold);

/// 走行距離 ÷ 計画経路長。計画が 0 以下なら infinity。
double PathLengthRatio(double travelled, double planned);

/// 連続メッセージの時刻列から実行周波数と周期 p99 [ms] を出す。
IntervalStats ComputeIntervalStats(const std::vector<double>& times);

/// グリッドのセル値。範囲外は nullopt。
std::optional<int> CellCost(const GridView& grid, double x, double y);

double SuccessRate(const std::vector<bool>& results);

/// ゴールが自由空間にあるかを検証する。問題の説明を返す（空なら実行してよい）。
/// ROS に依存しないので単体テストで固定できる。
std::vector<std::string> ValidateGoals(const std::vector<Goal>& goals,
                                       const GridView& cost_grid,
                                       int free_cost_max = kFreeCostMax);

/// 測定値が揃っている run だけを集計する。無ければ各値は nullopt。
RunSummary Summarize(const std::vector<RunRecord>& runs);

/// 空きセルから、互いに min_spacing 以上離れたゴールを決定的に選ぶ。
///
/// 走査順は row-major 固定なので、同じ地図と同じ引数なら必ず同じ結果になる。
/// エッジ近傍を避けたいので footprint のクリアランスでも足切りする。
std::vector<Goal> SelectFreeGoals(const GridView& free_grid,
                                  const std::vector<Cell>& occupied, int count,
                                  double min_spacing, double min_clearance,
                                  double half_length, double half_width);

}  // namespace ros2_autonomous_nav

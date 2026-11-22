// SPDX-License-Identifier: MIT
/// ROS のメッセージを bench_metrics の型へ変換する。
#pragma once

#include <vector>

#include <nav2_msgs/msg/costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "umeonigiri_autonomy/bench_metrics.hpp"

namespace umeonigiri_autonomy {

/// /global_costmap/costmap（0..100 に変換済み）を読むためのビュー。
GridView OccupancyToView(const nav_msgs::msg::OccupancyGrid& msg);

/// 生のコストマップと、そこから抜き出した LETHAL セル。
struct RawCostmap {
  GridView view;
  std::vector<Cell> lethal;
};

/// /global_costmap/costmap_raw（0..255 の生値）から両方を作る。
///
/// metadata の走査は 1 回で済ませる。LETHAL 判定は生値で行うこと
/// （変換済みの /costmap は内接リングを含むので距離が小さく出る）。
RawCostmap ParseRawCostmap(const nav2_msgs::msg::Costmap& msg);

}  // namespace umeonigiri_autonomy

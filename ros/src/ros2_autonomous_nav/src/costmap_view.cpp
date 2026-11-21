// SPDX-License-Identifier: MIT
#include "ros2_autonomous_nav/costmap_view.hpp"

namespace ros2_autonomous_nav {

GridView OccupancyToView(const nav_msgs::msg::OccupancyGrid& msg) {
  GridView view;
  view.width = static_cast<int>(msg.info.width);
  view.height = static_cast<int>(msg.info.height);
  view.resolution = msg.info.resolution;
  view.origin_x = msg.info.origin.position.x;
  view.origin_y = msg.info.origin.position.y;
  view.data.assign(msg.data.begin(), msg.data.end());
  return view;
}

RawCostmap ParseRawCostmap(const nav2_msgs::msg::Costmap& msg) {
  RawCostmap out;
  out.view.width = static_cast<int>(msg.metadata.size_x);
  out.view.height = static_cast<int>(msg.metadata.size_y);
  out.view.resolution = msg.metadata.resolution;
  out.view.origin_x = msg.metadata.origin.position.x;
  out.view.origin_y = msg.metadata.origin.position.y;
  out.view.data.assign(msg.data.begin(), msg.data.end());

  // 幅 0 のコストマップ（未初期化のまま配信された場合）で i % 0 を踏むと、
  // 整数のゼロ除算で未定義動作になる。x86 では SIGFPE、ARM では 0 が返って
  // 全てのセルが原点に重なる壊れた座標になる。セルが無いのと同じ扱いにする。
  if (out.view.width <= 0) {
    return out;
  }

  const double res = out.view.resolution;
  for (std::size_t i = 0; i < msg.data.size(); ++i) {
    if (msg.data[i] < kLethalRaw) {
      continue;
    }
    const auto col = static_cast<int>(i) % out.view.width;
    const auto row = static_cast<int>(i) / out.view.width;
    out.lethal.emplace_back(out.view.origin_x + (col + 0.5) * res,
                            out.view.origin_y + (row + 0.5) * res);
  }
  return out;
}

}  // namespace ros2_autonomous_nav

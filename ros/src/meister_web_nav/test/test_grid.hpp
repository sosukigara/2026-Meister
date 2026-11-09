// SPDX-License-Identifier: MIT
/// test で使う OccupancyGrid のビルダ。rclcpp::init は要らない。
#ifndef MEISTER_WEB_NAV_TEST_GRID_HPP_
#define MEISTER_WEB_NAV_TEST_GRID_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace meister_web_nav {
namespace test {

using Grid = nav_msgs::msg::OccupancyGrid;

/// Python 版 conftest.py の make_grid と同じ形。origin を非対称 (-1.0, -2.0) に
/// しておく（x と y を取り違えた実装が test で落ちるように）。
///
/// const を付けないのは、test が既知セルを作った後に書き込むため。SetGrid も
/// MetadataOf も const 参照を取るので、そのまま渡せる。
inline std::shared_ptr<Grid> MakeGrid(int size = 32, int fill = 0, int wall_row = 10) {
  auto grid = std::make_shared<Grid>();
  grid->info.width = static_cast<uint32_t>(size);
  grid->info.height = static_cast<uint32_t>(size);
  grid->info.resolution = 0.05;
  grid->info.origin.position.x = -1.0;
  grid->info.origin.position.y = -2.0;
  grid->data.assign(static_cast<std::size_t>(size) * static_cast<std::size_t>(size),
                    static_cast<int8_t>(fill));
  if (wall_row >= 0) {
    const std::size_t row = static_cast<std::size_t>(wall_row) * static_cast<std::size_t>(size);
    for (int c = 0; c < size; ++c) {
      grid->data[row + static_cast<std::size_t>(c)] = 100;
    }
  }
  return grid;
}

}  // namespace test
}  // namespace meister_web_nav

#endif  // MEISTER_WEB_NAV_TEST_GRID_HPP_

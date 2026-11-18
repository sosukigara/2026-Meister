// SPDX-License-Identifier: MIT
/// map_generator の単体テスト。ファイルを書かずグリッドだけを検査する。
///
/// バイト一致の検証はゴールデンファイル（maps/nav_world.pgm）との cmp で行う。
#include <gtest/gtest.h>

#include <cstdint>

#include "ros2_autonomous_nav/map_generator.hpp"

namespace ros2_autonomous_nav {
namespace {

int At(const MapSpec& spec, const std::vector<uint8_t>& grid, double wx, double wy) {
  const int px = static_cast<int>((wx - spec.origin_x) / spec.resolution);
  const int py = static_cast<int>((wy - spec.origin_y) / spec.resolution);
  return grid[static_cast<std::size_t>(py) * spec.width() + px];
}

TEST(MapGenerator, GridIsTwelveMetresAtFiveCentimetres) {
  const MapSpec spec;
  EXPECT_EQ(spec.width(), 240);
  EXPECT_EQ(spec.height(), 240);
  EXPECT_EQ(BuildNavWorldGrid(spec).size(), 240u * 240u);
}

TEST(MapGenerator, OuterWallIsOccupied) {
  const MapSpec spec;
  const auto grid = BuildNavWorldGrid(spec);
  EXPECT_EQ(At(spec, grid, 0.0, 5.0), kOccupiedPixel);    // north
  EXPECT_EQ(At(spec, grid, 0.0, -5.0), kOccupiedPixel);   // south
  EXPECT_EQ(At(spec, grid, 5.0, 0.0), kOccupiedPixel);    // east
  EXPECT_EQ(At(spec, grid, -5.0, 0.0), kOccupiedPixel);   // west
}

TEST(MapGenerator, OpenFloorIsFree) {
  const MapSpec spec;
  const auto grid = BuildNavWorldGrid(spec);
  EXPECT_EQ(At(spec, grid, 0.0, 0.0), kFreePixel);
  EXPECT_EQ(At(spec, grid, 4.0, 4.0), kFreePixel);
}

TEST(MapGenerator, PillarIsOccupied) {
  const MapSpec spec;
  const auto grid = BuildNavWorldGrid(spec);
  EXPECT_EQ(At(spec, grid, -3.0, -3.0), kOccupiedPixel);
  EXPECT_EQ(At(spec, grid, 3.0, 3.0), kOccupiedPixel);
}

TEST(MapGenerator, OutsideTheEnclosureIsUnknown) {
  const MapSpec spec;
  const auto grid = BuildNavWorldGrid(spec);
  EXPECT_EQ(At(spec, grid, -5.9, -5.9), kUnknownPixel);
  EXPECT_EQ(At(spec, grid, 5.9, 5.9), kUnknownPixel);
}

TEST(MapGenerator, WallIsOneCellWideOnTheGrid) {
  const MapSpec spec;
  const auto grid = BuildNavWorldGrid(spec);
  // 北壁は y=4.9..5.1。セル中心 4.925 は壁、4.875 は床。
  EXPECT_EQ(At(spec, grid, 0.0, 4.925), kOccupiedPixel);
  EXPECT_EQ(At(spec, grid, 0.0, 4.875), kFreePixel);
}

}  // namespace
}  // namespace ros2_autonomous_nav

// SPDX-License-Identifier: MIT
/// map_generator の単体テスト。ファイルを書かずグリッドだけを検査する。
///
/// バイト一致の検証はゴールデンファイル（maps/nav_world.pgm）との cmp で行う。
#include <gtest/gtest.h>

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include "ros2_autonomous_nav/map_generator.hpp"

namespace ros2_autonomous_nav {
namespace {

/// 書き込みに失敗したら false を返すこと。以前は失敗しても何も返さず、
/// generate_map が「map written」と表示して 0 で終わっていた。
TEST(MapGenerator, ReportsWriteFailure) {
  const MapSpec spec;
  const auto grid = BuildNavWorldGrid(spec);
  // 存在しないディレクトリの下は開けない。
  EXPECT_FALSE(WritePgm("/nonexistent-dir-for-test/nav_world.pgm", spec, grid));
  EXPECT_FALSE(WriteMapYaml("/nonexistent-dir-for-test/nav_world.yaml", spec, "nav_world.pgm"));
}

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

TEST(MapGenerator, WallBoundaryMatchesPythonRounding) {
  const MapSpec spec;
  const auto grid = BuildNavWorldGrid(spec);
  // wall_i1 の西端 x=-1.575 と wall_i4 の南端 y=-1.575 はちょうど半セル。
  // Python 版は乗算と加算を別々に丸めるので自由、FMA に融合されると占有になる。
  EXPECT_EQ(At(spec, grid, -1.575, -1.025), kFreePixel);
  EXPECT_EQ(At(spec, grid, -1.525, -1.025), kOccupiedPixel);
  EXPECT_EQ(At(spec, grid, 3.5, -1.575), kFreePixel);
  EXPECT_EQ(At(spec, grid, 3.5, -1.525), kOccupiedPixel);
}

TEST(MapGenerator, MatchesGoldenPgmByteForByte) {
  std::ifstream in(MAP_GOLDEN_PATH, std::ios::binary);
  ASSERT_TRUE(in.good()) << "missing golden " << MAP_GOLDEN_PATH;
  std::string magic;
  int width = 0;
  int height = 0;
  int max_value = 0;
  in >> magic >> width >> height >> max_value;
  ASSERT_EQ(magic, "P5");
  ASSERT_EQ(max_value, 255);
  in.get();  // ヘッダ末尾の改行

  std::vector<uint8_t> golden(static_cast<std::size_t>(width) * height);
  in.read(reinterpret_cast<char*>(golden.data()), static_cast<std::streamsize>(golden.size()));
  ASSERT_EQ(in.gcount(), static_cast<std::streamsize>(golden.size()));

  const MapSpec spec;
  const auto grid = BuildNavWorldGrid(spec);
  ASSERT_EQ(width, spec.width());
  ASSERT_EQ(height, spec.height());
  // PGM は row 0 が最大 y。グリッドは y 昇順なので上下を入れ替えて比べる。
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      const auto golden_index = static_cast<std::size_t>(row) * width + col;
      const auto grid_index = static_cast<std::size_t>(height - 1 - row) * width + col;
      ASSERT_EQ(golden[golden_index], grid[grid_index]) << "row " << row << " col " << col;
    }
  }
}

}  // namespace
}  // namespace ros2_autonomous_nav

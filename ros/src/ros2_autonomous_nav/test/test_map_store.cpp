// SPDX-License-Identifier: MIT
/// map_store の単体テスト。実ファイルは一時ディレクトリだけを使う。
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "ros2_autonomous_nav/map_store.hpp"

namespace ros2_autonomous_nav {
namespace {

TEST(MapStore, AcceptsOrdinaryNames) {
  EXPECT_TRUE(IsValidMapName("warehouse"));
  EXPECT_TRUE(IsValidMapName("warehouse_20261118-110000"));
  EXPECT_TRUE(IsValidMapName("map-1"));
  EXPECT_TRUE(IsValidMapName("A"));
}

TEST(MapStore, RejectsTraversal) {
  EXPECT_FALSE(IsValidMapName(".."));
  EXPECT_FALSE(IsValidMapName("../etc/passwd"));
  EXPECT_FALSE(IsValidMapName("a/b"));
  EXPECT_FALSE(IsValidMapName("a\\b"));
}

TEST(MapStore, RejectsEmptyAndTooLongNames) {
  EXPECT_FALSE(IsValidMapName(""));
  EXPECT_FALSE(IsValidMapName(std::string(65, 'a')));
  EXPECT_TRUE(IsValidMapName(std::string(64, 'a')));
}

TEST(MapStore, RejectsSpacesAndDots) {
  EXPECT_FALSE(IsValidMapName("my map"));
  EXPECT_FALSE(IsValidMapName("my.map"));
  EXPECT_FALSE(IsValidMapName(".hidden"));
}

TEST(MapStore, DefaultNameCombinesWorldAndTimestamp) {
  EXPECT_EQ(DefaultMapName("warehouse", "20261118-110000"),
            "warehouse_20261118-110000");
}

TEST(MapStore, DefaultNameFallsBackWhenWorldIsUnusable) {
  // world がファイル名に使えない場合は map に落とす。空名は作らない。
  EXPECT_EQ(DefaultMapName("my world", "20261118-110000"), "map_20261118-110000");
  EXPECT_EQ(DefaultMapName("", "20261118-110000"), "map_20261118-110000");
}

TEST(MapStore, ListsMapStemsSortedAndIgnoresOtherFiles) {
  const auto dir = std::filesystem::temp_directory_path() / "meister_map_store_test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  for (const char* name : {"b_second.yaml", "a_first.yaml", "notes.txt"}) {
    std::ofstream(dir / name) << "x";
  }
  const auto maps = ListMaps(dir.string());
  ASSERT_EQ(maps.size(), 2u);
  EXPECT_EQ(maps[0], "a_first");
  EXPECT_EQ(maps[1], "b_second");
  std::filesystem::remove_all(dir);
}

TEST(MapStore, ListsNothingWhenDirectoryIsMissing) {
  EXPECT_TRUE(ListMaps("/nonexistent/meister/maps").empty());
}

}  // namespace
}  // namespace ros2_autonomous_nav

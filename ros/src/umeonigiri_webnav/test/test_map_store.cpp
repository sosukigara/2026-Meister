// SPDX-License-Identifier: MIT
/// map_store の単体テスト。実ファイルは一時ディレクトリだけを使う。
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "umeonigiri_webnav/map_store.hpp"

namespace umeonigiri_webnav {
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
  const auto maps = ListMapNames(dir.string());
  ASSERT_EQ(maps.size(), 2u);
  EXPECT_EQ(maps[0], "a_first");
  EXPECT_EQ(maps[1], "b_second");
  std::filesystem::remove_all(dir);
}

TEST(MapStore, ListsNothingWhenDirectoryIsMissing) {
  EXPECT_TRUE(ListMapNames("/nonexistent/meister/maps").empty());
}

TEST(MapStore, NameOrDefaultKeepsRequestedName) {
  EXPECT_EQ(MapNameOrDefault("lab", "warehouse", "20261118-110000"), "lab");
}

TEST(MapStore, NameOrDefaultFillsTheEmptyCase) {
  // UI の「地図名（空なら自動）」。空のまま map_saver へ渡すと保存先が
  // ディレクトリになり、一覧に出ない地図ができていた。
  EXPECT_EQ(MapNameOrDefault("", "warehouse", "20261118-110000"),
            "warehouse_20261118-110000");
  EXPECT_EQ(MapNameOrDefault("", "", "20261118-110000"), "map_20261118-110000");
}

TEST(MapStore, DeleteRemovesYamlAndBothImageFormats) {
  const auto dir =
      std::filesystem::temp_directory_path() / "meister_map_delete_test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  for (const char* name : {"warehouse.yaml", "warehouse.png", "warehouse.pgm",
                           "keep.yaml"}) {
    std::ofstream(dir / name) << "x";
  }

  EXPECT_EQ(DeleteMapFiles(dir.string(), "warehouse"), "");
  EXPECT_FALSE(std::filesystem::exists(dir / "warehouse.yaml"));
  // map_saver は image_format 次第で .png も .pgm も書く。yaml だけ消すと
  // 画像が maps に溜まり続ける。
  EXPECT_FALSE(std::filesystem::exists(dir / "warehouse.png"));
  EXPECT_FALSE(std::filesystem::exists(dir / "warehouse.pgm"));
  EXPECT_TRUE(std::filesystem::exists(dir / "keep.yaml"));

  // 2 回目以降も成功 (ボタンを二度押しても同じ結果)。
  EXPECT_EQ(DeleteMapFiles(dir.string(), "warehouse"), "");

  std::filesystem::remove_all(dir);
}

TEST(MapStore, DeleteRejectsUnsafeNames) {
  EXPECT_FALSE(DeleteMapFiles("/tmp", "../etc/passwd").empty());
  EXPECT_FALSE(DeleteMapFiles("/tmp", "a/b").empty());
  EXPECT_FALSE(DeleteMapFiles("/tmp", "").empty());
}

}  // namespace
}  // namespace umeonigiri_webnav

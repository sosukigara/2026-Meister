// SPDX-License-Identifier: MIT
/// MapCache の単体テスト。ROS グラフは不要 (rclcpp::init もしない)。
///
/// crop / encode が version ごとに高々 1 回しか走らないことは、時間ではなく
/// MapCache::stats() の回数で検証する (時間は CI 負荷で変わる)。
#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "meister_web_nav/map_cache.hpp"
#include "test_grid.hpp"

namespace meister_web_nav {
namespace {

using test::Grid;
using test::MakeGrid;

/// C++17 に std::barrier が無いので、全スレッドが揃うまで spin する。
class SpinBarrier {
 public:
  explicit SpinBarrier(int count) : target_(count) {}

  void Wait() {
    arrived_.fetch_add(1);
    while (arrived_.load() < target_) {
    }
  }

 private:
  std::atomic<int> arrived_{0};
  const int target_;
};

cv::Mat DecodePng(const std::string& png) {
  const std::vector<uint8_t> bytes(png.begin(), png.end());
  return cv::imdecode(bytes, cv::IMREAD_GRAYSCALE);
}

TEST(MapCacheCropBounds, RepeatedPollsCropOnce) {
  MapCache cache;
  cache.SetGrid(MakeGrid());
  for (int i = 0; i < 10; ++i) {
    cache.Snapshot();
    cache.RenderPng();
  }
  const MapCache::Stats stats = cache.stats();
  EXPECT_EQ(stats.crop_calls, 1u);
  EXPECT_EQ(stats.encode_calls, 1u);
}

TEST(MapCacheCropBounds, MetadataAndRenderShareOneCrop) {
  // 1 ポーリングで /api/map と /api/map.png の両方が来る。crop は 1 回で足りる。
  MapCache cache;
  cache.SetGrid(MakeGrid());
  cache.Snapshot();
  cache.RenderPng();
  EXPECT_EQ(cache.stats().crop_calls, 1u);
}

TEST(MapCacheCropBounds, NewVersionCropsAgain) {
  MapCache cache;
  cache.SetGrid(MakeGrid());
  cache.RenderPng();
  EXPECT_EQ(cache.stats().crop_calls, 1u);

  cache.SetGrid(MakeGrid(32, 0, 14));
  cache.RenderPng();
  EXPECT_EQ(cache.stats().crop_calls, 2u);
  EXPECT_EQ(cache.stats().encode_calls, 2u);
}

TEST(MapCacheCropBounds, IdenticalContentKeepsVersion) {
  // 内容が同じなら version が変わらない (ETag を安定させるため)。
  MapCache cache;
  cache.SetGrid(MakeGrid());
  const std::string version_a = cache.Snapshot().version;
  cache.SetGrid(MakeGrid());  // 別インスタンスだが内容は同一
  const std::string version_b = cache.Snapshot().version;

  EXPECT_EQ(version_a, version_b);
  EXPECT_EQ(cache.stats().crop_calls, 1u);
}

TEST(MapCacheCropBounds, ConcurrentPollsCropOnce) {
  MapCache cache;
  cache.SetGrid(MakeGrid(64));
  SpinBarrier barrier(8);
  std::vector<std::thread> threads;
  threads.reserve(8);
  for (int i = 0; i < 8; ++i) {
    threads.emplace_back([&cache, &barrier]() {
      barrier.Wait();
      for (int n = 0; n < 5; ++n) {
        cache.RenderPng();
      }
    });
  }
  for (auto& thread : threads) {
    thread.join();
  }
  const MapCache::Stats stats = cache.stats();
  EXPECT_EQ(stats.crop_calls, 1u);
  EXPECT_EQ(stats.encode_calls, 1u);
}

TEST(MapCacheRenderPng, StableForSameVersion) {
  MapCache cache;
  cache.SetGrid(MakeGrid());
  const auto first = cache.RenderPng();
  const auto second = cache.RenderPng();
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(first->first, second->first);
  EXPECT_EQ(first->second, second->second);
  EXPECT_EQ(first->first.compare(0, 4, "\x89PNG"), 0);
}

TEST(MapCacheRenderPng, VersionChangesWithContent) {
  MapCache cache;
  cache.SetGrid(MakeGrid());
  const auto a = cache.RenderPng();
  cache.SetGrid(MakeGrid(32, 0, 20));
  const auto b = cache.RenderPng();
  ASSERT_TRUE(a.has_value());
  ASSERT_TRUE(b.has_value());
  EXPECT_NE(a->second, b->second);
}

TEST(MapCacheRenderPng, VersionChangesWithResolution) {
  // セルの色が変わらなくても resolution が変われば別の地図。
  MapCache cache;
  cache.SetGrid(MakeGrid());
  const auto a = cache.RenderPng();
  auto moved = MakeGrid();
  moved->info.resolution = 0.02;
  cache.SetGrid(moved);
  const auto b = cache.RenderPng();
  ASSERT_TRUE(a.has_value());
  ASSERT_TRUE(b.has_value());
  EXPECT_NE(a->second, b->second);
}

TEST(MapCacheRenderPng, NoGridYet) {
  MapCache cache;
  EXPECT_FALSE(cache.RenderPng().has_value());
  EXPECT_FALSE(cache.Snapshot().grid);
}

TEST(MapCacheRenderPng, CroppedToKnownCells) {
  MapCache cache;
  auto grid = MakeGrid(40, -1, -1);
  // 40x40 のうち row 20 / col 10..20 だけが既知。
  for (int c = 10; c <= 20; ++c) {
    grid->data[static_cast<std::size_t>(20) * 40 + c] = 0;
  }
  cache.SetGrid(grid);
  const auto png = cache.RenderPng();
  ASSERT_TRUE(png.has_value());

  const cv::Mat image = DecodePng(png->first);
  ASSERT_FALSE(image.empty());

  const MapSnapshot snapshot = cache.Snapshot();
  ASSERT_TRUE(snapshot.bounds.has_value());
  const CropBounds& bounds = *snapshot.bounds;
  EXPECT_EQ(image.cols, bounds.c1 - bounds.c0);
  EXPECT_EQ(image.rows, bounds.r1 - bounds.r0);
  EXPECT_LT(image.cols, 40);
  EXPECT_LT(image.rows, 40);
  // 既知セルが 1 本の線なので、上下は未知セルの灰色で埋まる。
  EXPECT_EQ(image.at<uint8_t>(0, 0), 205);
}

TEST(MapCacheRenderPng, PixelValuesFollowOccupancy) {
  // row 0 が地図の下端、画像 row 0 は上端。既知 0 -> 255、100 -> 0、50 -> 127、
  // 未知 -> 205。
  MapCache cache;
  auto grid = std::make_shared<Grid>();
  grid->info.width = 2;
  grid->info.height = 2;
  grid->info.resolution = 0.05;
  grid->data = {0, 100, -1, 50};  // row 0 = (0, 100), row 1 = (-1, 50)
  cache.SetGrid(grid);

  const auto png = cache.RenderPng();
  ASSERT_TRUE(png.has_value());
  const cv::Mat image = DecodePng(png->first);
  ASSERT_EQ(image.rows, 2);
  ASSERT_EQ(image.cols, 2);
  EXPECT_EQ(image.at<uint8_t>(0, 0), 205);  // 未知 (grid row 1)
  EXPECT_EQ(image.at<uint8_t>(0, 1), 127);  // 占有 50
  EXPECT_EQ(image.at<uint8_t>(1, 0), 255);  // 自由 0 (grid row 0)
  EXPECT_EQ(image.at<uint8_t>(1, 1), 0);    // 占有 100
}

}  // namespace
}  // namespace meister_web_nav

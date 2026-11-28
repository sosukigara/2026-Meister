#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "umeonigiri_lidar/scan_processor.hpp"

namespace umeonigiri_lidar {
namespace {

TEST(ScanProcessorTest, DeskewIgnoresBelowThresholdAndScalesToScanEnd) {
  EXPECT_DOUBLE_EQ(ComputeDeskewAngleOffsetDeg(0.0, 0.1, 0.02, 0.05), 0.0);
  // スキャン終了点（fraction = 1.0）ではオフセット 0
  EXPECT_DOUBLE_EQ(ComputeDeskewAngleOffsetDeg(1.0, 0.1, 0.5, 0.05), 0.0);
  // スキャン開始点（fraction = 0.0）、0.1s、+0.5 rad/s -> -0.05 rad = -2.8648 deg
  const double deg = ComputeDeskewAngleOffsetDeg(0.0, 0.1, 0.5, 0.05);
  EXPECT_NEAR(deg, -0.05 * 180.0 / M_PI, 1e-6);
}

TEST(ScanProcessorTest, SpeckleFilterRemovesIsolatedSpikePreservesWall) {
  const float inf = std::numeric_limits<float>::infinity();
  std::vector<float> ranges = {2.0f, 2.02f, 2.05f, 0.40f, 2.03f, 2.01f, inf, 5.0f, inf};
  const size_t removed = ApplySpeckleFilter(ranges, 0.25f);

  EXPECT_EQ(removed, 2U);
  EXPECT_FALSE(std::isfinite(ranges[3]));  // 単発飛び点 0.40m を除去
  EXPECT_FALSE(std::isfinite(ranges[7]));  // 両隣が inf の孤立点 5.0m を除去
  EXPECT_FLOAT_EQ(ranges[2], 2.05f);       // 連続した壁は保持
  EXPECT_FLOAT_EQ(ranges[4], 2.03f);
}

TEST(ScanProcessorTest, TiltProjectionScalesRangeAndRejectsExcessiveTilt) {
  // pitch = 10 deg, 前方ビーム (theta = 0): cos(10 deg) * 2.0 m
  const double pitch = 10.0 * M_PI / 180.0;
  const float proj = ProjectRangeToHorizontal(2.0f, 0.0, 0.0, pitch, 0.25);
  EXPECT_NEAR(proj, 2.0f * std::cos(pitch), 1e-4f);

  // max_tilt_rad (0.25 rad) を超える 20 deg 傾きは +inf で棄却
  const float rejected = ProjectRangeToHorizontal(2.0f, 0.0, 0.0, 20.0 * M_PI / 180.0, 0.25);
  EXPECT_FALSE(std::isfinite(rejected));
}

}  // namespace
}  // namespace umeonigiri_lidar

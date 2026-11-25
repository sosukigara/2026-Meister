// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "umeonigiri_vision/box_center_filter.hpp"

using umeonigiri_vision::BoxCenterFilter;
using umeonigiri_vision::SmoothedBox;

namespace {
SmoothedBox Box(double cx, double cy, int class_id = 0) {
  return SmoothedBox{cx - 5.0, cy - 10.0, cx + 5.0, cy + 10.0, 0.5, class_id};
}
}  // namespace

TEST(BoxCenterFilter, ReducesJitterAroundAStableTarget) {
  BoxCenterFilter filter(0.3);
  const std::vector<double> raw = {40.0, 41.0, 39.0, 40.5, 40.2, 39.8,
                                   40.1, 40.4, 39.9, 40.0};
  double raw_mean = 0.0;
  for (double value : raw) {
    raw_mean += value;
  }
  raw_mean /= raw.size();

  double smooth_mean = 0.0;
  double previous = 0.0;
  for (std::size_t i = 0; i < raw.size(); ++i) {
    const auto box = filter.Update(Box(raw[i], 30.0));
    const double center = (box.x0 + box.x1) * 0.5;
    if (i > 0) {
      smooth_mean += std::abs(center - previous);
    }
    previous = center;
  }
  EXPECT_LT(smooth_mean / (raw.size() - 1), 1.0);  // 生の+jitter より小さい
  EXPECT_NEAR(previous, raw_mean, 2.0);            // 中心は大きくずれない
}

TEST(BoxCenterFilter, ResetsWhenBoxJumpsToANewObject) {
  BoxCenterFilter filter;
  const auto first = filter.Update(Box(40.0, 30.0));
  EXPECT_NEAR((first.x0 + first.x1) * 0.5, 40.0, 1e-6);
  const auto far = filter.Update(Box(70.0, 30.0));
  EXPECT_NEAR((far.x0 + far.x1) * 0.5, 70.0, 1e-6);  // 12px 以上離れたら追わない
}

TEST(BoxCenterFilter, TracksClassesIndependently) {
  BoxCenterFilter filter;
  filter.Update(Box(40.0, 30.0, 0));
  const auto other = filter.Update(Box(60.0, 30.0, 5));
  EXPECT_NEAR((other.x0 + other.x1) * 0.5, 60.0, 1e-6);
  const auto first_again = filter.Update(Box(41.0, 30.0, 0));
  EXPECT_LT((first_again.x0 + first_again.x1) * 0.5, 41.0);  // class 0 は平滑化される
}

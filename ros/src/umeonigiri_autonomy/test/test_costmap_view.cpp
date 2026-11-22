// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <nav2_msgs/msg/costmap.hpp>

#include "umeonigiri_autonomy/costmap_view.hpp"

namespace umeonigiri_autonomy {
namespace {

nav2_msgs::msg::Costmap MakeCostmap(std::uint32_t size_x, std::uint32_t size_y,
                                    double resolution = 0.05) {
  nav2_msgs::msg::Costmap msg;
  msg.metadata.size_x = size_x;
  msg.metadata.size_y = size_y;
  msg.metadata.resolution = resolution;
  msg.metadata.origin.position.x = 0.0;
  msg.metadata.origin.position.y = 0.0;
  msg.data.assign(static_cast<std::size_t>(size_x) * size_y, 0);
  return msg;
}

}  // namespace

TEST(CostmapView, ExtractsLethalCellsWithCellCentres) {
  nav2_msgs::msg::Costmap msg = MakeCostmap(3, 2);
  // index = row * size_x + col。右下 (row1,col2) だけ LETHAL にする。
  msg.data[1 * 3 + 2] = 254;

  const RawCostmap raw = ParseRawCostmap(msg);
  ASSERT_EQ(raw.lethal.size(), 1u);
  // metadata.resolution は float32 なので 0.05 は厳密には 0.050000000745… になる。
  EXPECT_NEAR(raw.lethal[0].first, (2 + 0.5) * 0.05, 1e-6);
  EXPECT_NEAR(raw.lethal[0].second, (1 + 0.5) * 0.05, 1e-6);
}

TEST(CostmapView, IgnoresNonLethalValues) {
  nav2_msgs::msg::Costmap msg = MakeCostmap(2, 2);
  msg.data = {0, 100, 200, 253};  // 254 未満は LETHAL ではない

  const RawCostmap raw = ParseRawCostmap(msg);
  EXPECT_TRUE(raw.lethal.empty());
}

TEST(CostmapView, ZeroWidthDoesNotDivideByZero) {
  // 未初期化のコストマップは size_x=0 で配信され得る。以前は i % 0 で
  // 整数のゼロ除算 (SIGFPE) になっていた。
  nav2_msgs::msg::Costmap msg = MakeCostmap(0, 0);
  msg.data = {254, 254};  // 壊れた組み合わせ: データはあるが幅が無い

  const RawCostmap raw = ParseRawCostmap(msg);
  EXPECT_EQ(raw.view.width, 0);
  EXPECT_TRUE(raw.lethal.empty());
}

}  // namespace umeonigiri_autonomy

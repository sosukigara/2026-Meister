// SPDX-License-Identifier: MIT
/// MapListener が HTTP 面に渡す値の単体テスト（tf の平面合成と地図メタデータ）。
///
/// MapListener 本体（購読の配線）は rclcpp::init と ROS グラフを要するのでここでは
/// 検証しない。数値だけを固定する。配線は ✗ 未検証。
#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include <nlohmann/json.hpp>

#include "meister_web_nav/map_listener.hpp"
#include "test_grid.hpp"

namespace meister_web_nav {
namespace {

using test::Grid;
using test::MakeGrid;

constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::Quaternion YawOnly(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

TEST(YawOfTest, MatchesYawOnlyQuaternion) {
  EXPECT_NEAR(YawOf(YawOnly(0.0)), 0.0, 1e-12);
  EXPECT_NEAR(YawOf(YawOnly(kPi / 2.0)), kPi / 2.0, 1e-12);
  EXPECT_NEAR(YawOf(YawOnly(-kPi / 3.0)), -kPi / 3.0, 1e-12);
  // 半回転は atan2(0, -1) で pi。
  EXPECT_NEAR(YawOf(YawOnly(kPi)), kPi, 1e-12);
}

TEST(ComposePoseTest, RotatesChildOriginByParentYaw) {
  // map->odom が 90 度回転 + (1, 2)、odom->base が (1, 0)。子の原点は親の回転を
  // 受けるので base は (1, 3) に来る ((2, 2) ではない)。
  const PlanarTransform composed = ComposePose({1.0, 2.0, kPi / 2.0}, {1.0, 0.0, 0.1});
  EXPECT_NEAR(composed.tx, 1.0, 1e-9);
  EXPECT_NEAR(composed.ty, 3.0, 1e-9);
  EXPECT_NEAR(composed.yaw, kPi / 2.0 + 0.1, 1e-12);
}

TEST(ComposePoseTest, IdentityParentKeepsChild) {
  const PlanarTransform composed = ComposePose({0.0, 0.0, 0.0}, {3.0, -4.0, 0.5});
  EXPECT_NEAR(composed.tx, 3.0, 1e-12);
  EXPECT_NEAR(composed.ty, -4.0, 1e-12);
  EXPECT_NEAR(composed.yaw, 0.5, 1e-12);
}

TEST(ComposePoseTest, HalfTurnParentMovesChildBehind) {
  // 親が 180 度回ると、子の前進は親座標では後退になる。
  const PlanarTransform composed = ComposePose({0.0, 0.0, kPi}, {1.0, 0.0, 0.0});
  EXPECT_NEAR(composed.tx, -1.0, 1e-9);
  EXPECT_NEAR(composed.ty, 0.0, 1e-9);
}

TEST(MetadataOfTest, FullMapWithoutCrop) {
  const nlohmann::json meta = MetadataOf(*MakeGrid(), std::nullopt);
  EXPECT_EQ(meta.size(), 4u);
  // resolution は float32 なので 0.05 は厳密には 0.0500000007。Python 版も
  // メッセージの値をそのまま返すため、ここで丸めない。
  EXPECT_FLOAT_EQ(meta.at("resolution").get<double>(), 0.05);
  EXPECT_EQ(meta.at("width"), 32);
  EXPECT_EQ(meta.at("height"), 32);
  EXPECT_DOUBLE_EQ(meta.at("origin").at("x").get<double>(), -1.0);
  EXPECT_DOUBLE_EQ(meta.at("origin").at("y").get<double>(), -2.0);
}

TEST(MetadataOfTest, ShiftsOriginByCropOffset) {
  // crop で上端 (r0) が 8 セル削られると、画像の原点は 8 * resolution だけ動く。
  // これを返さないと Web UI のクリック座標が cell 単位でずれる。
  const nlohmann::json meta =
      MetadataOf(*MakeGrid(), CropBounds{/*r0=*/8, /*r1=*/13, /*c0=*/0, /*c1=*/32});
  EXPECT_EQ(meta.at("width"), 32);
  EXPECT_EQ(meta.at("height"), 5);
  EXPECT_DOUBLE_EQ(meta.at("origin").at("x").get<double>(), -1.0);
  EXPECT_NEAR(meta.at("origin").at("y").get<double>(), -1.6, 1e-6);
}

TEST(MetadataOfTest, ShiftsColumnOffsetToX) {
  // 縦 1 列だけ既知の地図では c0 側だけが動く。x に r0 を使う実装はここで落ちる。
  auto grid = MakeGrid(20, -1, -1);
  for (int r = 5; r <= 10; ++r) {
    grid->data[static_cast<std::size_t>(r) * 20 + 3] = 100;
  }
  const nlohmann::json meta = MetadataOf(*grid, CropBounds{/*r0=*/3, /*r1=*/13, /*c0=*/1, /*c1=*/6});
  EXPECT_EQ(meta.at("width"), 5);
  EXPECT_EQ(meta.at("height"), 10);
  EXPECT_NEAR(meta.at("origin").at("x").get<double>(), -1.0 + 0.05, 1e-6);
  EXPECT_NEAR(meta.at("origin").at("y").get<double>(), -2.0 + 0.15, 1e-6);
}

}  // namespace
}  // namespace meister_web_nav

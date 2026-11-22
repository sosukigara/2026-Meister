// SPDX-License-Identifier: MIT
/// waypoint_tour の単体テスト。ROS を起動しない。
#include <gtest/gtest.h>

#include "umeonigiri_autonomy/waypoint_tour.hpp"

namespace umeonigiri_autonomy {
namespace {

TEST(WaypointTour, HasNineStopsStartingBottomLeftAndEndingAtStart) {
  const auto& waypoints = TourWaypoints();
  ASSERT_EQ(waypoints.size(), 9u);
  EXPECT_DOUBLE_EQ(waypoints.front().x, -3.0);
  EXPECT_DOUBLE_EQ(waypoints.front().y, -1.5);
  EXPECT_DOUBLE_EQ(waypoints.back().x, -4.0);
  EXPECT_DOUBLE_EQ(waypoints.back().y, -4.0);
}

// 値は action_msgs/msg/GoalStatus の定数（SUCCEEDED=4 / CANCELED=5 / ABORTED=6）。
// 以前は 1/2/3 で書かれており、実機では成功が default に落ちて「失敗」と
// 表示されていた。テストが誤った値を固定していたので気付かなかった。
TEST(WaypointTour, SucceededGoesToNextStop) {
  EXPECT_EQ(ActionForResult(4), TourAction::kNext);
}

TEST(WaypointTour, CanceledStopsTheWholeTour) {
  EXPECT_EQ(ActionForResult(5), TourAction::kStop);
}

TEST(WaypointTour, AbortedSkipsOnlyThatStop) {
  EXPECT_EQ(ActionForResult(6), TourAction::kSkip);
}

TEST(WaypointTour, UnknownSkipsOnlyThatStop) {
  EXPECT_EQ(ActionForResult(0), TourAction::kSkip);
}

TEST(WaypointTour, AcceptedIsNotTreatedAsSuccess) {
  // 1 は STATUS_ACCEPTED。成功ではないので次へ進んではいけない。
  EXPECT_NE(ActionForResult(1), TourAction::kNext);
}

}  // namespace
}  // namespace umeonigiri_autonomy

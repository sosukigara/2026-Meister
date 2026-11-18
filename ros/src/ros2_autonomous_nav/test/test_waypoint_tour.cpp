// SPDX-License-Identifier: MIT
/// waypoint_tour の単体テスト。ROS を起動しない。
#include <gtest/gtest.h>

#include "ros2_autonomous_nav/waypoint_tour.hpp"

namespace ros2_autonomous_nav {
namespace {

TEST(WaypointTour, HasNineStopsStartingBottomLeftAndEndingAtStart) {
  const auto& waypoints = TourWaypoints();
  ASSERT_EQ(waypoints.size(), 9u);
  EXPECT_DOUBLE_EQ(waypoints.front().x, -3.0);
  EXPECT_DOUBLE_EQ(waypoints.front().y, -1.5);
  EXPECT_DOUBLE_EQ(waypoints.back().x, -4.0);
  EXPECT_DOUBLE_EQ(waypoints.back().y, -4.0);
}

TEST(WaypointTour, SucceededGoesToNextStop) {
  EXPECT_EQ(ActionForResult(1), TourAction::kNext);
}

TEST(WaypointTour, CanceledStopsTheWholeTour) {
  EXPECT_EQ(ActionForResult(2), TourAction::kStop);
}

TEST(WaypointTour, AbortedSkipsOnlyThatStop) {
  EXPECT_EQ(ActionForResult(3), TourAction::kSkip);
}

TEST(WaypointTour, UnknownSkipsOnlyThatStop) {
  EXPECT_EQ(ActionForResult(0), TourAction::kSkip);
}

}  // namespace
}  // namespace ros2_autonomous_nav

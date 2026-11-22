// SPDX-License-Identifier: MIT
#include "umeonigiri_autonomy/waypoint_tour.hpp"

namespace umeonigiri_autonomy {
namespace {

// action_msgs/msg/GoalStatus の定数。ROS 型に依存させないため整数で持つ。
constexpr int kStatusSucceeded = 4;
constexpr int kStatusCanceled = 5;

}  // namespace

const std::vector<TourWaypoint>& TourWaypoints() {
  static const std::vector<TourWaypoint> kWaypoints{
      {-3.0, -1.5, 0.0, "bottom-left area"},
      {-0.5, -0.5, 1.57, "center"},
      {-3.5, 3.5, 3.14, "top-left room"},
      {-0.5, 0.0, 0.0, "back to center"},
      {3.0, -0.5, 0.0, "right side gap"},
      {3.5, 1.0, 1.57, "right corridor"},
      {3.5, -3.5, -1.57, "bottom-right area"},
      {-0.5, -3.5, 3.14, "bottom corridor"},
      {-4.0, -4.0, 0.0, "start position"},
  };
  return kWaypoints;
}

TourAction ActionForResult(int result_code) {
  // 値は action_msgs/msg/GoalStatus の定数そのもの。rclcpp_action::ResultCode は
  // この値へ直接対応する (client_goal_handle.hpp)。
  //   UNKNOWN=0 ACCEPTED=1 EXECUTING=2 CANCELING=3 SUCCEEDED=4 CANCELED=5 ABORTED=6
  // 以前は 1/2/3 で判定していたため、成功 (4) が default に落ちて「失敗」と
  // 表示されていた (実測: Nav2 は "Goal succeeded" なのに Failed to reach)。
  switch (result_code) {
    case kStatusSucceeded:
      return TourAction::kNext;
    case kStatusCanceled:
      return TourAction::kStop;
    default:  // UNKNOWN / ABORTED / その他は この地点だけ飛ばす
      return TourAction::kSkip;
  }
}

}  // namespace umeonigiri_autonomy

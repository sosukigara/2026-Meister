// SPDX-License-Identifier: MIT
/// 迷路をひと巡りする固定ウェイポイント。以前は scripts/waypoint_follower.py。
#pragma once

#include <vector>

namespace ros2_autonomous_nav {

struct TourWaypoint {
  double x;
  double y;
  double yaw;
  const char* label;
};

/// 内部の壁の隙間を通る順路。値は Python 版から変えていない。
const std::vector<TourWaypoint>& TourWaypoints();

/// 1 ゴールの結果に対して次にどうするか。
enum class TourAction {
  kNext,  ///< 到達。次へ進む
  kStop,  ///< キャンセル。ツアー全体を止める
  kSkip,  ///< 失敗。この地点だけ飛ばして続ける
};

/// `rclcpp_action::ResultCode` の整数値を受ける。値は action_msgs/msg/GoalStatus の
/// 定数そのもの（UNKNOWN=0, ACCEPTED=1, EXECUTING=2, CANCELING=3,
/// SUCCEEDED=4, CANCELED=5, ABORTED=6）。ROS 型に依存させないため int で受ける。
TourAction ActionForResult(int result_code);

}  // namespace ros2_autonomous_nav

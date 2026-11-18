// SPDX-License-Identifier: MIT
#include "ros2_autonomous_nav/waypoint_tour.hpp"

namespace ros2_autonomous_nav {

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
  switch (result_code) {
    case 1:  // SUCCEEDED
      return TourAction::kNext;
    case 2:  // CANCELED
      return TourAction::kStop;
    default:  // UNKNOWN / ABORTED
      return TourAction::kSkip;
  }
}

}  // namespace ros2_autonomous_nav

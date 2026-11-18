// SPDX-License-Identifier: MIT
/// Nav2 に固定ウェイポイントを順に送る。以前は scripts/waypoint_follower.py。
///
/// 使い方:
///   ros2 run ros2_autonomous_nav waypoint_follower
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>

#include "ros2_autonomous_nav/waypoint_tour.hpp"

namespace {

using NavigateToPose = nav2_msgs::action::NavigateToPose;

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "waypoint_follower", rclcpp::NodeOptions().parameter_overrides(
                               {rclcpp::Parameter("use_sim_time", true)}));
  auto client = rclcpp_action::create_client<NavigateToPose>(node, "navigate_to_pose");

  if (!client->wait_for_action_server(std::chrono::seconds(30))) {
    std::fprintf(stderr, "ERROR: navigate_to_pose action server not available\n");
    rclcpp::shutdown();
    return 1;
  }

  const auto& waypoints = ros2_autonomous_nav::TourWaypoints();
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    const auto& waypoint = waypoints[i];
    std::printf("\n[%zu/%zu] Navigating to %s (%.1f, %.1f)...\n", i + 1, waypoints.size(),
                waypoint.label, waypoint.x, waypoint.y);

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = node->now();
    goal.pose.pose.position.x = waypoint.x;
    goal.pose.pose.position.y = waypoint.y;
    goal.pose.pose.orientation.z = std::sin(waypoint.yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(waypoint.yaw / 2.0);

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_options;
    send_options.feedback_callback =
        [](rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr,
           const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
          std::printf("  Distance remaining: %.2f m\r", feedback->distance_remaining);
          std::fflush(stdout);
        };

    auto handle = client->async_send_goal(goal, send_options).get();
    if (handle == nullptr) {
      std::fprintf(stderr, "  goal rejected, skipping...\n");
      continue;
    }

    const auto result = client->async_get_result(handle).get();

    switch (ros2_autonomous_nav::ActionForResult(static_cast<int>(result.code))) {
      case ros2_autonomous_nav::TourAction::kNext:
        std::printf("  Reached %s!\n", waypoint.label);
        break;
      case ros2_autonomous_nav::TourAction::kStop:
        std::printf("  Navigation to %s was canceled.\n", waypoint.label);
        std::printf("\nWaypoint tour canceled.\n");
        rclcpp::shutdown();
        return 0;
      case ros2_autonomous_nav::TourAction::kSkip:
        std::printf("  Failed to reach %s, skipping...\n", waypoint.label);
        break;
    }
  }

  std::printf("\nWaypoint tour complete!\n");
  rclcpp::shutdown();
  return 0;
}

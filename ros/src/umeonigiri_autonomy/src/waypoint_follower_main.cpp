// SPDX-License-Identifier: MIT
/// Nav2 に固定ウェイポイントを順に送る。以前は scripts/waypoint_follower.py。
///
/// 使い方:
///   ros2 run umeonigiri_autonomy waypoint_follower
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>

#include "umeonigiri_autonomy/waypoint_tour.hpp"

namespace {

using NavigateToPose = nav2_msgs::action::NavigateToPose;

/// ゴール要求の応答待ち。サーバ側は受けたらすぐ応答するので、これを超えるのは
/// 異常（サーバが詰まっている）とみなして次へ進む。
constexpr auto kGoalResponseTimeout = std::chrono::seconds(10);

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

  const auto& waypoints = umeonigiri_autonomy::TourWaypoints();
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

    // future を .get() で直接待つと駄目。アクションの応答は executor が回って
    // 初めて配送されるので、何も spin しないまま待つと永久に返らない。
    // 実測: Nav2 側はゴールを受けて走行まで完了しているのに、このツアーは
    // 1 地点目で futex_wait のまま止まり、フィードバックも出なかった。
    auto goal_future = client->async_send_goal(goal, send_options);
    if (rclcpp::spin_until_future_complete(node, goal_future, kGoalResponseTimeout) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      std::fprintf(stderr, "  goal request timed out, skipping...\n");
      continue;
    }
    auto handle = goal_future.get();
    if (handle == nullptr) {
      std::fprintf(stderr, "  goal rejected, skipping...\n");
      continue;
    }

    // 走行中は結果が返るまで spin し続ける。ここで回すことで feedback_callback も届く。
    auto result_future = client->async_get_result(handle);
    if (rclcpp::spin_until_future_complete(node, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      // Ctrl+C で止められた場合もここに来る。走行を残さないようキャンセルする。
      client->async_cancel_goal(handle);
      std::printf("\nWaypoint tour interrupted.\n");
      rclcpp::shutdown();
      return 1;
    }
    const auto result = result_future.get();

    switch (umeonigiri_autonomy::ActionForResult(static_cast<int>(result.code))) {
      case umeonigiri_autonomy::TourAction::kNext:
        std::printf("  Reached %s!\n", waypoint.label);
        break;
      case umeonigiri_autonomy::TourAction::kStop:
        std::printf("  Navigation to %s was canceled.\n", waypoint.label);
        std::printf("\nWaypoint tour canceled.\n");
        rclcpp::shutdown();
        return 0;
      case umeonigiri_autonomy::TourAction::kSkip:
        // 結果コードを出す。Nav2 側は "Goal succeeded" なのに失敗として扱われる
        // ような食い違いを、現場で切り分けられるようにするため。
        std::printf("  Failed to reach %s (result code %d), skipping...\n",
                    waypoint.label, static_cast<int>(result.code));
        break;
    }
  }

  std::printf("\nWaypoint tour complete!\n");
  rclcpp::shutdown();
  return 0;
}

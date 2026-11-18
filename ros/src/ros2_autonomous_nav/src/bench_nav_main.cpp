// SPDX-License-Identifier: MIT
/// ナビゲーションのベンチマーク。固定ゴールを順に走らせて JSON に記録する。
///
/// 設計書 docs/superpowers/specs/2026-11-18-navigation-design.md §8.1/§8.2。
/// 指標は bench_metrics、引数は bench_options、メッセージ変換は costmap_view、
/// JSON は bench_report、/proc は bench_proc にある。ここは ROS の配線と
/// 実行ループだけを持つ。
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "ros2_autonomous_nav/bench_metrics.hpp"
#include "ros2_autonomous_nav/bench_options.hpp"
#include "ros2_autonomous_nav/bench_proc.hpp"
#include "ros2_autonomous_nav/bench_report.hpp"
#include "ros2_autonomous_nav/costmap_view.hpp"

namespace {

using json = nlohmann::json;
using ros2_autonomous_nav::BenchOptions;
using ros2_autonomous_nav::Goal;
using ros2_autonomous_nav::GridView;
using ros2_autonomous_nav::Pose2;
using ros2_autonomous_nav::RunRecord;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

double YawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std::string NowIso8601() {
  const std::time_t now = std::time(nullptr);
  char buffer[40];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S%z", std::localtime(&now));
  return buffer;
}

std::string LoadAverageText() {
  double load = 0.0;
  if (::getloadavg(&load, 1) != 1) {
    return "unknown";
  }
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.2f", load);
  return buffer;
}

}  // namespace

int main(int argc, char** argv) {
  BenchOptions options;
  const auto status = ros2_autonomous_nav::ParseBenchArgs(argc, argv, &options);
  if (status == ros2_autonomous_nav::ParseStatus::kHelp) {
    return 0;
  }
  if (status != ros2_autonomous_nav::ParseStatus::kOk) {
    return 1;
  }

  json doc;
  std::vector<Goal> goals;
  try {
    std::ifstream goals_in(options.goals);
    if (!goals_in) {
      std::fprintf(stderr, "ERROR: cannot read %s\n", options.goals.c_str());
      return 2;
    }
    goals_in >> doc;
    for (const auto& entry : doc.at("goals")) {
      goals.push_back(Goal{entry.at("x").get<double>(), entry.at("y").get<double>(),
                           entry.value("yaw", 0.0)});
    }
  } catch (const std::exception& error) {
    std::fprintf(stderr, "ERROR: malformed goal file %s: %s\n", options.goals.c_str(),
                 error.what());
    return 2;
  }
  if (options.limit >= 0 && static_cast<std::size_t>(options.limit) < goals.size()) {
    goals.resize(static_cast<std::size_t>(options.limit));
  }
  const int repeats = options.repeats >= 0 ? options.repeats : doc.value("repeats", 1);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "bench_nav", rclcpp::NodeOptions().parameter_overrides(
                       {rclcpp::Parameter("use_sim_time", true)}));
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  std::optional<nav2_msgs::msg::Costmap> raw_msg;
  std::optional<nav_msgs::msg::OccupancyGrid> occupancy_msg;
  std::optional<nav_msgs::msg::Path> plan;
  double plan_first_at = 0.0;
  std::vector<double> cmd_times;

  auto raw_sub = node->create_subscription<nav2_msgs::msg::Costmap>(
      "/global_costmap/costmap_raw", 10,
      [&raw_msg](nav2_msgs::msg::Costmap::ConstSharedPtr msg) { raw_msg = *msg; });
  auto occupancy_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap/costmap", 10,
      [&occupancy_msg](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        occupancy_msg = *msg;
      });
  auto plan_sub = node->create_subscription<nav_msgs::msg::Path>(
      "plan", 10, [&](nav_msgs::msg::Path::ConstSharedPtr msg) {
        plan = *msg;
        if (plan_first_at == 0.0) {
          plan_first_at = node->now().seconds();
        }
      });
  auto cmd_sub = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_nav", 10, [&](geometry_msgs::msg::Twist::ConstSharedPtr) {
        cmd_times.push_back(node->now().seconds());
      });
  auto client = rclcpp_action::create_client<NavigateToPose>(node, "navigate_to_pose");

  auto spin_for = [&node](double seconds) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  };

  const std::string started_at = NowIso8601();
  const std::string load_average = LoadAverageText();

  spin_for(8.0);
  if (!raw_msg.has_value() || !occupancy_msg.has_value()) {
    const char* domain = std::getenv("ROS_DOMAIN_ID");
    std::fprintf(stderr,
                 "ERROR: costmap not received (ROS_DOMAIN_ID=%s, ROS_LOCALHOST_ONLY=%s). "
                 "launch.sh と同じ環境変数を設定してください。\n",
                 domain != nullptr ? domain : "unset",
                 std::getenv("ROS_LOCALHOST_ONLY") != nullptr ? "set" : "unset");
    rclcpp::shutdown();
    return 2;
  }

  const GridView free_grid = ros2_autonomous_nav::OccupancyToView(*occupancy_msg);
  const auto raw = ros2_autonomous_nav::ParseRawCostmap(*raw_msg);
  const std::vector<ros2_autonomous_nav::Cell>& occupied = raw.lethal;

  if (options.auto_goals > 0) {
    const auto picked = ros2_autonomous_nav::SelectFreeGoals(
        free_grid, occupied, options.auto_goals, options.min_spacing,
        options.min_clearance, ros2_autonomous_nav::kFootprintHalfLength,
        ros2_autonomous_nav::kFootprintHalfWidth);
    if (static_cast<int>(picked.size()) < options.auto_goals) {
      std::fprintf(stderr, "ERROR: only %zu goals found (need %d)\n", picked.size(),
                   options.auto_goals);
      rclcpp::shutdown();
      return 2;
    }
    json picked_json = json::array();
    for (const auto& goal : picked) {
      picked_json.push_back(json{{"x", goal.x}, {"y", goal.y}, {"yaw", goal.yaw}});
      std::printf("goal: (%.2f, %.2f)\n", goal.x, goal.y);
    }
    doc["goals"] = picked_json;
    std::ofstream out(options.goals);
    out << doc.dump(2) << "\n";
    std::printf("wrote %s\n", options.goals.c_str());
    rclcpp::shutdown();
    return 0;
  }

  const auto problems = ros2_autonomous_nav::ValidateGoals(goals, free_grid);
  if (!problems.empty()) {
    for (const auto& problem : problems) {
      std::fprintf(stderr, "ERROR: %s\n", problem.c_str());
    }
    rclcpp::shutdown();
    return 2;
  }

  if (!client->wait_for_action_server(std::chrono::seconds(10))) {
    std::fprintf(stderr, "ERROR: navigate_to_pose action server not available\n");
    rclcpp::shutdown();
    return 2;
  }

  std::vector<RunRecord> runs;
  std::map<std::string, double> cpu_seconds_by_node;
  double cpu_wall_seconds = 0.0;

  for (int repeat = 0; repeat < repeats; ++repeat) {
    for (std::size_t index = 0; index < goals.size(); ++index) {
      const Goal& goal = goals[index];
      std::printf("[%d/%d] goal %zu: (%.2f, %.2f, %.2f)\n", repeat + 1, repeats, index,
                  goal.x, goal.y, goal.yaw);

      plan.reset();
      plan_first_at = 0.0;
      cmd_times.clear();
      std::vector<Pose2> trajectory;
      const auto cpu_before = ros2_autonomous_nav::SampleNavCpu();
      const auto wall_before = std::chrono::steady_clock::now();
      const double started = node->now().seconds();
      double last_sample = 0.0;

      NavigateToPose::Goal goal_msg;
      goal_msg.pose.header.frame_id = "map";
      goal_msg.pose.header.stamp.sec = 0;
      goal_msg.pose.header.stamp.nanosec = 0;
      goal_msg.pose.pose.position.x = goal.x;
      goal_msg.pose.pose.position.y = goal.y;
      goal_msg.pose.pose.orientation.z = std::sin(goal.yaw / 2.0);
      goal_msg.pose.pose.orientation.w = std::cos(goal.yaw / 2.0);

      auto goal_future = client->async_send_goal(goal_msg);
      while (goal_future.wait_for(std::chrono::milliseconds(50)) !=
             std::future_status::ready) {
        rclcpp::spin_some(node);
        if (node->now().seconds() - started > options.goal_timeout) {
          break;
        }
      }
      auto handle = goal_future.wait_for(std::chrono::seconds(0)) ==
                            std::future_status::ready
                        ? goal_future.get()
                        : nullptr;
      if (handle == nullptr) {
        runs.emplace_back();
        continue;
      }
      // 計画時間は「アクションが受理されてから /plan が届くまで」で測る。
      // 送信前を起点にすると DDS と行動サーバの往復まで含んで過大になる。
      const double accepted_at = node->now().seconds();

      auto result_future = client->async_get_result(handle);
      while (result_future.wait_for(std::chrono::milliseconds(50)) !=
             std::future_status::ready) {
        rclcpp::spin_some(node);
        const double now = node->now().seconds();
        if (now - last_sample >= 1.0 / options.sample_hz) {
          last_sample = now;
          try {
            const auto tf =
                tf_buffer.lookupTransform("map", "base_footprint", tf2::TimePointZero);
            trajectory.push_back(Pose2{tf.transform.translation.x,
                                       tf.transform.translation.y,
                                       YawFromQuaternion(tf.transform.rotation)});
          } catch (const tf2::TransformException&) {
            // TF がまだ張られていない間は記録しない。
          }
        }
        if (now - started > options.goal_timeout) {
          client->async_cancel_goal(handle);
          break;
        }
      }

      try {
        const auto tf = tf_buffer.lookupTransform("map", "base_footprint", tf2::TimePointZero);
        trajectory.push_back(Pose2{tf.transform.translation.x, tf.transform.translation.y,
                                   YawFromQuaternion(tf.transform.rotation)});
      } catch (const tf2::TransformException&) {
      }

      const auto result = result_future.get();
      RunRecord run;
      run.success = result.code == rclcpp_action::ResultCode::SUCCEEDED;
      run.duration_s = node->now().seconds() - started;
      run.path_length_m = ros2_autonomous_nav::PathLength(trajectory);

      const auto cpu_after = ros2_autonomous_nav::SampleNavCpu();
      cpu_wall_seconds +=
          std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_before).count();
      for (const auto& [pid, sample] : cpu_after) {
        const auto before = cpu_before.find(pid);
        if (before == cpu_before.end()) {
          continue;
        }
        cpu_seconds_by_node[sample.node] += sample.cpu_seconds - before->second.cpu_seconds;
      }

      if (plan.has_value()) {
        double planned = 0.0;
        for (std::size_t i = 1; i < plan->poses.size(); ++i) {
          planned += std::hypot(
              plan->poses[i].pose.position.x - plan->poses[i - 1].pose.position.x,
              plan->poses[i].pose.position.y - plan->poses[i - 1].pose.position.y);
        }
        run.planned_length_m = planned;
        run.path_length_ratio =
            ros2_autonomous_nav::PathLengthRatio(run.path_length_m, planned);
      }
      if (plan_first_at > 0.0) {
        run.planner_latency_ms = (plan_first_at - accepted_at) * 1000.0;
      }
      const auto control = ros2_autonomous_nav::ComputeIntervalStats(cmd_times);
      if (cmd_times.size() >= 2) {
        run.control_hz = control.effective_hz;
        run.control_p99_ms = control.p99_ms;
      }

      if (trajectory.size() >= 2) {
        const Pose2& last = trajectory.back();
        run.final_error_xy = std::hypot(last.x - goal.x, last.y - goal.y);
        run.final_error_yaw =
            std::abs(std::atan2(std::sin(last.yaw - goal.yaw), std::cos(last.yaw - goal.yaw)));
        run.min_clearance_m = ros2_autonomous_nav::MinFootprintClearance(
            trajectory, occupied, ros2_autonomous_nav::kFootprintHalfLength,
            ros2_autonomous_nav::kFootprintHalfWidth);
        run.contact_count = ros2_autonomous_nav::ContactCount(
            trajectory, occupied, ros2_autonomous_nav::kFootprintHalfLength,
            ros2_autonomous_nav::kFootprintHalfWidth);
      } else {
        run.success = false;
        run.path_length_m = 0.0;
      }
      runs.push_back(run);
    }
  }

  json cpu_json = json::object();
  for (const auto& [name, seconds] : cpu_seconds_by_node) {
    cpu_json[name] = cpu_wall_seconds > 0.0 ? seconds / cpu_wall_seconds : 0.0;
  }

  json payload;
  payload["world"] = doc.value("world", std::string{});
  payload["map_id"] = options.map_id;
  payload["gui"] = options.gui;
  payload["started_at"] = started_at;
  payload["load_average"] = load_average;
  payload["runs"] = json::parse(ros2_autonomous_nav::RunsToJsonText(runs));
  payload["cpu_core_fraction"] = cpu_json;
  payload["cpu_core_fraction_total"] = 0.0;
  for (const auto& item : cpu_json.items()) {
    payload["cpu_core_fraction_total"] =
        payload["cpu_core_fraction_total"].get<double>() + item.value().get<double>();
  }
  payload["summary"] = json::parse(ros2_autonomous_nav::SummaryToJsonText(
      ros2_autonomous_nav::Summarize(runs)));

  std::ofstream out(options.out);
  out << payload.dump(2) << "\n";
  std::printf("%s\n", payload["summary"].dump(2).c_str());

  rclcpp::shutdown();
  return 0;
}

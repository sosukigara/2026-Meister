// SPDX-License-Identifier: MIT
/// ナビゲーションのベンチマーク。固定ゴールを順に走らせて JSON に記録する。
///
/// 設計書 docs/superpowers/specs/2026-11-18-navigation-design.md §8.1/§8.2。
/// 指標の計算は bench_metrics.{hpp,cpp} にあり、ここは ROS との接続だけを持つ。
///
/// 障害物セルは /global_costmap/costmap_raw（生の 0..255）から LETHAL 以上だけを
/// 拾う。/global_costmap/costmap は 0..100 に変換済みで内接リング(253)を含むため、
/// そのまま使うと距離が実際より robot_radius ぶん小さく出る。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <action_msgs/msg/goal_status.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "ros2_autonomous_nav/bench_metrics.hpp"

namespace {

using json = nlohmann::json;
using ros2_autonomous_nav::Cell;
using ros2_autonomous_nav::Goal;
using ros2_autonomous_nav::GridView;
using ros2_autonomous_nav::Pose2;
using ros2_autonomous_nav::RunRecord;

// 機体寸法 [m]。設計書 §3: 0.6 x 0.4。
constexpr double kFootprintHalfLength = 0.30;
constexpr double kFootprintHalfWidth = 0.20;

struct Options {
  std::string goals;
  std::string out;
  std::string map_id;
  int repeats{-1};
  int limit{-1};
  int auto_goals{-1};
  double goal_timeout{120.0};
  double sample_hz{10.0};
  double min_spacing{1.0};
  double min_clearance{0.5};
};

void PrintUsage() {
  std::fprintf(stderr,
               "usage: bench_nav --goals FILE --out FILE [--repeats N] [--limit N]\n"
               "                 [--goal-timeout S] [--sample-hz HZ] [--map-id ID]\n"
               "                 [--auto-goals N] [--min-spacing M] [--min-clearance M]\n");
}

bool ParseArgs(int argc, char** argv, Options* options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const char* name) -> std::optional<std::string> {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "ERROR: %s requires a value\n", name);
        return std::nullopt;
      }
      return std::string(argv[++i]);
    };
    if (arg == "--help" || arg == "-h") {
      PrintUsage();
      return false;
    } else if (arg == "--goals") {
      auto value = next("--goals");
      if (!value) return false;
      options->goals = *value;
    } else if (arg == "--out") {
      auto value = next("--out");
      if (!value) return false;
      options->out = *value;
    } else if (arg == "--map-id") {
      auto value = next("--map-id");
      if (!value) return false;
      options->map_id = *value;
    } else if (arg == "--repeats") {
      auto value = next("--repeats");
      if (!value) return false;
      options->repeats = std::stoi(*value);
    } else if (arg == "--limit") {
      auto value = next("--limit");
      if (!value) return false;
      options->limit = std::stoi(*value);
    } else if (arg == "--auto-goals") {
      auto value = next("--auto-goals");
      if (!value) return false;
      options->auto_goals = std::stoi(*value);
    } else if (arg == "--goal-timeout") {
      auto value = next("--goal-timeout");
      if (!value) return false;
      options->goal_timeout = std::stod(*value);
    } else if (arg == "--sample-hz") {
      auto value = next("--sample-hz");
      if (!value) return false;
      options->sample_hz = std::stod(*value);
    } else if (arg == "--min-spacing") {
      auto value = next("--min-spacing");
      if (!value) return false;
      options->min_spacing = std::stod(*value);
    } else if (arg == "--min-clearance") {
      auto value = next("--min-clearance");
      if (!value) return false;
      options->min_clearance = std::stod(*value);
    } else {
      std::fprintf(stderr, "ERROR: unknown argument %s\n", arg.c_str());
      PrintUsage();
      return false;
    }
  }
  if (options->goals.empty() || options->out.empty()) {
    PrintUsage();
    return false;
  }
  return true;
}

double YawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

GridView OccupancyToView(const nav_msgs::msg::OccupancyGrid& msg) {
  GridView view;
  view.width = static_cast<int>(msg.info.width);
  view.height = static_cast<int>(msg.info.height);
  view.resolution = msg.info.resolution;
  view.origin_x = msg.info.origin.position.x;
  view.origin_y = msg.info.origin.position.y;
  view.data.assign(msg.data.begin(), msg.data.end());
  return view;
}

GridView RawToView(const nav2_msgs::msg::Costmap& msg) {
  GridView view;
  view.width = static_cast<int>(msg.metadata.size_x);
  view.height = static_cast<int>(msg.metadata.size_y);
  view.resolution = msg.metadata.resolution;
  view.origin_x = msg.metadata.origin.position.x;
  view.origin_y = msg.metadata.origin.position.y;
  view.data.assign(msg.data.begin(), msg.data.end());
  return view;
}

std::vector<Cell> LethalCells(const nav2_msgs::msg::Costmap& msg) {
  std::vector<Cell> cells;
  const auto size_x = static_cast<int>(msg.metadata.size_x);
  const double res = msg.metadata.resolution;
  const double ox = msg.metadata.origin.position.x;
  const double oy = msg.metadata.origin.position.y;
  for (std::size_t i = 0; i < msg.data.size(); ++i) {
    if (msg.data[i] < ros2_autonomous_nav::kLethalRaw) {
      continue;
    }
    const int col = static_cast<int>(i) % size_x;
    const int row = static_cast<int>(i) / size_x;
    cells.emplace_back(ox + (col + 0.5) * res, oy + (row + 0.5) * res);
  }
  return cells;
}

json SummaryToJson(const ros2_autonomous_nav::RunSummary& summary) {
  auto opt = [](const std::optional<double>& value) -> json {
    return value.has_value() ? json(*value) : json(nullptr);
  };
  return json{{"total", summary.total},
              {"measured", summary.measured},
              {"success_rate", summary.success_rate},
              {"max_final_error_xy", opt(summary.max_final_error_xy)},
              {"max_final_error_yaw", opt(summary.max_final_error_yaw)},
              {"min_clearance_m", opt(summary.min_clearance_m)},
              {"mean_duration_s", opt(summary.mean_duration_s)}};
}

json RunToJson(const RunRecord& run) {
  auto opt = [](const std::optional<double>& value) -> json {
    return value.has_value() ? json(*value) : json(nullptr);
  };
  auto opt_int = [](const std::optional<int>& value) -> json {
    return value.has_value() ? json(*value) : json(nullptr);
  };
  return json{{"success", run.success},
              {"duration_s", run.duration_s},
              {"path_length_m", run.path_length_m},
              {"planned_length_m", run.planned_length_m},
              {"path_length_ratio", opt(run.path_length_ratio)},
              {"final_error_xy", opt(run.final_error_xy)},
              {"final_error_yaw", opt(run.final_error_yaw)},
              {"min_clearance_m", opt(run.min_clearance_m)},
              {"contact_count", opt_int(run.contact_count)},
              {"planner_latency_ms", opt(run.planner_latency_ms)},
              {"control_hz", opt(run.control_hz)},
              {"control_p99_ms", opt(run.control_p99_ms)}};
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!ParseArgs(argc, argv, &options)) {
    return 1;
  }

  std::ifstream goals_in(options.goals);
  if (!goals_in) {
    std::fprintf(stderr, "ERROR: cannot read %s\n", options.goals.c_str());
    return 2;
  }
  json doc;
  goals_in >> doc;

  std::vector<Goal> goals;
  for (const auto& entry : doc.at("goals")) {
    goals.push_back(Goal{entry.at("x").get<double>(), entry.at("y").get<double>(),
                         entry.value("yaw", 0.0)});
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

  std::optional<nav2_msgs::msg::Costmap> raw;
  std::optional<nav_msgs::msg::OccupancyGrid> occupancy;
  std::optional<nav_msgs::msg::Path> plan;
  double plan_first_at{0.0};
  std::vector<double> cmd_times;

  auto raw_sub = node->create_subscription<nav2_msgs::msg::Costmap>(
      "/global_costmap/costmap_raw", 10,
      [&raw](nav2_msgs::msg::Costmap::ConstSharedPtr msg) { raw = *msg; });
  auto occupancy_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap/costmap", 10,
      [&occupancy](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { occupancy = *msg; });
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

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  auto client = rclcpp_action::create_client<NavigateToPose>(node, "navigate_to_pose");

  auto spin_for = [&node](double seconds) {
    const auto end = std::chrono::steady_clock::now() +
                     std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  };

  spin_for(8.0);
  if (!raw.has_value() || !occupancy.has_value()) {
    const char* domain = std::getenv("ROS_DOMAIN_ID");
    std::fprintf(stderr,
                 "ERROR: costmap not received (ROS_DOMAIN_ID=%s, "
                 "ROS_LOCALHOST_ONLY=%s). launch.sh と同じ環境変数を設定してください。\n",
                 domain != nullptr ? domain : "unset",
                 std::getenv("ROS_LOCALHOST_ONLY") != nullptr ? "set" : "unset");
    rclcpp::shutdown();
    return 2;
  }

  const GridView free_grid = OccupancyToView(*occupancy);
  const GridView raw_grid = RawToView(*raw);
  const std::vector<Cell> occupied = LethalCells(*raw);
  (void)raw_grid;

  auto is_free = [&free_grid](double x, double y) {
    const auto cost = ros2_autonomous_nav::CellCost(free_grid, x, y);
    return cost.has_value() && *cost >= 0 && *cost < ros2_autonomous_nav::kFreeCostMax;
  };

  if (options.auto_goals > 0) {
    const auto picked = ros2_autonomous_nav::SelectFreeGoals(
        free_grid, occupied, options.auto_goals, options.min_spacing,
        options.min_clearance, kFootprintHalfLength, kFootprintHalfWidth);
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

  if (goals.empty()) {
    std::fprintf(stderr, "ERROR: no goals defined\n");
    rclcpp::shutdown();
    return 2;
  }
  for (std::size_t i = 0; i < goals.size(); ++i) {
    if (!is_free(goals[i].x, goals[i].y)) {
      std::fprintf(stderr, "ERROR: goal %zu (%.1f, %.1f) is not in free space\n", i,
                   goals[i].x, goals[i].y);
      rclcpp::shutdown();
      return 2;
    }
  }

  if (!client->wait_for_action_server(std::chrono::seconds(10))) {
    std::fprintf(stderr, "ERROR: navigate_to_pose action server not available\n");
    rclcpp::shutdown();
    return 2;
  }

  std::vector<RunRecord> runs;
  std::vector<Pose2> all_trajectories;
  (void)all_trajectories;

  for (int repeat = 0; repeat < repeats; ++repeat) {
    for (std::size_t index = 0; index < goals.size(); ++index) {
      const Goal& goal = goals[index];
      std::printf("[%d/%d] goal %zu: (%.2f, %.2f, %.2f)\n", repeat + 1, repeats, index,
                  goal.x, goal.y, goal.yaw);

      plan.reset();
      plan_first_at = 0.0;
      cmd_times.clear();
      std::vector<Pose2> trajectory;
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
        RunRecord run;
        runs.push_back(run);
        continue;
      }

      auto result_future = client->async_get_result(handle);
      while (result_future.wait_for(std::chrono::milliseconds(50)) !=
             std::future_status::ready) {
        rclcpp::spin_some(node);
        const double now = node->now().seconds();
        if (now - last_sample >= 1.0 / options.sample_hz) {
          last_sample = now;
          try {
            const auto tf = tf_buffer.lookupTransform("map", "base_footprint", tf2::TimePointZero);
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
        trajectory.push_back(Pose2{tf.transform.translation.x,
                                   tf.transform.translation.y,
                                   YawFromQuaternion(tf.transform.rotation)});
      } catch (const tf2::TransformException&) {
      }

      const auto wrapped = result_future.get();
      const bool success = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED;
      const double elapsed = node->now().seconds() - started;

      RunRecord run;
      run.success = success;
      run.duration_s = elapsed;
      run.path_length_m = ros2_autonomous_nav::PathLength(trajectory);
      if (plan.has_value()) {
        double planned = 0.0;
        for (std::size_t i = 1; i < plan->poses.size(); ++i) {
          planned += std::hypot(plan->poses[i].pose.position.x - plan->poses[i - 1].pose.position.x,
                                plan->poses[i].pose.position.y - plan->poses[i - 1].pose.position.y);
        }
        run.planned_length_m = planned;
        run.path_length_ratio = ros2_autonomous_nav::PathLengthRatio(run.path_length_m, planned);
      }
      if (plan_first_at > 0.0) {
        run.planner_latency_ms = (plan_first_at - started) * 1000.0;
      }
      const auto control = ros2_autonomous_nav::ComputeIntervalStats(cmd_times);
      if (cmd_times.size() >= 2) {
        run.control_hz = control.effective_hz;
        run.control_p99_ms = control.p99_ms;
      }

      if (trajectory.size() >= 2) {
        const Pose2& last = trajectory.back();
        run.final_error_xy = std::hypot(last.x - goal.x, last.y - goal.y);
        const double dyaw = std::atan2(std::sin(last.yaw - goal.yaw),
                                       std::cos(last.yaw - goal.yaw));
        run.final_error_yaw = std::abs(dyaw);
        run.min_clearance_m = ros2_autonomous_nav::MinFootprintClearance(
            trajectory, occupied, kFootprintHalfLength, kFootprintHalfWidth);
        run.contact_count = ros2_autonomous_nav::ContactCount(
            trajectory, occupied, kFootprintHalfLength, kFootprintHalfWidth);
      } else {
        run.success = false;
        run.path_length_m = 0.0;
      }
      runs.push_back(run);
    }
  }

  json runs_json = json::array();
  for (const auto& run : runs) {
    runs_json.push_back(RunToJson(run));
  }
  const auto summary = ros2_autonomous_nav::Summarize(runs);
  const json payload{{"world", doc.value("world", std::string{})},
                     {"map_id", options.map_id},
                     {"runs", runs_json},
                     {"summary", SummaryToJson(summary)}};

  std::ofstream out(options.out);
  out << payload.dump(2) << "\n";
  std::printf("%s\n", payload["summary"].dump(2).c_str());

  rclcpp::shutdown();
  return 0;
}

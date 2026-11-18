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
#include <filesystem>
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
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
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
  std::vector<Pose2> first_plan_poses;
  double plan_first_at = 0.0;
  // 受理直後に届いた最初の計画の長さ。最後の plan はゴール直前の残りで短い。
  double first_plan_length = 0.0;
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
          double length = 0.0;
          for (std::size_t i = 1; i < msg->poses.size(); ++i) {
            length += std::hypot(
                msg->poses[i].pose.position.x - msg->poses[i - 1].pose.position.x,
                msg->poses[i].pose.position.y - msg->poses[i - 1].pose.position.y);
          }
          first_plan_length = length;
          first_plan_poses.clear();
          for (const auto& pose : msg->poses) {
            first_plan_poses.push_back(Pose2{pose.pose.position.x, pose.pose.position.y,
                                             YawFromQuaternion(pose.pose.orientation)});
          }
        }
      });
  auto cmd_sub = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_nav", 10, [&](geometry_msgs::msg::Twist::ConstSharedPtr) {
        cmd_times.push_back(node->now().seconds());
      });
  // 構成によっては controller の出力が直接 /cmd_vel に出る。どちらでも測れるようにする。
  auto cmd_smoothed_sub = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, [&](geometry_msgs::msg::Twist::ConstSharedPtr) {
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

  // 自己診断: /clock と TF がこのノードに届いているかを事実で確かめる。
  if (options.self_check > 0.0) {
    int clock_count = 0;
    int tf_count = 0;
    int odom_count = 0;
    auto clock_sub = node->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", 10, [&](rosgraph_msgs::msg::Clock::ConstSharedPtr) { ++clock_count; });
    auto tf_sub = node->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", 100, [&](tf2_msgs::msg::TFMessage::ConstSharedPtr) { ++tf_count; });
    auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, [&](nav_msgs::msg::Odometry::ConstSharedPtr) { ++odom_count; });
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(options.self_check);
    int tick = 0;
    while (std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node);
      if (++tick % 50 == 0) {
        std::string status;
        try {
          const auto tf =
              tf_buffer.lookupTransform("map", "base_footprint", tf2::TimePointZero);
          char text[160];
          std::snprintf(text, sizeof(text), "ok x=%.3f y=%.3f stamp=%.3f",
                        tf.transform.translation.x, tf.transform.translation.y,
                        static_cast<double>(tf.header.stamp.sec) +
                            static_cast<double>(tf.header.stamp.nanosec) * 1e-9);
          status = text;
        } catch (const tf2::TransformException& error) {
          status = std::string("throw ") + error.what();
        }
        std::printf("[self] now=%.3f clock=%d tf=%d odom=%d map=%s\n",
                    node->now().seconds(), clock_count, tf_count, odom_count, status.c_str());
        std::fflush(stdout);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    rclcpp::shutdown();
    return 0;
  }

  const std::string started_at = NowIso8601();
  const std::string load_average = LoadAverageText();

  spin_for(25.0);  // コストマップは 1 Hz。起動直後を取りこぼさないよう長めに待つ。
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

      // 軌跡は送信直後から採る。受理応答が数秒遅れることがあり、その間にも
      // 車体は動くので、受理待ちの間に採らないと軌跡が欠ける。
      auto sample_pose = [&]() {
        const double now = node->now().seconds();
        if (now - last_sample < 1.0 / options.sample_hz) {
          return;
        }
        last_sample = now;
        try {
          const auto tf =
              tf_buffer.lookupTransform("map", "base_footprint", tf2::TimePointZero);
          trajectory.push_back(Pose2{tf.transform.translation.x, tf.transform.translation.y,
                                     YawFromQuaternion(tf.transform.rotation)});
        } catch (const tf2::TransformException&) {
          // TF がまだ張られていない間は記録しない。
        }
      };

      const double sent_at = node->now().seconds();
      plan.reset();
      plan_first_at = 0.0;
      first_plan_length = 0.0;
      first_plan_poses.clear();
      auto goal_future = client->async_send_goal(goal_msg);
      while (goal_future.wait_for(std::chrono::milliseconds(50)) !=
             std::future_status::ready) {
        rclcpp::spin_some(node);
        sample_pose();
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
      // 受理の瞬間にリセットする: ここより前の /plan は前のゴールのもの。
      const double accepted_at = node->now().seconds();
      // 受理応答が遅れると、計画は受理より先に届く。その場合は送信時刻を起点に
      // しないと latency が負になる。
      // 計画は受理より先に届くことがある（受理応答が遅い）。その場合は送信を起点に。
      const double plan_baseline =
          (plan_first_at > 0.0 && plan_first_at < accepted_at) ? sent_at : accepted_at;

      auto result_future = client->async_get_result(handle);
      while (result_future.wait_for(std::chrono::milliseconds(50)) !=
             std::future_status::ready) {
        rclcpp::spin_some(node);
        sample_pose();
        if (node->now().seconds() - started > options.goal_timeout) {
          client->async_cancel_goal(handle);
          break;
        }
      }

      try {
        const auto tf =
                tf_buffer.lookupTransform("map", "base_footprint", tf2::TimePointZero);
        trajectory.push_back(Pose2{tf.transform.translation.x, tf.transform.translation.y,
                                   YawFromQuaternion(tf.transform.rotation)});
      } catch (const tf2::TransformException&) {
      }

      // タイムアウトで抜けた場合、結果は永遠に来ない。待ち続けると計測全体が
      // 止まるので、上限付きで待ち、来なければ「測定不能」として次へ進む。
      if (result_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        const auto cpu_after = ros2_autonomous_nav::SampleNavCpu();
        (void)cpu_after;
        runs.emplace_back();
        continue;
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

      run.planned_length_m = first_plan_length;
      run.path_length_ratio =
          ros2_autonomous_nav::PathLengthRatio(run.path_length_m, first_plan_length);
      if (plan_first_at > 0.0) {
        run.planner_latency_ms = (plan_first_at - plan_baseline) * 1000.0;
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
        // 最初のサンプルは「走って行った位置」ではなく置かれた位置。とくに 1 本目の
        // ゴールはスポーン地点なので、そこを走行の余裕として数えない。
        // 計画経路がそもそも寄っているのか、追従で寄るのかを分けて記録する。
        if (first_plan_poses.size() >= 2) {
          run.plan_min_clearance_m = ros2_autonomous_nav::MinFootprintClearance(
              first_plan_poses, occupied, ros2_autonomous_nav::kFootprintHalfLength,
              ros2_autonomous_nav::kFootprintHalfWidth);
        }
        if (trajectory.size() >= 3) {
          const std::vector<Pose2> driving(trajectory.begin() + 1, trajectory.end());
          run.min_clearance_driving_m = ros2_autonomous_nav::MinFootprintClearance(
              driving, occupied, ros2_autonomous_nav::kFootprintHalfLength,
              ros2_autonomous_nav::kFootprintHalfWidth);
        }
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

  // 出力先は存在しないことがある（docs/bench など）。作ってから開き、
  // 開けなければ黙って成功扱いにしない。
  const std::filesystem::path out_path(options.out);
  std::error_code dir_error;
  if (out_path.has_parent_path()) {
    std::filesystem::create_directories(out_path.parent_path(), dir_error);
  }
  std::ofstream out(options.out);
  if (!out) {
    std::fprintf(stderr, "ERROR: cannot write %s\n", options.out.c_str());
    rclcpp::shutdown();
    return 2;
  }
  out << payload.dump(2) << "\n";
  if (!out) {
    std::fprintf(stderr, "ERROR: failed while writing %s\n", options.out.c_str());
    rclcpp::shutdown();
    return 2;
  }
  std::printf("%s\n", payload["summary"].dump(2).c_str());

  rclcpp::shutdown();
  return 0;
}

// SPDX-License-Identifier: MIT
#include "meister_web_nav/nav_worker.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "lifecycle_msgs/srv/get_state.hpp"

namespace meister_web_nav {
namespace {

using NavigateToPose = nav2_msgs::action::NavigateToPose;

constexpr auto kNavigationWait = std::chrono::seconds(2);
constexpr auto kActionServerWait = std::chrono::milliseconds(200);
constexpr auto kResultPoll = std::chrono::milliseconds(100);

}  // namespace

NavWorker::~NavWorker() { Stop(); }

void NavWorker::Start() { thread_ = std::thread([this] { Run(); }); }

void NavWorker::Stop() {
  stop_requested_.store(true);
  wakeup_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
}

nlohmann::json NavWorker::Status() const {
  std::lock_guard<std::mutex> lock(lock_);
  return status_;
}

void NavWorker::Submit(const std::vector<Waypoint>& waypoints) {
  // 前のツアーの取り消し要求をここで解除する。残したままだと、次に送った地点が
  // 到着と同時に取り消される。
  cancel_requested_.store(false);
  {
    std::lock_guard<std::mutex> lock(lock_);
    queue_.push_back(waypoints);
  }
  wakeup_.notify_one();
}

void NavWorker::Cancel() { cancel_requested_.store(true); }

void NavWorker::SetStatus(const nlohmann::json& patch) {
  std::lock_guard<std::mutex> lock(lock_);
  status_.update(patch);
}

bool NavWorker::WaitFor(std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lock(lock_);
  return !wakeup_.wait_for(lock, timeout,
                           [this] { return stop_requested_.load(); });
}

void NavWorker::Run() {
  node_ = std::make_shared<rclcpp::Node>("web_nav_commander");
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");

  SetStatus({{"message", "Nav2の起動を待っています..."}});
  WaitUntilNav2Active();
  if (stop_requested_.load()) {
    return;
  }
  SetStatus({{"state", "idle"}, {"nav2_ready", true}, {"message", "待機中"}});

  while (!stop_requested_.load()) {
    std::vector<Waypoint> tour;
    {
      std::unique_lock<std::mutex> lock(lock_);
      wakeup_.wait(lock, [this] { return stop_requested_.load() || !queue_.empty(); });
      if (stop_requested_.load()) {
        return;
      }
      tour = std::move(queue_.front());
      queue_.pop_front();
    }
    RunTour(tour);
  }
}

void NavWorker::WaitUntilNav2Active() {
  // この構成の map->odom は slam_toolbox が出す。AMCL は居ないので、Python 版の
  // waitUntilNav2Active(localizer='robot_localization') と同じく localizer の起動と
  // 初期姿勢の入力は待たず、bt_navigator が active になるのだけ待つ。
  auto client =
      node_->create_client<lifecycle_msgs::srv::GetState>("bt_navigator/get_state");
  while (rclcpp::ok() && !stop_requested_.load()) {
    if (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(node_->get_logger(), "bt_navigator/get_state を待っています...");
      continue;
    }
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) ==
            rclcpp::FutureReturnCode::SUCCESS &&
        future.get()->current_state.label == "active") {
      RCLCPP_INFO(node_->get_logger(), "Nav2 が起動しました");
      return;
    }
    WaitFor(kNavigationWait);
  }
}

geometry_msgs::msg::PoseStamped NavWorker::ToPoseStamped(
    const Waypoint& waypoint) const {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = node_->get_clock()->now();
  pose.pose.position.x = waypoint.x;
  pose.pose.position.y = waypoint.y;
  pose.pose.orientation.z = std::sin(waypoint.yaw / 2.0);
  pose.pose.orientation.w = std::cos(waypoint.yaw / 2.0);
  return pose;
}

NavWorker::Outcome NavWorker::GoToPose(const Waypoint& waypoint) {
  while (rclcpp::ok() && !stop_requested_.load() && !cancel_requested_.load()) {
    if (nav_client_->action_server_is_ready()) {
      break;
    }
    // 待っている間も取り消しと停止を見る。Python 版の
    // wait_for_server(timeout_sec=1.0) ループと同じ役割。
    nav_client_->wait_for_action_server(kActionServerWait);
  }
  if (!rclcpp::ok() || stop_requested_.load() || cancel_requested_.load()) {
    return Outcome::kCanceled;
  }

  NavigateToPose::Goal goal;
  goal.pose = ToPoseStamped(waypoint);
  auto goal_future = nav_client_->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(node_, goal_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    return Outcome::kFailed;
  }
  goal_handle_ = goal_future.get();
  if (!goal_handle_) {
    return Outcome::kRejected;
  }

  auto result_future = nav_client_->async_get_result(goal_handle_);
  bool cancel_sent = false;
  while (rclcpp::ok() && !stop_requested_.load()) {
    // Python 版の isTaskComplete と同じく 100 ms だけ spin して返す。
    if (rclcpp::spin_until_future_complete(node_, result_future, kResultPoll) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      break;
    }
    if (cancel_requested_.load() && !cancel_sent) {
      auto cancel_future = nav_client_->async_cancel_goal(goal_handle_);
      rclcpp::spin_until_future_complete(node_, cancel_future);
      cancel_sent = true;
    }
  }
  if (!rclcpp::ok() || stop_requested_.load() || cancel_requested_.load()) {
    // 取り消しは結果コードより優先する。CANCELED が返る前に取り消し要求が立つ
    // ことがあり、その場合は「キャンセルしました」と言うのが正しい。
    return Outcome::kCanceled;
  }
  switch (result_future.get().code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return Outcome::kSucceeded;
    case rclcpp_action::ResultCode::CANCELED:
      return Outcome::kCanceled;
    default:
      return Outcome::kFailed;
  }
}

void NavWorker::RunTour(const std::vector<Waypoint>& waypoints) {
  const std::string count = std::to_string(waypoints.size());
  SetStatus({{"state", "sending"},
             {"waypoint_count", static_cast<int>(waypoints.size())},
             {"message", count + " 地点へのルートを送信中..."}});
  // Python 版と同じ理由で followWaypoints は使わない: waypoint_follower ノードが
  // このスタックには無い (RViz が動くのは /navigate_to_pose 直行のため)。地点ごとに
  // 順番に /navigate_to_pose を呼ぶ。
  SetStatus({{"state", "navigating"}, {"message", "移動中..."}});

  Outcome outcome = Outcome::kFailed;
  for (std::size_t idx = 0; idx < waypoints.size(); ++idx) {
    if (cancel_requested_.load() || stop_requested_.load()) {
      break;
    }
    SetStatus({{"current_waypoint", static_cast<int>(idx)},
               {"message", "地点 " + std::to_string(idx + 1) + " へ移動中..."}});
    outcome = GoToPose(waypoints[idx]);
    if (cancel_requested_.load() || outcome != Outcome::kSucceeded) {
      break;
    }
  }

  if (cancel_requested_.load() || stop_requested_.load()) {
    SetStatus({{"state", "canceled"}, {"message", "キャンセルしました"}});
  } else if (outcome == Outcome::kSucceeded) {
    SetStatus({{"state", "succeeded"}, {"message", "全地点に到着しました"}});
  } else {
    SetStatus({{"state", "failed"}, {"message", "ナビゲーションに失敗しました"}});
  }
}

}  // namespace meister_web_nav

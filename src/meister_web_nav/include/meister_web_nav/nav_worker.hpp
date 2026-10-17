// SPDX-License-Identifier: MIT
/// /navigate_to_pose を専用スレッド 1 本で叩く。
///
/// Python 版 (nav2_simple_commander) の blocking 呼び出しは内部で
/// rclpy.spin_until_future_complete を回し、自分のノードを自分で spin する。同じ
/// ノードを複数スレッドから同時に扱うと壊れるので、ナビゲータを触るのはこのスレッド
/// だけに限り、他のスレッドは mutex で守られた status と queue にしか触らない。
/// C++ 版も同じ構造で、rclcpp::spin_until_future_complete を worker スレッドから
/// 呼ぶ（node_ は他の Executor に追加しない）。
#ifndef MEISTER_WEB_NAV_NAV_WORKER_HPP_
#define MEISTER_WEB_NAV_NAV_WORKER_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "meister_web_nav/ports.hpp"

namespace meister_web_nav {

class NavWorker : public NavSink {
 public:
  NavWorker() = default;
  ~NavWorker() override;

  NavWorker(const NavWorker&) = delete;
  NavWorker& operator=(const NavWorker&) = delete;

  void Start();
  /// queue を閉じて worker スレッドを join する。停止要求は待ちループにも伝わる
  /// ので、最長でも action server の 1 回分の timeout で戻る。
  void Stop();

  nlohmann::json Status() const override;
  void Submit(const std::vector<Waypoint>& waypoints) override;
  void Cancel() override;

 private:
  enum class Outcome { kSucceeded, kFailed, kCanceled, kRejected };

  void Run();
  void RunTour(const std::vector<Waypoint>& waypoints);
  void WaitUntilNav2Active();
  Outcome GoToPose(const Waypoint& waypoint);
  geometry_msgs::msg::PoseStamped ToPoseStamped(const Waypoint& waypoint) const;
  void SetStatus(const nlohmann::json& patch);
  /// 停止要求が来たら false を返して早く抜ける。
  bool WaitFor(std::chrono::milliseconds timeout);

  mutable std::mutex lock_;
  std::condition_variable wakeup_;
  std::deque<std::vector<Waypoint>> queue_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> cancel_requested_{false};
  nlohmann::json status_ = nlohmann::json{{"state", "starting"},
                                          {"nav2_ready", false},
                                          {"waypoint_count", 0},
                                          {"current_waypoint", nullptr},
                                          {"message", "ナビゲータを初期化しています..."}};

  // 以下は worker スレッドだけが触る。
  std::thread thread_;
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>
      goal_handle_;
};

}  // namespace meister_web_nav

#endif  // MEISTER_WEB_NAV_NAV_WORKER_HPP_

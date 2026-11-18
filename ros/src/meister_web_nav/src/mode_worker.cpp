// SPDX-License-Identifier: MIT
#include "meister_web_nav/mode_worker.hpp"

#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/srv/save_map.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "meister_web_nav/map_store.hpp"

namespace meister_web_nav {
namespace {

// サービス発見は短く、応答待ちは長く取る。切替は lifecycle 遷移を2回挟むので
// 3 秒では足りない（成功していてもタイムアウトに見える）。
constexpr auto kServiceWait = std::chrono::seconds(3);
constexpr auto kResponseWait = std::chrono::seconds(20);

std::string TimestampNow() {
  const std::time_t now = std::time(nullptr);
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y%m%d-%H%M%S", std::localtime(&now));
  return buffer;
}

}  // namespace

ModeWorker::ModeWorker(const std::string& maps_dir)
    : node_(std::make_shared<rclcpp::Node>("web_nav_mode")) {
  maps_dir_ = maps_dir.empty() ? node_->declare_parameter<std::string>("maps_dir", "maps")
                               : maps_dir;
}

ModeWorker::~ModeWorker() = default;

bool ModeWorker::CallTrigger(const std::string& service, std::string* message) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto client = node_->create_client<std_srvs::srv::Trigger>(service);
  // グラフの更新はスピンで進む。wait_for_service だけで待つと、この Node が
  // 他で回っていないため発見が完了せずタイムアウトする。
  for (int i = 0; i < 40 && !client->service_is_ready(); ++i) {
    rclcpp::spin_some(node_);
    rclcpp::sleep_for(std::chrono::milliseconds(50));
  }
  if (!client->wait_for_service(kServiceWait)) {
    *message = service + " が見つかりません";
    return false;
  }
  auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  if (rclcpp::spin_until_future_complete(node_, future, kResponseWait) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    *message = service + " が応答しません";
    return false;
  }
  // future からは 1 回だけ取り出す。2 回 get() すると
  // "No associated state" で落ちる。
  const auto response = future.get();
  *message = response->message;
  return response->success;
}

std::string ModeWorker::Mode() {
  std::string message;
  if (!CallTrigger("get_mode", &message)) {
    return "unknown";
  }
  return message;
}

std::string ModeWorker::SwitchToLocalization() {
  std::string message;
  if (CallTrigger("switch_to_localization", &message)) {
    return {};
  }
  return message;
}

std::vector<std::string> ModeWorker::ListMaps() { return ListMapNames(maps_dir_); }

std::string ModeWorker::SaveMap(const std::string& name) {
  std::error_code error;
  std::filesystem::create_directories(maps_dir_, error);
  const std::string target = maps_dir_ + "/" + name;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    // map_saver は保存先を parameter でしか受け取らない。設定してから呼ぶ。
    auto param_client =
        std::make_shared<rclcpp::AsyncParametersClient>(node_, "/map_saver");
    if (!param_client->wait_for_service(kServiceWait)) {
      return "map_saver が見つかりません";
    }
    auto param_future = param_client->set_parameters(
        {rclcpp::Parameter("save_map_filename", target)});
    if (rclcpp::spin_until_future_complete(node_, param_future, kResponseWait) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return "map_saver の保存先を設定できません";
    }
    auto client = node_->create_client<nav2_msgs::srv::SaveMap>("/map_saver/save_map");
    for (int i = 0; i < 40 && !client->service_is_ready(); ++i) {
      rclcpp::spin_some(node_);
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    if (!client->wait_for_service(kServiceWait)) {
      return "map_saver/save_map が見つかりません";
    }
    auto future =
        client->async_send_request(std::make_shared<nav2_msgs::srv::SaveMap::Request>());
    if (rclcpp::spin_until_future_complete(node_, future, kResponseWait) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return "map_saver が応答しません";
    }
    if (!future.get()->result) {
      return "地図の保存に失敗しました";
    }
  }
  return {};
}

std::string ModeWorker::DeleteMap(const std::string& name) {
  if (!IsValidMapName(name)) {
    return "invalid map name";
  }
  std::error_code error;
  for (const char* extension : {".yaml", ".pgm"}) {
    const auto path = std::filesystem::path(maps_dir_) / (name + extension);
    std::filesystem::remove(path, error);
  }
  return {};
}

void ModeWorker::SetInitialPose(double x, double y, double yaw) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto publisher = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10);
  geometry_msgs::msg::PoseWithCovarianceStamped message;
  message.header.frame_id = "map";
  message.header.stamp = node_->now();
  message.pose.pose.position.x = x;
  message.pose.pose.position.y = y;
  message.pose.pose.orientation.z = std::sin(yaw / 2.0);
  message.pose.pose.orientation.w = std::cos(yaw / 2.0);
  for (double& element : message.pose.covariance) {
    element = 0.0;
  }
  message.pose.covariance[0] = 0.25;
  message.pose.covariance[7] = 0.25;
  message.pose.covariance[35] = 0.07;
  publisher->publish(message);
  rclcpp::sleep_for(std::chrono::milliseconds(50));  // 配送を待つ
}

}  // namespace meister_web_nav

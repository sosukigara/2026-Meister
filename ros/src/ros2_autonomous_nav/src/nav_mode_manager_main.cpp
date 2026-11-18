// SPDX-License-Identifier: MIT
/// 地図生成と保存地図ナビを切り替える。
///
/// `map→odom` を publish してよいのは常に1ノードだけ（設計書 §6.4）。
/// 地図側と自己位置推定側にそれぞれ nav2_lifecycle_manager を置き、
/// 旧スタックを PAUSE（inactive）してから新スタックを RESUME（active）する。
/// 途中で失敗したら元に戻してから失敗を返す。握り潰さない。
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ros2_autonomous_nav/nav_mode.hpp"

namespace {

using ManageLifecycleNodes = nav2_msgs::srv::ManageLifecycleNodes;
using Trigger = std_srvs::srv::Trigger;

constexpr auto kServiceWait = std::chrono::seconds(3);
constexpr auto kResponseWait = std::chrono::seconds(10);

}  // namespace

class NavModeManager : public rclcpp::Node {
 public:
  NavModeManager() : rclcpp::Node("nav_mode_manager") {
    mapping_manager_ = declare_parameter<std::string>("mapping_manager",
                                                     "/lifecycle_manager_mapping");
    localization_manager_ = declare_parameter<std::string>("localization_manager",
                                                          "/lifecycle_manager_localization");
    initial_mode_ = declare_parameter<std::string>("initial_mode", "mapping");
    mode_ = initial_mode_ == "localization" ? ros2_autonomous_nav::NavMode::kLocalization
                                            : ros2_autonomous_nav::NavMode::kMapping;

    // サービスコールバックの中で応答を待つので、クライアントは別グループに置く。
    client_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    mapping_client_ = create_client<ManageLifecycleNodes>(
        mapping_manager_, rmw_qos_profile_services_default, client_group_);
    localization_client_ = create_client<ManageLifecycleNodes>(
        localization_manager_, rmw_qos_profile_services_default, client_group_);

    mode_publisher_ = create_publisher<std_msgs::msg::String>("nav_mode", 10);
    switch_service_ = create_service<Trigger>(
        "switch_to_localization",
        [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
          SwitchToLocalization(response);
        });
    get_service_ = create_service<Trigger>(
        "get_mode",
        [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
          response->success = true;
          response->message = mode_ == ros2_autonomous_nav::NavMode::kMapping ? "mapping"
                                                                             : "localization";
        });

    PublishMode();
    RCLCPP_INFO(get_logger(), "nav_mode_manager ready (mode=%s, mapping=%s, localization=%s)",
                initial_mode_.c_str(), mapping_manager_.c_str(), localization_manager_.c_str());
  }

 private:
  bool SendCommand(const rclcpp::Client<ManageLifecycleNodes>::SharedPtr& client, uint8_t command) {
    if (!client->wait_for_service(kServiceWait)) {
      RCLCPP_ERROR(get_logger(), "%s が見つかりません", client->get_service_name());
      return false;
    }
    auto request = std::make_shared<ManageLifecycleNodes::Request>();
    request->command = command;
    auto future = client->async_send_request(request);
    if (future.wait_for(kResponseWait) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "%s が応答しません", client->get_service_name());
      return false;
    }
    const bool ok = future.get()->success;
    if (!ok) {
      RCLCPP_ERROR(get_logger(), "%s が command=%u を拒否しました", client->get_service_name(),
                   command);
    }
    return ok;
  }

  void SwitchToLocalization(const std::shared_ptr<Trigger::Response>& response) {
    using ros2_autonomous_nav::MapToOdomOwner;
    using ros2_autonomous_nav::NavMode;

    const auto plan = ros2_autonomous_nav::PlanTransition(mode_, NavMode::kLocalization);
    if (!plan.allowed) {
      response->success = false;
      response->message = plan.reason;
      return;
    }

    RCLCPP_INFO(get_logger(), "地図生成 → 保存地図ナビ へ切り替えます");

    if (!SendCommand(mapping_client_, ManageLifecycleNodes::Request::PAUSE)) {
      response->success = false;
      response->message = "地図生成側を止められませんでした";
      return;
    }
    // 自己位置推定側は起動時 unconfigured なので、まず configure する。
    // 既に configured なら失敗するが、その場合は RESUME だけで足りる。
    if (!SendCommand(localization_client_, ManageLifecycleNodes::Request::CONFIGURE)) {
      RCLCPP_WARN(get_logger(), "localization stack was already configured; resuming");
    }
    if (!SendCommand(localization_client_, ManageLifecycleNodes::Request::RESUME)) {
      // 自己位置推定側が起動しないなら、地図生成側に戻す。
      const bool rolled_back =
          SendCommand(mapping_client_, ManageLifecycleNodes::Request::RESUME);
      response->success = false;
      response->message = rolled_back ? "自己位置推定を起動できず、地図生成に戻しました"
                                      : "自己位置推定を起動できず、地図生成にも戻せません";
      return;
    }

    mode_ = NavMode::kLocalization;
    PublishMode();
    response->success = true;
    response->message = plan.reason;
    (void)MapToOdomOwner::kNone;
  }

  void PublishMode() {
    std_msgs::msg::String message;
    message.data = mode_ == ros2_autonomous_nav::NavMode::kMapping ? "mapping" : "localization";
    mode_publisher_->publish(message);
  }

  std::string mapping_manager_;
  std::string localization_manager_;
  std::string initial_mode_;
  ros2_autonomous_nav::NavMode mode_{ros2_autonomous_nav::NavMode::kMapping};
  rclcpp::CallbackGroup::SharedPtr client_group_;
  rclcpp::Client<ManageLifecycleNodes>::SharedPtr mapping_client_;
  rclcpp::Client<ManageLifecycleNodes>::SharedPtr localization_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_publisher_;
  rclcpp::Service<Trigger>::SharedPtr switch_service_;
  rclcpp::Service<Trigger>::SharedPtr get_service_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavModeManager>();
  // サービスコールバックがクライアント応答を待つ間も他スレッドで処理を進める。
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

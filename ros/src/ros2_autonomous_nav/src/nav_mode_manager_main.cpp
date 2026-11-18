// SPDX-License-Identifier: MIT
/// 地図生成と保存地図ナビを切り替える。
///
/// `map→odom` を publish してよいのは常に1ノードだけ（設計書 §6.4）。
///
/// nav2_lifecycle_manager は bond を張るため、切替のために deactivate すると
/// 「サーバーが落ちた」と誤判定して自壊する。ここでは各ノードの
/// `/change_state` を直接叩き、bond に依存しない。
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ros2_autonomous_nav/nav_mode.hpp"

namespace {

using ChangeState = lifecycle_msgs::srv::ChangeState;
using Trigger = std_srvs::srv::Trigger;
using Transition = lifecycle_msgs::msg::Transition;

constexpr auto kServiceWait = std::chrono::seconds(3);
// lifecycle 遷移は重い（SLAM の初期化など）。余裕を持たせる。
constexpr auto kResponseWait = std::chrono::seconds(40);

}  // namespace

class NavModeManager : public rclcpp::Node {
 public:
  NavModeManager() : rclcpp::Node("nav_mode_manager") {
    slam_node_ = declare_parameter<std::string>("slam_node", "/slam_toolbox");
    map_server_node_ = declare_parameter<std::string>("map_server_node", "/map_server");
    amcl_node_ = declare_parameter<std::string>("amcl_node", "/amcl");
    initial_mode_ = declare_parameter<std::string>("initial_mode", "mapping");
    mode_ = initial_mode_ == "localization" ? ros2_autonomous_nav::NavMode::kLocalization
                                            : ros2_autonomous_nav::NavMode::kMapping;

    // サービスコールバックの中で応答を待つので、クライアントは別グループに置く。
    client_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    slam_client_ = MakeClient(slam_node_);
    map_server_client_ = MakeClient(map_server_node_);
    amcl_client_ = MakeClient(amcl_node_);

    mode_publisher_ = create_publisher<std_msgs::msg::String>("nav_mode", 10);
    switch_service_ = create_service<Trigger>(
        "switch_to_localization",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) { SwitchToLocalization(response); });
    get_service_ = create_service<Trigger>(
        "get_mode",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          response->success = true;
          response->message = mode_ == ros2_autonomous_nav::NavMode::kMapping ? "mapping"
                                                                             : "localization";
        });

    PublishMode();
    // 起動時のモードの持ち主だけを active にする。ノードが spin し始めてから
    // 動かしたいので once-shot のタイマーで行う。
    startup_timer_ = create_wall_timer(std::chrono::seconds(2), [this] {
      startup_timer_->cancel();
      const bool ok = mode_ == ros2_autonomous_nav::NavMode::kMapping
                          ? BringUp(slam_client_)
                          : (BringUp(map_server_client_) && BringUp(amcl_client_));
      if (!ok) {
        RCLCPP_ERROR(get_logger(), "起動時のモードを確立できません");
      }
    });
    RCLCPP_INFO(get_logger(), "nav_mode_manager ready (mode=%s)", initial_mode_.c_str());
  }

 private:
  rclcpp::Client<ChangeState>::SharedPtr MakeClient(const std::string& node) {
    return create_client<ChangeState>(node + "/change_state",
                                      rmw_qos_profile_services_default, client_group_);
  }

  bool ApplyTransition(const rclcpp::Client<ChangeState>::SharedPtr& client, uint8_t transition) {
    for (int i = 0; i < 40 && !client->service_is_ready(); ++i) {
      rclcpp::spin_some(shared_from_this());
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    if (!client->wait_for_service(kServiceWait)) {
      RCLCPP_ERROR(get_logger(), "%s が見つかりません", client->get_service_name());
      return false;
    }
    auto request = std::make_shared<ChangeState::Request>();
    request->transition.id = transition;
    auto future = client->async_send_request(request);
    if (future.wait_for(kResponseWait) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "%s が応答しません", client->get_service_name());
      return false;
    }
    const auto response = future.get();
    if (!response->success) {
      RCLCPP_ERROR(get_logger(), "%s が transition=%u を拒否しました",
                   client->get_service_name(), transition);
    }
    return response->success;
  }

  bool BringUp(const rclcpp::Client<ChangeState>::SharedPtr& client) {
    // 既に active なら configure は失敗する。その場合は activate だけで足りる。
    ApplyTransition(client, Transition::TRANSITION_CONFIGURE);
    return ApplyTransition(client, Transition::TRANSITION_ACTIVATE);
  }

  void SwitchToLocalization(const std::shared_ptr<Trigger::Response>& response) {
    const auto plan = ros2_autonomous_nav::PlanTransition(
        mode_, ros2_autonomous_nav::NavMode::kLocalization);
    if (!plan.allowed) {
      response->success = false;
      response->message = plan.reason;
      return;
    }

    RCLCPP_INFO(get_logger(), "地図生成 → 保存地図ナビ へ切り替えます");
    if (!ApplyTransition(slam_client_, Transition::TRANSITION_DEACTIVATE)) {
      response->success = false;
      response->message = "地図生成側を止められませんでした";
      return;
    }
    if (!BringUp(map_server_client_) || !BringUp(amcl_client_)) {
      // 自己位置推定側が立ち上がらないなら、地図生成側に戻す。
      ApplyTransition(map_server_client_, Transition::TRANSITION_DEACTIVATE);
      ApplyTransition(amcl_client_, Transition::TRANSITION_DEACTIVATE);
      const bool rolled_back = ApplyTransition(slam_client_, Transition::TRANSITION_ACTIVATE);
      response->success = false;
      response->message = rolled_back ? "自己位置推定を起動できず、地図生成に戻しました"
                                      : "自己位置推定を起動できず、地図生成にも戻せません";
      return;
    }

    mode_ = ros2_autonomous_nav::NavMode::kLocalization;
    PublishMode();
    response->success = true;
    response->message = plan.reason;
  }

  void PublishMode() {
    std_msgs::msg::String message;
    message.data = mode_ == ros2_autonomous_nav::NavMode::kMapping ? "mapping" : "localization";
    mode_publisher_->publish(message);
  }

  std::string slam_node_;
  std::string map_server_node_;
  std::string amcl_node_;
  std::string initial_mode_;
  ros2_autonomous_nav::NavMode mode_{ros2_autonomous_nav::NavMode::kMapping};
  rclcpp::CallbackGroup::SharedPtr client_group_;
  rclcpp::Client<ChangeState>::SharedPtr slam_client_;
  rclcpp::Client<ChangeState>::SharedPtr map_server_client_;
  rclcpp::Client<ChangeState>::SharedPtr amcl_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_publisher_;
  rclcpp::Service<Trigger>::SharedPtr switch_service_;
  rclcpp::Service<Trigger>::SharedPtr get_service_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
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

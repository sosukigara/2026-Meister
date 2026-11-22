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

namespace {

using ChangeState = lifecycle_msgs::srv::ChangeState;
using Trigger = std_srvs::srv::Trigger;
using Transition = lifecycle_msgs::msg::Transition;

/// map→odom の持ち主を決めるモード。
enum class NavMode { kMapping, kLocalization };

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
    mode_ = initial_mode_ == "localization" ? NavMode::kLocalization
                                            : NavMode::kMapping;

    // サービスコールバックの中で応答を待つので、クライアントは別グループに置く。
    client_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    slam_client_ = MakeClient(slam_node_);
    map_server_client_ = MakeClient(map_server_node_);
    amcl_client_ = MakeClient(amcl_node_);

    mode_publisher_ = create_publisher<std_msgs::msg::String>("nav_mode", 10);
    switch_service_ = create_service<Trigger>(
        "switch_to_localization",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          SwitchMode(NavMode::kLocalization, response);
        });
    // 逆方向。slam_toolbox を再 activate してもポーズグラフは保持されることを
    // 実測済み (332x302 / 55233 cells が前後で一致) なので、地図を捨てずに戻せる。
    mapping_service_ = create_service<Trigger>(
        "switch_to_mapping", [this](const std::shared_ptr<Trigger::Request>,
                                    std::shared_ptr<Trigger::Response> response) {
          SwitchMode(NavMode::kMapping, response);
        });
    get_service_ = create_service<Trigger>(
        "get_mode",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          response->success = true;
          response->message =
              mode_ == NavMode::kMapping ? "mapping" : "localization";
        });

    PublishMode();
    // 起動時のモードの持ち主だけを active にする。ノードが spin し始めてから
    // 動かしたいので once-shot のタイマーで行う。
    startup_timer_ = create_wall_timer(std::chrono::seconds(2), [this] {
      startup_timer_->cancel();
      const bool ok = mode_ == NavMode::kMapping
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
                                      rclcpp::ServicesQoS(), client_group_);
  }

  bool ApplyTransition(const rclcpp::Client<ChangeState>::SharedPtr& client, uint8_t transition) {
    // この Node は MultiThreadedExecutor が回している。探索を進めようとして自分で
    // spin_some(shared_from_this()) を呼ぶと、rclcpp は「既に executor に載っている」
    // として std::runtime_error を投げ、モード管理ごと落ちる（起動直後に
    // slam_toolbox がまだ configure されていないと必ずここへ入る。実測で再現）。
    // 探索は他のスレッドが進めるので、ここは待つだけでよい。
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

  /// target へ切り替える。旧 owner の deactivate が終わってから新 owner を
  /// activate する（map→odom の publisher を 2 つにしない）。
  void SwitchMode(NavMode target,
                  const std::shared_ptr<Trigger::Response>& response) {
    if (mode_ == target) {
      response->success = false;
      response->message = "既にそのモードです";
      return;
    }

    const bool to_localization = target == NavMode::kLocalization;
    // 保存地図 → 地図生成 は slam_toolbox の再 activate で戻す。ポーズグラフが
    // 保持されることは実測で確認済み (前後で 332x302 / 55233 cells が一致)。
    const char* const reason = to_localization
                                   ? "地図生成から保存地図ナビへ切り替えます"
                                   : "保存地図ナビから地図生成へ切り替えます（地図は引き継ぎます）";
    RCLCPP_INFO(get_logger(), "%s", reason);
    const char* const old_side = to_localization ? "地図生成側" : "自己位置推定側";
    const char* const new_side = to_localization ? "自己位置推定" : "地図生成";

    if (!DeactivateCurrent(to_localization)) {
      response->success = false;
      response->message = std::string(old_side) + "を止められませんでした";
      return;
    }
    if (!ActivateTarget(to_localization)) {
      DeactivateTarget(to_localization);
      const bool rolled_back = ActivateCurrent(to_localization);
      response->success = false;
      response->message = std::string(new_side) + "を起動できず、" +
                          (rolled_back ? std::string(old_side) + "に戻しました"
                                       : std::string(old_side) + "にも戻せません");
      return;
    }

    mode_ = target;
    PublishMode();
    response->success = true;
    response->message = reason;
  }

  /// これから離れる側を止める。
  bool DeactivateCurrent(bool to_localization) {
    if (to_localization) {
      return ApplyTransition(slam_client_, Transition::TRANSITION_DEACTIVATE);
    }
    // map_server と amcl は両方止める。片方だけ残ると map→odom が二重になる。
    const bool map_ok = ApplyTransition(map_server_client_, Transition::TRANSITION_DEACTIVATE);
    const bool amcl_ok = ApplyTransition(amcl_client_, Transition::TRANSITION_DEACTIVATE);
    return map_ok && amcl_ok;
  }

  bool ActivateTarget(bool to_localization) {
    if (to_localization) {
      return BringUp(map_server_client_) && BringUp(amcl_client_);
    }
    return BringUp(slam_client_);
  }

  void DeactivateTarget(bool to_localization) {
    if (to_localization) {
      ApplyTransition(map_server_client_, Transition::TRANSITION_DEACTIVATE);
      ApplyTransition(amcl_client_, Transition::TRANSITION_DEACTIVATE);
    } else {
      ApplyTransition(slam_client_, Transition::TRANSITION_DEACTIVATE);
    }
  }

  /// 失敗時に元のモードへ戻す。
  bool ActivateCurrent(bool to_localization) {
    if (to_localization) {
      return ApplyTransition(slam_client_, Transition::TRANSITION_ACTIVATE);
    }
    return BringUp(map_server_client_) && BringUp(amcl_client_);
  }

  void PublishMode() {
    std_msgs::msg::String message;
    message.data = mode_ == NavMode::kMapping ? "mapping" : "localization";
    mode_publisher_->publish(message);
  }

  std::string slam_node_;
  std::string map_server_node_;
  std::string amcl_node_;
  std::string initial_mode_;
  NavMode mode_{NavMode::kMapping};
  rclcpp::CallbackGroup::SharedPtr client_group_;
  rclcpp::Client<ChangeState>::SharedPtr slam_client_;
  rclcpp::Client<ChangeState>::SharedPtr map_server_client_;
  rclcpp::Client<ChangeState>::SharedPtr amcl_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_publisher_;
  rclcpp::Service<Trigger>::SharedPtr switch_service_;
  rclcpp::Service<Trigger>::SharedPtr mapping_service_;
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

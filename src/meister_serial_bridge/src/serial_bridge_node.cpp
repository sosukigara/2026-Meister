#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "meister_serial_bridge/bridge_core.hpp"
#include "meister_serial_bridge/comm_check.hpp"
#include "meister_serial_bridge/frames.hpp"

namespace meister_serial_bridge {
namespace {

double NowSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

/// 受信 1 回の待ち。"0 なら待たずに「貯まっている分」を取る。
constexpr int kRxTimeoutMs = 0;

class SerialBridgeNode final : public rclcpp::Node {
 public:
  SerialBridgeNode() : Node("serial_bridge") {
    BridgeParams params;
    params.serial_port = declare_parameter<std::string>("serial_port", kDefaultPort);
    params.baud = declare_parameter<int>("baud", kDefaultBaud);
    params.kinematics.wheelbase_m = declare_parameter<double>("wheelbase", 0.30);
    params.kinematics.max_linear_vel_mps = declare_parameter<double>("max_linear_vel", 1.0);
    params.kinematics.max_steering_deg =
        declare_parameter<double>("max_steering_deg", 30.0);
    params.kinematics.steer_invert = declare_parameter<bool>("steer_invert", false);
    params.kinematics.drive_invert = declare_parameter<bool>("drive_invert", false);
    params.cmd_timeout_s = declare_parameter<double>("cmd_timeout", 0.5);

    core_ = std::make_unique<BridgeCore>(
        params,
        [](const std::string& path, int baud) -> std::shared_ptr<SerialIo> {
          try {
            return std::shared_ptr<SerialIo>(OpenSerialPort(path, baud));
          } catch (const std::exception& e) {
            // ポート未接続でも起動は継続する。RX スレッドが後で再試行する。
            RCLCPP_ERROR(rclcpp::get_logger("serial_bridge"), "serial open failed: %s",
                         std::string(e.what()).c_str());
            return nullptr;
          }
        },
        [] { return NowSeconds(); });

    // /cmd_vel と同じ cmd_ 規約に揃える。腕とグリッパーは独立購読にする:
    // 1 本の UART を共有しているので、片方の不通がもう片方を止めない。
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist& msg) {
          core_->OnCmdVel(msg.linear.x, msg.angular.z);
        });
    arm_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
        "/cmd_arm_joint", 10,
        [this](const std_msgs::msg::Int16MultiArray& msg) { OnArmJoint(msg); });
    gripper_sub_ = create_subscription<std_msgs::msg::UInt8>(
        "/cmd_gripper", 10,
        [this](const std_msgs::msg::UInt8& msg) { core_->OnGripper(msg.data); });
    state_pub_ = create_publisher<std_msgs::msg::Int16MultiArray>("/esp32/state", 10);
    watchdog_ = create_wall_timer(std::chrono::milliseconds(100), [this] { OnWatchdog(); });

    rx_stop_ = false;
    rx_thread_ = std::thread([this] { RxLoop(); });

    RCLCPP_INFO(get_logger(),
                "serial bridge: port=%s baud=%d wheelbase=%f max_vel=%f max_steer=%fdeg",
                params.serial_port.c_str(), params.baud, params.kinematics.wheelbase_m,
                params.kinematics.max_linear_vel_mps,
                params.kinematics.max_steering_deg);
  }

  ~SerialBridgeNode() override {
    if (rx_thread_.joinable()) {
      rx_stop_ = true;
      rx_thread_.join();
    }
  }

  /// 速度 0 を送る。rclcpp::init は SIGINT と SIGTERM の両方を捕捉するので、
  /// どちらで終了しても main() のこの行を通る。
  void SendStopFrames() { core_->SendStop(); }

 private:
  void OnArmJoint(const std_msgs::msg::Int16MultiArray& msg) {
    // 軸数が違うメッセージでコールバックを落とすとノードごと死ぬので、
    // 無視して警告だけ出して核心には到達させない。
    const std::vector<int16_t> angles(msg.data.begin(), msg.data.end());
    if (!core_->OnArmJoint(angles)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "cmd_arm_joint: expected %zu values, got %zu; ignored",
                           kArmChannels, angles.size());
    }
  }

  void OnWatchdog() {
    const WatchdogVerdict verdict = core_->OnWatchdog(NowSeconds());
    if (verdict.arm_idle_started) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "arm command timeout -> holding last target");
    }
    if (verdict.send_stop) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cmd_vel timeout -> stop");
    }
  }

  void RxLoop() {
    std::vector<Feedback> states;
    std::vector<uint8_t> fb_errors;
    std::string error;

    while (!rx_stop_) {
      states.clear();
      fb_errors.clear();
      error.clear();
      if (!core_->CurrentLink(&error)) {
        // 再試行を待っている間は error が空なので、ログも出さない。
        if (!error.empty()) {
          RCLCPP_ERROR(get_logger(), "serial open failed: %s", error.c_str());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      if (!core_->RxOnce(&states, &fb_errors, kRxTimeoutMs)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      for (const auto& fb : states) {
        PublishState(fb);
      }
      for (uint8_t code : fb_errors) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "ESP32 error frame: code=%u", static_cast<unsigned>(code));
      }
    }
  }

  void PublishState(const Feedback& fb) {
    std_msgs::msg::Int16MultiArray msg;
    msg.data.reserve(fb.encoders.size() + 2);
    for (int16_t e : fb.encoders) {
      msg.data.push_back(e);
    }
    msg.data.push_back(static_cast<int16_t>(fb.state));
    msg.data.push_back(static_cast<int16_t>(fb.error_flags));
    state_pub_->publish(msg);
  }

  std::unique_ptr<BridgeCore> core_;
  std::atomic<bool> rx_stop_{false};
  std::thread rx_thread_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr arm_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr gripper_sub_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

}  // namespace
}  // namespace meister_serial_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<meister_serial_bridge::SerialBridgeNode>();
  rclcpp::spin(node);
  node->SendStopFrames();
  rclcpp::shutdown();
  return 0;
}

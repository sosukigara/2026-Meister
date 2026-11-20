#include <algorithm>
#include <atomic>
#include <chrono>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "umeonigiri_serial_bridge/ackermann_imu_odometry.hpp"
#include "umeonigiri_serial_bridge/bridge_core.hpp"
#include "umeonigiri_serial_bridge/comm_check.hpp"
#include "umeonigiri_serial_bridge/feedback_publisher.hpp"
#include "umeonigiri_serial_bridge/frames.hpp"
#include "umeonigiri_serial_bridge/odometry_publisher.hpp"

namespace umeonigiri_serial_bridge {

/// C++ 側の唯一の定義。launch は言語をまたぐため同じ文字列を別に持ち、
/// serial_bridge.launch.py と real_robot.launch.py の name= も揃える。
constexpr char kNodeName[] = "umeonigiri_serial_bridge";

namespace {

double NowSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

/// 受信 1 回の待ち [ms]。
/// 旧実装は kRxTimeoutMs = 0 だったため、UART 空き時間（99%以上）に RxLoop が
/// スリープ無しで無限スピンし、Orange Pi 5 の CPU 1 コアを常時 100% 占有していた。
/// 20 ms に設定することで、データ未着時は OS 待機に入り（アイドル CPU < 0.1%）、
/// 1 バイト到着した瞬間に即座に起床して溜まっている全バイトを一括取得する。
constexpr int kRxTimeoutMs = 20;

class SerialBridgeNode final : public rclcpp::Node {
 public:
  SerialBridgeNode() : Node(kNodeName) {
    BridgeParams params;
    params.serial_port = declare_parameter<std::string>("serial_port", kDefaultPort);
    params.baud = declare_parameter<int>("baud", kDefaultBaud);
    params.kinematics.wheelbase_m = declare_parameter<double>("wheelbase", 0.30);
    // vx [m/s] -> パーミル指令の正規化値。実機の実測最高速と一致していないと
    // 指令速度と実速度がずれ、AckermannImuOdometry もその分ドリフトする。
    // 5 km/h (1.39 m/s) を狙うなら実測後に launch から上書きする。
    // ★要確認★ 1.0 は実測値ではなく従来既定のまま（R4）。
    params.kinematics.max_linear_vel_mps = declare_parameter<double>("max_linear_vel", 1.0);
    params.kinematics.max_steering_deg =
        declare_parameter<double>("max_steering_deg", 30.0);
    params.kinematics.steer_invert = declare_parameter<bool>("steer_invert", false);
    params.kinematics.drive_invert = declare_parameter<bool>("drive_invert", false);
    params.cmd_timeout_s = declare_parameter<double>("cmd_timeout", 0.5);

    // 0 以下だと TwistToActuators が購読コールバック内で例外を投げ、
    // rclcpp::spin ごと落ちる。起動時に落として原因を 1 行で返す。
    if (params.kinematics.max_linear_vel_mps <= 0.0) {
      throw std::invalid_argument("max_linear_vel must be positive");
    }
    if (params.kinematics.max_steering_deg <= 0.0) {
      throw std::invalid_argument("max_steering_deg must be positive");
    }
    if (params.cmd_timeout_s <= 0.0) {
      throw std::invalid_argument("cmd_timeout must be positive");
    }
    if (params.baud <= 0) {
      throw std::invalid_argument("baud must be positive");
    }

    publish_odom_ = declare_parameter<bool>("publish_odom", true);
    publish_odom_tf_ = declare_parameter<bool>("publish_odom_tf", true);
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_footprint");
    const double odom_rate_hz = std::max(1.0, declare_parameter<double>("odom_rate_hz", 50.0));

    OdometryConfig odom_cfg{};
    odom_cfg.wheelbase_m = params.kinematics.wheelbase_m;
    odom_cfg.max_steering_deg = params.kinematics.max_steering_deg;
    odom_cfg.cmd_timeout_s = params.cmd_timeout_s;
    odom_cfg.imu_yaw_fusion_gain = declare_parameter<double>("imu_yaw_fusion_gain", 0.85);
    odometry_ = std::make_unique<OdometryPublisher>(
        *this, odom_cfg, publish_odom_, publish_odom_tf_, odom_frame_id_, base_frame_id_);
    feedback_publisher_ = std::make_unique<FeedbackPublisher>(*this);

    core_ = std::make_unique<BridgeCore>(
        params,
        [](const std::string& path, int baud) -> std::shared_ptr<SerialIo> {
          try {
            return std::shared_ptr<SerialIo>(OpenSerialPort(path, baud));
          } catch (const std::exception& e) {
            // ポート未接続でも起動は継続する。RX スレッドが後で再試行する。
            RCLCPP_ERROR(rclcpp::get_logger(kNodeName), "serial open failed: %s",
                         std::string(e.what()).c_str());
            return nullptr;
          }
        },
        [] { return NowSeconds(); });

    // /cmd_vel と同じ cmd_ 規約に揃える。腕とグリッパーは独立購読にする:
    // 1 本の UART を共有しているので、片方の不通がもう片方を止めない。
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist& msg) {
          odometry_->UpdateTwistCommand(msg.linear.x, msg.angular.z, NowSeconds());
          const std::string error =
              RunSerialOp(*core_, [&] { core_->OnCmdVel(msg.linear.x, msg.angular.z); });
          if (!error.empty()) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                  "serial bridge I/O error: %s", error.c_str());
          }
        });
    arm_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
        "/cmd_arm_joint", 10,
        [this](const std_msgs::msg::Int16MultiArray& msg) { OnArmJoint(msg); });
    gripper_sub_ = create_subscription<std_msgs::msg::UInt8>(
        "/cmd_gripper", 10, [this](const std_msgs::msg::UInt8& msg) {
          const std::string error = RunSerialOp(*core_, [&] { core_->OnGripper(msg.data); });
          if (!error.empty()) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                  "serial bridge I/O error: %s", error.c_str());
          }
        });

    watchdog_ = create_wall_timer(std::chrono::milliseconds(100), [this] { OnWatchdog(); });

    if (publish_odom_ || publish_odom_tf_) {
      const auto period = std::chrono::duration<double>(1.0 / odom_rate_hz);
      odom_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(period),
          [this] { odometry_->StepAndPublish(NowSeconds()); });
    }

    rx_stop_ = false;
    rx_thread_ = std::thread([this] { RxLoop(); });

    RCLCPP_INFO(get_logger(),
                "serial bridge: port=%s baud=%d wheelbase=%.2f max_vel=%.2f max_steer=%.1fdeg odom=%.0fHz",
                params.serial_port.c_str(), params.baud, params.kinematics.wheelbase_m,
                params.kinematics.max_linear_vel_mps,
                params.kinematics.max_steering_deg, odom_rate_hz);
  }

  ~SerialBridgeNode() override {
    if (rx_thread_.joinable()) {
      rx_stop_ = true;
      rx_thread_.join();
    }
  }

  /// 速度 0 を送る。rclcpp::init は SIGINT と SIGTERM の両方を捕捉するので、
  /// どちらで終了しても main() のこの行を通る。
  void SendStopFrames() {
    const std::string error = RunSerialOp(*core_, [&] { core_->SendStop(); });
    if (!error.empty()) {
      RCLCPP_ERROR(get_logger(), "failed to send stop frames: %s", error.c_str());
    }
  }

 private:
  void OnArmJoint(const std_msgs::msg::Int16MultiArray& msg) {
    // 軸数が違うメッセージでコールバックを落とすとノードごと死ぬので、
    // 無視して警告だけ出して核心には到達させない。
    const std::vector<int16_t> angles(msg.data.begin(), msg.data.end());
    bool accepted = false;
    const std::string error =
        RunSerialOp(*core_, [&] { accepted = core_->OnArmJoint(angles); });
    if (!error.empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "serial bridge I/O error: %s", error.c_str());
      return;
    }
    if (!accepted) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "cmd_arm_joint: expected %zu values, got %zu; ignored",
                           kArmChannels, angles.size());
    }
  }

  void OnWatchdog() {
    WatchdogVerdict verdict;
    const std::string error =
        RunSerialOp(*core_, [&] { verdict = core_->OnWatchdog(NowSeconds()); });
    if (!error.empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "serial bridge I/O error: %s", error.c_str());
      return;
    }
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
    std::vector<ImuReport> imus;
    std::string error;

    while (!rx_stop_) {
      states.clear();
      fb_errors.clear();
      imus.clear();
      error.clear();
      if (!core_->CurrentLink(&error)) {
        // 再試行を待っている間は error が空なので、ログも出さない。
        if (!error.empty()) {
          RCLCPP_ERROR(get_logger(), "serial open failed: %s", error.c_str());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      bool rx_ok = false;
      const std::string rx_error = RunSerialOp(
          *core_, [&] { rx_ok = core_->RxOnce(&states, &fb_errors, kRxTimeoutMs, &imus); });
      if (!rx_error.empty()) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                              "serial bridge I/O error: %s", rx_error.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      if (!rx_ok) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      for (const auto& fb : states) {
        feedback_publisher_->PublishState(fb);
      }
      for (const auto& imu : imus) {
        odometry_->UpdateImuReport(imu, NowSeconds());
        feedback_publisher_->PublishImu(imu);
      }
      for (uint8_t code : fb_errors) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "ESP32 error frame: code=%u", static_cast<unsigned>(code));
      }
    }
  }

  std::unique_ptr<BridgeCore> core_;
  std::unique_ptr<OdometryPublisher> odometry_;
  std::unique_ptr<FeedbackPublisher> feedback_publisher_;
  bool publish_odom_{true};
  bool publish_odom_tf_{true};
  std::string odom_frame_id_{"odom"};
  std::string base_frame_id_{"base_footprint"};

  std::atomic<bool> rx_stop_{false};
  std::thread rx_thread_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr arm_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr gripper_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
};

}  // namespace
}  // namespace umeonigiri_serial_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<umeonigiri_serial_bridge::SerialBridgeNode> node;
  try {
    node = std::make_shared<umeonigiri_serial_bridge::SerialBridgeNode>();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger(umeonigiri_serial_bridge::kNodeName), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  node->SendStopFrames();
  rclcpp::shutdown();
  return 0;
}

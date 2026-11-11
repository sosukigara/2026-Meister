#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "meister_serial_bridge/ackermann_imu_odometry.hpp"
#include "meister_serial_bridge/bridge_core.hpp"
#include "meister_serial_bridge/comm_check.hpp"
#include "meister_serial_bridge/frames.hpp"

namespace meister_serial_bridge {
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
  SerialBridgeNode() : Node("umeonigiri_serial_bridge") {
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
    odometry_ = std::make_unique<AckermannImuOdometry>(odom_cfg);

    core_ = std::make_unique<BridgeCore>(
        params,
        [](const std::string& path, int baud) -> std::shared_ptr<SerialIo> {
          try {
            return std::shared_ptr<SerialIo>(OpenSerialPort(path, baud));
          } catch (const std::exception& e) {
            // ポート未接続でも起動は継続する。RX スレッドが後で再試行する。
            RCLCPP_ERROR(rclcpp::get_logger("umeonigiri_serial_bridge"), "serial open failed: %s",
                         std::string(e.what()).c_str());
            return nullptr;
          }
        },
        [] { return NowSeconds(); });

    // /cmd_vel と同じ cmd_ 規約に揃える。腕とグリッパーは独立購読にする:
    // 1 本の UART を共有しているので、片方の不通がもう片方を止めない。
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist& msg) {
          {
            std::lock_guard<std::mutex> lk(odom_mutex_);
            odometry_->UpdateTwistCommand(msg.linear.x, msg.angular.z, NowSeconds());
          }
          core_->OnCmdVel(msg.linear.x, msg.angular.z);
        });
    arm_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
        "/cmd_arm_joint", 10,
        [this](const std_msgs::msg::Int16MultiArray& msg) { OnArmJoint(msg); });
    gripper_sub_ = create_subscription<std_msgs::msg::UInt8>(
        "/cmd_gripper", 10,
        [this](const std_msgs::msg::UInt8& msg) { core_->OnGripper(msg.data); });

    state_pub_ = create_publisher<std_msgs::msg::Int16MultiArray>("/esp32/state", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    if (publish_odom_) {
      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 20);
    }
    if (publish_odom_tf_) {
      tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", 20);
    }

    watchdog_ = create_wall_timer(std::chrono::milliseconds(100), [this] { OnWatchdog(); });

    if (publish_odom_ || publish_odom_tf_) {
      const auto period = std::chrono::duration<double>(1.0 / odom_rate_hz);
      odom_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(period),
          [this] { StepAndPublishOdometry(); });
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

  void StepAndPublishOdometry() {
    const rclcpp::Time stamp = now();
    OdometryState st{};
    OdometryConfig cfg{};
    {
      std::lock_guard<std::mutex> lk(odom_mutex_);
      st = odometry_->Step(NowSeconds());
      cfg = odometry_->config();
    }

    if (publish_odom_tf_ && tf_pub_) {
      auto tf_msg = std::make_unique<tf2_msgs::msg::TFMessage>();
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = stamp;
      t.header.frame_id = odom_frame_id_;
      t.child_frame_id = base_frame_id_;
      t.transform.translation.x = st.x;
      t.transform.translation.y = st.y;
      t.transform.translation.z = 0.0;
      t.transform.rotation.x = 0.0;
      t.transform.rotation.y = 0.0;
      t.transform.rotation.z = st.qz;
      t.transform.rotation.w = st.qw;
      tf_msg->transforms.push_back(std::move(t));
      tf_pub_->publish(std::move(tf_msg));
    }

    if (publish_odom_ && odom_pub_) {
      auto odom = std::make_unique<nav_msgs::msg::Odometry>();
      odom->header.stamp = stamp;
      odom->header.frame_id = odom_frame_id_;
      odom->child_frame_id = base_frame_id_;
      odom->pose.pose.position.x = st.x;
      odom->pose.pose.position.y = st.y;
      odom->pose.pose.position.z = 0.0;
      odom->pose.pose.orientation.x = 0.0;
      odom->pose.pose.orientation.y = 0.0;
      odom->pose.pose.orientation.z = st.qz;
      odom->pose.pose.orientation.w = st.qw;
      odom->pose.covariance[0]  = cfg.pose_cov_xy;
      odom->pose.covariance[7]  = cfg.pose_cov_xy;
      odom->pose.covariance[14] = 1e-6;
      odom->pose.covariance[21] = 1e-6;
      odom->pose.covariance[28] = 1e-6;
      odom->pose.covariance[35] = cfg.pose_cov_yaw;

      odom->twist.twist.linear.x = st.vx;
      odom->twist.twist.linear.y = 0.0;
      odom->twist.twist.angular.z = st.wz;
      odom->twist.covariance[0]  = cfg.twist_cov_vx;
      odom->twist.covariance[7]  = 1e-6;
      odom->twist.covariance[14] = 1e-6;
      odom->twist.covariance[21] = 1e-6;
      odom->twist.covariance[28] = 1e-6;
      odom->twist.covariance[35] = cfg.twist_cov_wz;
      odom_pub_->publish(std::move(odom));
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
      if (!core_->RxOnce(&states, &fb_errors, kRxTimeoutMs, &imus)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      for (const auto& fb : states) {
        PublishState(fb);
      }
      for (const auto& imu : imus) {
        {
          std::lock_guard<std::mutex> lk(odom_mutex_);
          odometry_->UpdateImuReport(imu, NowSeconds());
        }
        PublishImu(imu);
      }
      for (uint8_t code : fb_errors) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "ESP32 error frame: code=%u", static_cast<unsigned>(code));
      }
    }
  }

  void PublishImu(const ImuReport& imu) {
    auto msg = std::make_unique<sensor_msgs::msg::Imu>();
    msg->header.stamp = now();
    msg->header.frame_id = "imu_link";
    const YawQuat q = YawTenthsToQuat(imu.yaw_tenths);
    msg->orientation.x = 0.0;
    msg->orientation.y = 0.0;
    msg->orientation.z = q.z;
    msg->orientation.w = q.w;
    // BNO055 yaw orientation covariance (non-zero diagonal required by SLAM/EKF)
    msg->orientation_covariance = {
        0.01, 0.0, 0.0,
        0.0, 0.01, 0.0,
        0.0, 0.0, 0.0015};
    // yaw 以外は送っていないので、角速度・加速度の先頭を -1（未知）で示す。
    msg->angular_velocity_covariance[0] = -1.0;
    msg->linear_acceleration_covariance[0] = -1.0;
    imu_pub_->publish(std::move(msg));
  }

  void PublishState(const Feedback& fb) {
    auto msg = std::make_unique<std_msgs::msg::Int16MultiArray>();
    msg->data.reserve(fb.encoders.size() + 2);
    for (int16_t e : fb.encoders) {
      msg->data.push_back(e);
    }
    msg->data.push_back(static_cast<int16_t>(fb.state));
    msg->data.push_back(static_cast<int16_t>(fb.error_flags));
    state_pub_->publish(std::move(msg));
  }

  std::unique_ptr<BridgeCore> core_;
  std::unique_ptr<AckermannImuOdometry> odometry_;
  std::mutex odom_mutex_;
  bool publish_odom_{true};
  bool publish_odom_tf_{true};
  std::string odom_frame_id_{"odom"};
  std::string base_frame_id_{"base_footprint"};

  std::atomic<bool> rx_stop_{false};
  std::thread rx_thread_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr arm_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr gripper_sub_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
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

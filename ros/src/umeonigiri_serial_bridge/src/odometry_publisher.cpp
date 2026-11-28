#include "umeonigiri_serial_bridge/odometry_publisher.hpp"

#include <memory>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace umeonigiri_serial_bridge {

OdometryPublisher::OdometryPublisher(rclcpp::Node& node, const OdometryConfig& config,
                                     bool publish_odom, bool publish_tf,
                                     std::string odom_frame_id,
                                     std::string base_frame_id)
    : node_(node),
      odometry_(config),
      publish_odom_(publish_odom),
      publish_tf_(publish_tf),
      odom_frame_id_(std::move(odom_frame_id)),
      base_frame_id_(std::move(base_frame_id)) {
  if (publish_odom_) {
    odom_pub_ = node_.create_publisher<nav_msgs::msg::Odometry>("/odom", 20);
  }
  if (publish_tf_) {
    tf_pub_ = node_.create_publisher<tf2_msgs::msg::TFMessage>("/tf", 20);
  }
}

void OdometryPublisher::UpdateTwistCommand(double vx, double wz, double now_s) {
  std::lock_guard<std::mutex> lock(mutex_);
  odometry_.UpdateTwistCommand(vx, wz, now_s);
}

void OdometryPublisher::UpdateWheelFeedback(const Feedback& fb, double now_s) {
  std::lock_guard<std::mutex> lock(mutex_);
  odometry_.UpdateWheelFeedback(fb, now_s);
}

void OdometryPublisher::UpdateImuReport(const ImuReport& imu, double now_s) {
  std::lock_guard<std::mutex> lock(mutex_);
  odometry_.UpdateImuReport(imu, now_s);
}

void OdometryPublisher::StepAndPublish(double now_s) {
  const rclcpp::Time stamp = node_.now();
  OdometryState st{};
  OdometryConfig cfg{};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    st = odometry_.Step(now_s);
    cfg = odometry_.config();
  }

  if (publish_tf_ && tf_pub_) {
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
    // エンコーダ未接続（オープンループ指令推定）時は共分散を広めに出し、
    // LiDAR 側（SLAM/AMCL）の重みが適切に効くようにする。
    const double cov_scale = st.using_encoder_feedback ? 1.0 : 4.0;
    odom->pose.covariance[0] = cfg.pose_cov_xy * cov_scale;
    odom->pose.covariance[7] = cfg.pose_cov_xy * cov_scale;
    odom->pose.covariance[14] = 1e-6;
    odom->pose.covariance[21] = 1e-6;
    odom->pose.covariance[28] = 1e-6;
    odom->pose.covariance[35] = cfg.pose_cov_yaw * cov_scale;

    odom->twist.twist.linear.x = st.vx;
    odom->twist.twist.linear.y = 0.0;
    odom->twist.twist.angular.z = st.wz;
    odom->twist.covariance[0] = cfg.twist_cov_vx * cov_scale;
    odom->twist.covariance[7] = 1e-6;
    odom->twist.covariance[14] = 1e-6;
    odom->twist.covariance[21] = 1e-6;
    odom->twist.covariance[28] = 1e-6;
    odom->twist.covariance[35] = cfg.twist_cov_wz * cov_scale;
    odom_pub_->publish(std::move(odom));
  }
}

}  // namespace umeonigiri_serial_bridge

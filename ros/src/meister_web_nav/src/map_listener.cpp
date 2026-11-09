// SPDX-License-Identifier: MIT
#include "meister_web_nav/map_listener.hpp"

#include <cmath>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace meister_web_nav {
namespace {

/// 地図だけ TRANSIENT_LOCAL で受ける。slam_toolbox は地図を TRANSIENT_LOCAL で
/// 配信するので、後から接続した Web UI サーバーも起動時点の地図を受け取れる。
/// VOLATILE だと最初の配信を取りこぼし、「地図を待っています」のまま固まる。
rclcpp::QoS MapQos() {
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

}  // namespace

double YawOf(const geometry_msgs::msg::Quaternion& q) {
  return 2.0 * std::atan2(q.z, q.w);
}

PlanarTransform ComposePose(const PlanarTransform& map_odom,
                            const PlanarTransform& odom_base) {
  // map->base = map->odom ∘ odom->base（平面 2D なので yaw の合成は足し算）。
  return PlanarTransform{
      map_odom.tx + std::cos(map_odom.yaw) * odom_base.tx -
          std::sin(map_odom.yaw) * odom_base.ty,
      map_odom.ty + std::sin(map_odom.yaw) * odom_base.tx +
          std::cos(map_odom.yaw) * odom_base.ty,
      map_odom.yaw + odom_base.yaw};
}

MapListener::MapListener() : rclcpp::Node("web_nav_map_listener") {
  // Subscription の SharedPtr は Node が保持するので、ここでは捨てる。
  create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", MapQos(), [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        cache_.SetGrid(std::move(msg));
      });
  create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::QoS(10),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) { OnTf(*msg); });
}

void MapListener::OnTf(const tf2_msgs::msg::TFMessage& msg) {
  for (const auto& t : msg.transforms) {
    if (t.header.frame_id == "map" && t.child_frame_id == "odom") {
      const PlanarTransform planar{t.transform.translation.x,
                                   t.transform.translation.y,
                                   YawOf(t.transform.rotation)};
      std::lock_guard<std::mutex> lock(lock_);
      map_odom_ = planar;
    } else if (t.header.frame_id == "odom" &&
               (t.child_frame_id == "base_footprint" ||
                t.child_frame_id == "base_link")) {
      const PlanarTransform planar{t.transform.translation.x,
                                   t.transform.translation.y,
                                   YawOf(t.transform.rotation)};
      std::lock_guard<std::mutex> lock(lock_);
      odom_base_ = planar;
    }
  }
}

std::optional<nlohmann::json> MapListener::RobotPose() {
  std::optional<PlanarTransform> map_odom;
  std::optional<PlanarTransform> odom_base;
  {
    std::lock_guard<std::mutex> lock(lock_);
    map_odom = map_odom_;
    odom_base = odom_base_;
  }
  if (!map_odom || !odom_base) {
    return std::nullopt;
  }
  const PlanarTransform pose = ComposePose(*map_odom, *odom_base);
  return nlohmann::json{{"x", pose.tx}, {"y", pose.ty}, {"yaw", pose.yaw}};
}

std::optional<nlohmann::json> MapListener::Metadata() {
  const MapSnapshot snapshot = cache_.Snapshot();
  if (!snapshot.grid) {
    return std::nullopt;
  }
  return MetadataOf(*snapshot.grid, snapshot.bounds);
}

std::optional<std::pair<std::string, std::string>> MapListener::RenderPng() {
  return cache_.RenderPng();
}

}  // namespace meister_web_nav

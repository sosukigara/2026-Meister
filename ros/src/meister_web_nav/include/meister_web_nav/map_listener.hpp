// SPDX-License-Identifier: MIT
/// /map と /tf を購読し、Web UI が要る 3 つ（地図メタデータ / PNG / ロボット姿勢）に
/// 答える。
///
/// 地図は MapCache が持ち、ここは tf の最新値だけを持つ。
///
/// 座標変換とメタデータの計算は自由関数に出してある。Node は rclcpp::init 無しに
/// 構築できないので、クラスの中に置くと数値の検証に ROS グラフが要る。
#ifndef MEISTER_WEB_NAV_MAP_LISTENER_HPP_
#define MEISTER_WEB_NAV_MAP_LISTENER_HPP_

#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "meister_web_nav/map_cache.hpp"
#include "meister_web_nav/ports.hpp"

namespace meister_web_nav {

/// 平面 (x, y, yaw) に落とした tf。姿勢の合成に要る 3 つ以外は保持しない。
struct PlanarTransform {
  double tx = 0.0;
  double ty = 0.0;
  double yaw = 0.0;
};

/// クォータニオンの yaw。この機体は常に水平 (roll = pitch = 0) なので、
/// 全成分を使う回転行列より 2*atan2(z, w) で足りる。
double YawOf(const geometry_msgs::msg::Quaternion& q);

/// map->base = map->odom ∘ odom->base。平面 2D なので yaw の合成は足し算。
PlanarTransform ComposePose(const PlanarTransform& map_odom,
                            const PlanarTransform& odom_base);

class MapListener : public rclcpp::Node, public MapSource {
 public:
  MapListener();

  std::optional<nlohmann::json> Metadata() override;
  std::optional<std::pair<std::string, std::string>> RenderPng() override;
  std::optional<nlohmann::json> RobotPose() override;

 private:
  void OnTf(const tf2_msgs::msg::TFMessage& msg);

  std::mutex lock_;
  std::optional<PlanarTransform> map_odom_;
  std::optional<PlanarTransform> odom_base_;
  MapCache cache_;
};

}  // namespace meister_web_nav

#endif  // MEISTER_WEB_NAV_MAP_LISTENER_HPP_

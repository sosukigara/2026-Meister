// SPDX-License-Identifier: MIT
/// /map と /tf および /tf_static を購読し、Web UI が要る 3 つ（地図メタデータ / PNG / ロボット姿勢）に答える。
///
/// 重大バグ修正:
///   旧実装は `/tf` のみを購読し `/tf_static` を購読していなかったため、
///   `static_transform_publisher` 等が `/tf_static` (TransientLocal) に配信する変換を
///   一切受信できず、実機で `RobotPose()` が永久に `std::nullopt`（「未受信」）となっていた。
///   本修正により `/tf` と `/tf_static` の双方を購読し、先頭スラッシュの有無も正規化する。
#ifndef UMEONIGIRI_WEBNAV_MAP_LISTENER_HPP_
#define UMEONIGIRI_WEBNAV_MAP_LISTENER_HPP_

#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "umeonigiri_webnav/map_cache.hpp"
#include "umeonigiri_webnav/ports.hpp"

namespace umeonigiri_webnav {

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
  std::optional<MapImage> RenderPng() override;
  std::optional<nlohmann::json> RobotPose() override;

 private:
  void OnTf(const tf2_msgs::msg::TFMessage& msg);

  std::mutex lock_;
  std::optional<PlanarTransform> map_odom_;
  std::optional<PlanarTransform> odom_base_;
  MapCache cache_;
  // Subscription はメンバで保持する。戻りの SharedPtr を捨てると Jazzy では
  // 購読が破棄され、グラフにも /map が現れない (vision_listener と同じ持ち方)。
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
};

}  // namespace umeonigiri_webnav

#endif  // UMEONIGIRI_WEBNAV_MAP_LISTENER_HPP_

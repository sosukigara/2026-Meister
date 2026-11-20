// SPDX-License-Identifier: MIT
/// モード切替・保存地図・初期位置の ROS 側の実装。
///
/// すべて HTTP ハンドラから呼ばれる。1 本のハンドラが秒単位で詰まると httplib の
/// スレッドプールが枯れ、無関係な要求まで止まる（実測: /api/mode を 2 秒ごとに
/// 引く画面で、地図と姿勢の要求まで返らなくなった）。
///
/// そのため、この Node は専用スレッドの Executor で常に回す。サービス探索は
/// そのスレッドが進めるので、呼び出し側は「spin しないで待つ」だけでよい。
/// モードの照会 (/api/mode) に限っては待ち時間ゼロにしてある。
#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/srv/save_map.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "umeonigiri_webnav/ports.hpp"

namespace umeonigiri_webnav {

class ModeWorker : public ModeSink {
 public:
  /// maps_dir が空なら Node のパラメータ maps_dir（既定 "maps"）を使う。
  explicit ModeWorker(const std::string& maps_dir = "");
  ~ModeWorker() override;

  ModeWorker(const ModeWorker&) = delete;
  ModeWorker& operator=(const ModeWorker&) = delete;

  /// executor スレッドを止める。rclcpp::shutdown() より先に呼ぶ。
  void Stop();

  std::string Mode() override;
  std::string SwitchToLocalization() override;
  std::vector<std::string> ListMaps() override;
  std::string SaveMap(const std::string& name) override;
  std::string DeleteMap(const std::string& name) override;
  void SetInitialPose(double x, double y, double yaw) override;

 private:
  using Trigger = std_srvs::srv::Trigger;
  // メンバ関数 SaveMap と名前が衝突するので別名にする。
  using SaveMapService = nav2_msgs::srv::SaveMap;

  /// 要求を送って応答を待つ。応答は executor スレッドが届けるので spin しない。
  bool CallTrigger(const rclcpp::Client<Trigger>::SharedPtr& client,
                   std::string* message);

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
  /// 切替と保存は 1 本ずつ。重ねて要求されても互いを待たせない。
  std::mutex busy_;
  std::string maps_dir_;
  std::string world_;
  rclcpp::Client<Trigger>::SharedPtr mode_client_;
  rclcpp::Client<Trigger>::SharedPtr switch_client_;
  rclcpp::Client<SaveMapService>::SharedPtr save_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_publisher_;
};

}  // namespace umeonigiri_webnav

// SPDX-License-Identifier: MIT
/// モード切替・保存地図・初期位置の ROS 側の実装。
///
/// HTTP スレッドから呼ばれるので、この Node は他の Executor に載せない。
/// 呼び出しごとに spin_until_future_complete で自分の Node を回し、
/// 同時リクエストは mutex で直列化する。
#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "meister_web_nav/ports.hpp"

namespace meister_web_nav {

class ModeWorker : public ModeSink {
 public:
  /// maps_dir が空なら Node のパラメータ maps_dir（既定 "maps"）を使う。
  explicit ModeWorker(const std::string& maps_dir = "");
  ~ModeWorker() override;

  std::string Mode() override;
  std::string SwitchToLocalization() override;
  std::vector<std::string> ListMaps() override;
  std::string SaveMap(const std::string& name) override;
  std::string DeleteMap(const std::string& name) override;
  void SetInitialPose(double x, double y, double yaw) override;

 private:
  bool CallTrigger(const std::string& service, std::string* message);

  rclcpp::Node::SharedPtr node_;
  std::mutex mutex_;
  std::string maps_dir_;
};

}  // namespace meister_web_nav

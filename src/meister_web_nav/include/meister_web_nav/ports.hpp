// SPDX-License-Identifier: MIT
/// HTTP 面が実装側に要求する口。
///
/// ROS 型をここに出さない: HTTP 面は ROS グラフ無しで test できる (AGENTS.md R8)。
/// 実装は MapListener / NavWorker、テストは stub が満たす。
#ifndef MEISTER_WEB_NAV_PORTS_HPP_
#define MEISTER_WEB_NAV_PORTS_HPP_

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace meister_web_nav {

/// 目的地 1 地点。yaw は map 座標系 [rad]。
struct Waypoint {
  double x;
  double y;
  double yaw;
};

struct MapSource {
  virtual ~MapSource() = default;

  /// 地図未着なら nullopt。着いていれば resolution / width / height / origin。
  virtual std::optional<nlohmann::json> Metadata() = 0;

  /// (PNG bytes, version)。version は引用符無しの content hash。
  virtual std::optional<std::pair<std::string, std::string>> RenderPng() = 0;

  /// 姿勢が未確定なら nullopt。{"x", "y", "yaw"}。
  virtual std::optional<nlohmann::json> RobotPose() = 0;
};

struct NavSink {
  virtual ~NavSink() = default;

  virtual nlohmann::json Status() const = 0;
  virtual void Submit(const std::vector<Waypoint>& waypoints) = 0;
  virtual void Cancel() = 0;
};

/// ストリームに流す 1 フレーム。jpeg は完成品 (境界や長さは HTTP 面が付ける)。
struct VisionFrame {
  long seq;
  std::string jpeg;
};

struct VisionSource {
  virtual ~VisionSource() = default;

  /// 認識モニターの状態。monitor / has_frame / stale / fps / detections[*] など。
  virtual nlohmann::json Status() = 0;

  /// after_seq より新しいフレームを最大 timeout_s 秒待つ。after_seq に -1 を渡すと
  /// 「いま持っているフレーム」を待たずに返す。無ければ nullopt。
  virtual std::optional<VisionFrame> Latest(long after_seq, double timeout_s) = 0;

  /// 監視の ON/OFF。ここでは要求を立てるだけで、購読の作成/破棄は executor
  /// スレッドが行う (HTTP スレッドから購読を触ると executor と競合する)。
  virtual void SetEnabled(bool enabled) = 0;
};

}  // namespace meister_web_nav

#endif  // MEISTER_WEB_NAV_PORTS_HPP_

// SPDX-License-Identifier: MIT
/// HTTP 面が実装側に要求する口。
///
/// ROS 型をここに出さない: HTTP 面は ROS グラフ無しで test できる (AGENTS.md R8)。
/// 実装は MapListener / NavWorker、テストは stub が満たす。
#ifndef UMEONIGIRI_WEBNAV_PORTS_HPP_
#define UMEONIGIRI_WEBNAV_PORTS_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace umeonigiri_webnav {

/// 目的地 1 地点。yaw は map 座標系 [rad]。
struct Waypoint {
  double x;
  double y;
  double yaw;
};

/// 地図 PNG 1 枚。bytes はキャッシュと共有する: 地図が変わっていないとき
/// (304) に数百 KB のコピーを作らないため。version は引用符無しの content hash。
struct MapImage {
  std::shared_ptr<const std::string> bytes;
  std::string version;
};

struct MapSource {
  virtual ~MapSource() = default;

  /// 地図未着なら nullopt。着いていれば resolution / width / height / origin。
  virtual std::optional<nlohmann::json> Metadata() = 0;

  /// 地図未着なら nullopt。PNG は version ごとに 1 回だけ作る。
  virtual std::optional<MapImage> RenderPng() = 0;

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

/// モード切替・保存地図・初期位置。HTTP 面は ROS を知らないまま操作する。
///
/// どの実装も HTTP ハンドラから呼ばれるので、秒単位でブロックしてはいけない:
/// ハンドラが詰まると httplib のスレッドプールが枯れ、無関係な要求まで止まる。
struct ModeSink {
  virtual ~ModeSink() = default;

  /// "mapping" / "localization" / "unknown"。モードは毎秒読まれるので短く返す。
  virtual std::string Mode() = 0;

  /// 保存地図モードへ切り替える。空文字列なら成功、失敗なら理由。
  virtual std::string SwitchToLocalization() = 0;

  /// 保存済み地図の名前（拡張子なし、昇順）。
  virtual std::vector<std::string> ListMaps() = 0;

  /// 地図を保存する。name が空なら実装側が既定名を決める。
  /// 空文字列なら成功、失敗なら理由。
  virtual std::string SaveMap(const std::string& name) = 0;

  /// 保存済み地図を削除する。空文字列なら成功、失敗なら理由。
  virtual std::string DeleteMap(const std::string& name) = 0;

  /// 初期位置を配る。
  virtual void SetInitialPose(double x, double y, double yaw) = 0;
};

}  // namespace umeonigiri_webnav

#endif  // UMEONIGIRI_WEBNAV_PORTS_HPP_

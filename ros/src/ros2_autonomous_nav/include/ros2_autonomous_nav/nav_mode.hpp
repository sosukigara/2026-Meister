// SPDX-License-Identifier: MIT
/// ナビゲーションのモードと、`map→odom` を publish する権利の持ち主。
///
/// 設計書 §6.4: `map→odom` を publish してよいのは常に1ノードだけ。
/// 地図生成は slam_toolbox、保存地図での自己位置推定は AMCL が持つ。
/// 切替は「旧を deactivate → 新を activate」の順でしか行わない。
#pragma once

#include <string>

namespace ros2_autonomous_nav {

enum class NavMode { kMapping, kLocalization };

enum class MapToOdomOwner { kNone, kSlamToolbox, kAmcl };

struct ModeTransition {
  bool allowed{false};
  MapToOdomOwner deactivate_first{MapToOdomOwner::kNone};
  MapToOdomOwner activate_after{MapToOdomOwner::kNone};
  /// 許可しなかった理由。UI のメッセージにそのまま出す。
  std::string reason;
};

MapToOdomOwner OwnerForMode(NavMode mode);

/// kNone は渡さない前提。渡された場合は kMapping を返す（呼び出し側の誤り）。
NavMode ModeForOwner(MapToOdomOwner owner);

/// 遷移計画。地図生成 → 自己位置推定の一方通行だけを許す（§6.4）。
ModeTransition PlanTransition(NavMode from, NavMode to);

}  // namespace ros2_autonomous_nav

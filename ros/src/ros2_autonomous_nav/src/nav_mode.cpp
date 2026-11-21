// SPDX-License-Identifier: MIT
#include "ros2_autonomous_nav/nav_mode.hpp"

namespace ros2_autonomous_nav {

MapToOdomOwner OwnerForMode(NavMode mode) {
  return mode == NavMode::kMapping ? MapToOdomOwner::kSlamToolbox : MapToOdomOwner::kAmcl;
}

NavMode ModeForOwner(MapToOdomOwner owner) {
  return owner == MapToOdomOwner::kAmcl ? NavMode::kLocalization : NavMode::kMapping;
}

ModeTransition PlanTransition(NavMode from, NavMode to) {
  ModeTransition plan;
  if (from == to) {
    plan.reason = "既にそのモードです";
    return plan;
  }
  plan.allowed = true;
  plan.deactivate_first = OwnerForMode(from);
  plan.activate_after = OwnerForMode(to);
  // 双方向。保存地図 → 地図生成 は slam_toolbox の再 activate で戻す。
  // ポーズグラフが保持されることは実測で確認済み (前後で 332x302 / 55233 cells が一致)。
  plan.reason = to == NavMode::kLocalization
                    ? "地図生成から保存地図ナビへ切り替えます"
                    : "保存地図ナビから地図生成へ切り替えます（地図は引き継ぎます）";
  return plan;
}

}  // namespace ros2_autonomous_nav

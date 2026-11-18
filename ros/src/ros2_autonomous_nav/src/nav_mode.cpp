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
  if (from == NavMode::kLocalization && to == NavMode::kMapping) {
    plan.reason = "地図生成へ戻るには launch を再起動してください";
    return plan;
  }
  plan.allowed = true;
  plan.deactivate_first = OwnerForMode(from);
  plan.activate_after = OwnerForMode(to);
  plan.reason = "地図生成から保存地図ナビへ切り替えます";
  return plan;
}

}  // namespace ros2_autonomous_nav

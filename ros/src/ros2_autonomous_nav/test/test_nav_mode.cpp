// SPDX-License-Identifier: MIT
/// nav_mode の単体テスト。ROS を起動しない。
#include <gtest/gtest.h>

#include "ros2_autonomous_nav/nav_mode.hpp"

namespace ros2_autonomous_nav {
namespace {

TEST(NavMode, MappingModeIsOwnedBySlamToolbox) {
  EXPECT_EQ(ModeForOwner(MapToOdomOwner::kSlamToolbox), NavMode::kMapping);
}

TEST(NavMode, LocalizationModeIsOwnedByAmcl) {
  EXPECT_EQ(ModeForOwner(MapToOdomOwner::kAmcl), NavMode::kLocalization);
}

TEST(NavMode, SwitchingToLocalizationDeactivatesBeforeActivating) {
  const auto plan = PlanTransition(NavMode::kMapping, NavMode::kLocalization);
  EXPECT_TRUE(plan.allowed);
  EXPECT_EQ(plan.deactivate_first, MapToOdomOwner::kSlamToolbox);
  EXPECT_EQ(plan.activate_after, MapToOdomOwner::kAmcl);
}

TEST(NavMode, SwitchingBackIsAllowed) {
  // 設計書 §6.4 は「slam_toolbox の再 activate でポーズグラフが残るか未定義」
  // を理由に一方通行だった。実測すると保持されたので双方向にしてある。
  const auto plan = PlanTransition(NavMode::kLocalization, NavMode::kMapping);
  EXPECT_TRUE(plan.allowed);
  EXPECT_EQ(plan.deactivate_first, MapToOdomOwner::kAmcl);
  EXPECT_EQ(plan.activate_after, MapToOdomOwner::kSlamToolbox);
  EXPECT_FALSE(plan.reason.empty());
}

TEST(NavMode, SwitchingToTheSameModeIsANoOp) {
  const auto mapping = PlanTransition(NavMode::kMapping, NavMode::kMapping);
  EXPECT_FALSE(mapping.allowed);
  EXPECT_EQ(mapping.deactivate_first, MapToOdomOwner::kNone);
  EXPECT_EQ(mapping.activate_after, MapToOdomOwner::kNone);

  const auto localization = PlanTransition(NavMode::kLocalization, NavMode::kLocalization);
  EXPECT_FALSE(localization.allowed);
  EXPECT_EQ(localization.deactivate_first, MapToOdomOwner::kNone);
}

TEST(NavMode, OwnerForModeChoosesThePublisherOfMapToOdom) {
  EXPECT_EQ(OwnerForMode(NavMode::kMapping), MapToOdomOwner::kSlamToolbox);
  EXPECT_EQ(OwnerForMode(NavMode::kLocalization), MapToOdomOwner::kAmcl);
}

}  // namespace
}  // namespace ros2_autonomous_nav

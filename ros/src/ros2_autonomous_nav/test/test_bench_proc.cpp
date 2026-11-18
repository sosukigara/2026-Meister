// SPDX-License-Identifier: MIT
/// /proc の解釈の単体テスト。実プロセスには触れない。
#include <gtest/gtest.h>

#include <string>

#include "ros2_autonomous_nav/bench_proc.hpp"

namespace ros2_autonomous_nav {
namespace {

TEST(BenchProc, MatchesNodeNameFromExecutablePath) {
  EXPECT_EQ(MatchNodeName("/opt/ros/jazzy/lib/nav2_controller/controller_server "
                          "--ros-args --params-file /tmp/x.yaml"),
            "controller_server");
}

TEST(BenchProc, MatchesPlannerAndAmcl) {
  EXPECT_EQ(MatchNodeName("/opt/ros/jazzy/lib/nav2_planner/planner_server"), "planner_server");
  EXPECT_EQ(MatchNodeName("/opt/ros/jazzy/lib/nav2_amcl/amcl --ros-args"), "amcl");
}

TEST(BenchProc, IgnoresUnrelatedProcesses) {
  EXPECT_EQ(MatchNodeName("/usr/bin/bash -c echo hello"), "");
  EXPECT_EQ(MatchNodeName("/opt/ros/jazzy/lib/rviz2/rviz2"), "");
}

TEST(BenchProc, DoesNotMatchPartialWords) {
  // 「my_controller_server_wrapper」のような別物を拾わない。
  EXPECT_EQ(MatchNodeName("/usr/bin/python3 my_controller_server_wrapper.py"), "");
}

TEST(BenchProc, ReadsUtimeAndStimeFromStat) {
  // field 3 以降: S, 0 x10, utime=100, stime=50
  const std::string line = "1234 (controller_server) S 0 0 0 0 0 0 0 0 0 0 100 50 0 0";
  const auto seconds = CpuSecondsFromStat(line, 100);
  ASSERT_TRUE(seconds.has_value());
  EXPECT_DOUBLE_EQ(*seconds, 1.5);
}

TEST(BenchProc, HandlesCommasWithSpacesInsideTheName) {
  const std::string line = "1 (weird (name) here) S 0 0 0 0 0 0 0 0 0 0 10 10 0";
  const auto seconds = CpuSecondsFromStat(line, 100);
  ASSERT_TRUE(seconds.has_value());
  EXPECT_DOUBLE_EQ(*seconds, 0.2);
}

TEST(BenchProc, RejectsTruncatedStat) {
  EXPECT_FALSE(CpuSecondsFromStat("1234 (x) S 0 0", 100).has_value());
  EXPECT_FALSE(CpuSecondsFromStat("garbage", 100).has_value());
}

TEST(BenchProc, RejectsZeroClockTicks) {
  const std::string line = "1 (x) S 0 0 0 0 0 0 0 0 0 0 100 50 0";
  EXPECT_FALSE(CpuSecondsFromStat(line, 0).has_value());
}

}  // namespace
}  // namespace ros2_autonomous_nav

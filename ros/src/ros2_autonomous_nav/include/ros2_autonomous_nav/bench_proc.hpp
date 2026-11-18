// SPDX-License-Identifier: MIT
/// Nav2 ノードの CPU 時間を読む。実測は /proc だが、解釈は純関数に切り出して
/// 単体テストで固定する。
#pragma once

#include <map>
#include <optional>
#include <string>

namespace ros2_autonomous_nav {

/// CPU を積算する対象。launch が起動するノード名と対応させること（R2）。
inline constexpr const char* kCpuTargets[] = {
    "controller_server", "planner_server", "bt_navigator", "behavior_server",
    "velocity_smoother",   "amcl",           "map_server",   "slam_toolbox",
};

struct CpuSample {
  std::string node;
  double cpu_seconds{0.0};
};

/// /proc/<pid>/cmdline から一致するノード名を返す。無ければ空文字列。
std::string MatchNodeName(const std::string& cmdline);

/// /proc/<pid>/stat の 1 行から utime+stime [秒] を出す。壊れていれば nullopt。
std::optional<double> CpuSecondsFromStat(const std::string& stat_line,
                                         long clock_ticks_per_sec);

/// 現在動いている対象プロセスの CPU 時間。キーは pid 文字列。
std::map<std::string, CpuSample> SampleNavCpu();

}  // namespace ros2_autonomous_nav

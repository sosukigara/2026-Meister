// SPDX-License-Identifier: MIT
/// bench_nav のコマンドライン引数。
#pragma once

#include <string>

namespace ros2_autonomous_nav {

struct BenchOptions {
  std::string goals;
  std::string out;
  std::string map_id;
  int repeats{-1};
  int limit{-1};
  int auto_goals{-1};
  double goal_timeout{120.0};
  double sample_hz{10.0};
  double min_spacing{1.0};
  double min_clearance{0.5};
  /// 計測時の GUI 有無。"unknown" のまま推測で埋めない。
  std::string gui{"unknown"};
};

enum class ParseStatus { kOk, kHelp, kError };

/// 失敗時は stderr に理由を出し、戻り値は kHelp / kError。
ParseStatus ParseBenchArgs(int argc, char** argv, BenchOptions* options);

}  // namespace ros2_autonomous_nav

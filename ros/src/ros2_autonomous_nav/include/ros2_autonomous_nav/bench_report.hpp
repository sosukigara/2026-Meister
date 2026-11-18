// SPDX-License-Identifier: MIT
/// ベンチマーク結果の JSON 表記。nlohmann/json はこの .cpp の中だけに閉じる。
#pragma once

#include <string>
#include <vector>

#include "ros2_autonomous_nav/bench_metrics.hpp"

namespace ros2_autonomous_nav {

/// runs の配列を JSON テキストにする。
std::string RunsToJsonText(const std::vector<RunRecord>& runs);

/// summary を JSON テキストにする。測定できない値は null になる。
std::string SummaryToJsonText(const RunSummary& summary);

}  // namespace ros2_autonomous_nav

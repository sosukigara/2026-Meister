// SPDX-License-Identifier: MIT
/// ベンチマーク結果の JSON 表記。nlohmann/json はこの .cpp の中だけに閉じる。
#pragma once

#include <string>
#include <vector>

#include "umeonigiri_autonomy/bench_metrics.hpp"

namespace umeonigiri_autonomy {

/// runs の配列を JSON テキストにする。
std::string RunsToJsonText(const std::vector<RunRecord>& runs);

/// summary を JSON テキストにする。測定できない値は null になる。
std::string SummaryToJsonText(const RunSummary& summary);

}  // namespace umeonigiri_autonomy

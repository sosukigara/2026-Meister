// SPDX-License-Identifier: MIT
#include "ros2_autonomous_nav/bench_report.hpp"

#include <nlohmann/json.hpp>

namespace ros2_autonomous_nav {
namespace {

using json = nlohmann::json;

json ToJson(const std::optional<double>& value) {
  return value.has_value() ? json(*value) : json(nullptr);
}

json ToJson(const std::optional<int>& value) {
  return value.has_value() ? json(*value) : json(nullptr);
}

json ToJson(const std::optional<std::string>& value) {
  return value.has_value() ? json(*value) : json(nullptr);
}

json RunToJson(const RunRecord& run) {
  return json{{"success", run.success},
              {"duration_s", run.duration_s},
              {"path_length_m", run.path_length_m},
              {"planned_length_m", run.planned_length_m},
              {"path_length_ratio", ToJson(run.path_length_ratio)},
              {"final_error_xy", ToJson(run.final_error_xy)},
              {"final_error_yaw", ToJson(run.final_error_yaw)},
              {"min_clearance_m", ToJson(run.min_clearance_m)},
              {"min_clearance_driving_m", ToJson(run.min_clearance_driving_m)},
              {"plan_min_clearance_m", ToJson(run.plan_min_clearance_m)},
              {"result_code", ToJson(run.result_code)},
              {"nav_error_code", ToJson(run.nav_error_code)},
              {"nav_error_msg", ToJson(run.nav_error_msg)},
              {"contact_count", ToJson(run.contact_count)},
              {"planner_latency_ms", ToJson(run.planner_latency_ms)},
              {"control_hz", ToJson(run.control_hz)},
              {"control_p99_ms", ToJson(run.control_p99_ms)}};
}

}  // namespace

std::string RunsToJsonText(const std::vector<RunRecord>& runs) {
  json array = json::array();
  for (const auto& run : runs) {
    array.push_back(RunToJson(run));
  }
  return array.dump(2);
}

std::string SummaryToJsonText(const RunSummary& summary) {
  const json value{{"total", summary.total},
                   {"measured", summary.measured},
                   {"success_rate", summary.success_rate},
                   {"max_final_error_xy", ToJson(summary.max_final_error_xy)},
                   {"max_final_error_yaw", ToJson(summary.max_final_error_yaw)},
                   {"min_clearance_m", ToJson(summary.min_clearance_m)},
                   {"min_clearance_driving_m", ToJson(summary.min_clearance_driving_m)},
                   {"mean_duration_s", ToJson(summary.mean_duration_s)}};
  return value.dump(2);
}

}  // namespace ros2_autonomous_nav

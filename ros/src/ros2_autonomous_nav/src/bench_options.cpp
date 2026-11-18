// SPDX-License-Identifier: MIT
#include "ros2_autonomous_nav/bench_options.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace ros2_autonomous_nav {
namespace {

void PrintUsage() {
  std::fprintf(stderr,
               "usage: bench_nav --goals FILE --out FILE [--repeats N] [--limit N]\n"
               "                 [--goal-timeout S] [--sample-hz HZ] [--map-id ID]\n"
               "                 [--auto-goals N] [--min-spacing M] [--min-clearance M]\n"
               "                 [--gui on|off|unknown]\n");
}

/// 数値の解釈に失敗したら false。呼び出し側は ERROR を出して終わる。
bool ParseInt(const char* text, int* out) {
  char* end = nullptr;
  const long value = std::strtol(text, &end, 10);
  if (end == text || *end != '\0') {
    return false;
  }
  *out = static_cast<int>(value);
  return true;
}

bool ParseDouble(const char* text, double* out) {
  char* end = nullptr;
  const double value = std::strtod(text, &end);
  if (end == text || *end != '\0') {
    return false;
  }
  *out = value;
  return true;
}

}  // namespace

ParseStatus ParseBenchArgs(int argc, char** argv, BenchOptions* options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "ERROR: %s requires a value\n", name);
        return nullptr;
      }
      return argv[++i];
    };

    if (arg == "--help" || arg == "-h") {
      PrintUsage();
      return ParseStatus::kHelp;
    }
    if (arg == "--goals" || arg == "--out" || arg == "--map-id" || arg == "--gui") {
      const char* value = next(arg.c_str());
      if (value == nullptr) return ParseStatus::kError;
      if (arg == "--goals") options->goals = value;
      if (arg == "--out") options->out = value;
      if (arg == "--map-id") options->map_id = value;
      if (arg == "--gui") options->gui = value;
      continue;
    }
    if (arg == "--repeats" || arg == "--limit" || arg == "--auto-goals") {
      const char* value = next(arg.c_str());
      if (value == nullptr) return ParseStatus::kError;
      int parsed = 0;
      if (!ParseInt(value, &parsed)) {
        std::fprintf(stderr, "ERROR: %s expects an integer, got %s\n", arg.c_str(), value);
        return ParseStatus::kError;
      }
      if (arg == "--repeats") options->repeats = parsed;
      if (arg == "--limit") options->limit = parsed;
      if (arg == "--auto-goals") options->auto_goals = parsed;
      continue;
    }
    if (arg == "--goal-timeout" || arg == "--sample-hz" || arg == "--min-spacing" ||
        arg == "--min-clearance") {
      const char* value = next(arg.c_str());
      if (value == nullptr) return ParseStatus::kError;
      double parsed = 0.0;
      if (!ParseDouble(value, &parsed)) {
        std::fprintf(stderr, "ERROR: %s expects a number, got %s\n", arg.c_str(), value);
        return ParseStatus::kError;
      }
      if (arg == "--goal-timeout") options->goal_timeout = parsed;
      if (arg == "--sample-hz") options->sample_hz = parsed;
      if (arg == "--min-spacing") options->min_spacing = parsed;
      if (arg == "--min-clearance") options->min_clearance = parsed;
      continue;
    }
    std::fprintf(stderr, "ERROR: unknown argument %s\n", arg.c_str());
    PrintUsage();
    return ParseStatus::kError;
  }

  if (options->goals.empty() || options->out.empty()) {
    PrintUsage();
    return ParseStatus::kError;
  }
  return ParseStatus::kOk;
}

}  // namespace ros2_autonomous_nav

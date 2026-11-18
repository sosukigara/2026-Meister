// SPDX-License-Identifier: MIT
#include "ros2_autonomous_nav/bench_proc.hpp"

#include <cctype>
#include <cstdlib>
#include <dirent.h>
#include <fstream>
#include <iterator>
#include <sstream>
#include <unistd.h>

namespace ros2_autonomous_nav {

std::string MatchNodeName(const std::string& cmdline) {
  for (const char* target : kCpuTargets) {
    // 実行ファイル名として一致しているかを見る。/ や空白の直後に来るものを拾う。
    const std::string name(target);
    std::size_t pos = cmdline.find(name);
    while (pos != std::string::npos) {
      const std::size_t end = pos + name.size();
      const bool boundary_before =
          pos == 0 || cmdline[pos - 1] == '/' || std::isspace(static_cast<unsigned char>(cmdline[pos - 1]));
      const bool boundary_after =
          end >= cmdline.size() || cmdline[end] == ' ' || cmdline[end] == '\0';
      if (boundary_before && boundary_after) {
        return name;
      }
      pos = cmdline.find(name, pos + 1);
    }
  }
  return {};
}

std::optional<double> CpuSecondsFromStat(const std::string& stat_line,
                                         long clock_ticks_per_sec) {
  if (clock_ticks_per_sec <= 0) {
    return std::nullopt;
  }
  // "(comm)" の中に空白や ')' が入り得るので、最後の ") " で切る。
  const auto close = stat_line.rfind(") ");
  if (close == std::string::npos) {
    return std::nullopt;
  }
  std::istringstream rest(stat_line.substr(close + 2));
  std::string field;
  int index = 0;
  long utime = -1;
  long stime = -1;
  // ") " の直後が field 3 (state)。utime は field 14、stime は field 15。
  while (rest >> field) {
    if (index == 11) {
      utime = std::strtol(field.c_str(), nullptr, 10);
    } else if (index == 12) {
      stime = std::strtol(field.c_str(), nullptr, 10);
      break;
    }
    ++index;
  }
  if (utime < 0 || stime < 0) {
    return std::nullopt;
  }
  return static_cast<double>(utime + stime) / static_cast<double>(clock_ticks_per_sec);
}

namespace {

std::string ReadFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return {};
  }
  return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

}  // namespace

std::map<std::string, CpuSample> SampleNavCpu() {
  std::map<std::string, CpuSample> samples;
  DIR* proc = opendir("/proc");
  if (proc == nullptr) {
    return samples;
  }
  const long clock_ticks = sysconf(_SC_CLK_TCK);
  while (const dirent* entry = readdir(proc)) {
    if (!std::isdigit(static_cast<unsigned char>(entry->d_name[0]))) {
      continue;
    }
    const std::string pid(entry->d_name);
    std::string cmdline = ReadFile("/proc/" + pid + "/cmdline");
    for (char& ch : cmdline) {
      if (ch == '\0') {
        ch = ' ';
      }
    }
    const std::string node = MatchNodeName(cmdline);
    if (node.empty()) {
      continue;
    }
    const auto seconds = CpuSecondsFromStat(ReadFile("/proc/" + pid + "/stat"), clock_ticks);
    if (seconds.has_value()) {
      samples[pid] = CpuSample{node, *seconds};
    }
  }
  closedir(proc);
  return samples;
}

}  // namespace ros2_autonomous_nav

// SPDX-License-Identifier: MIT
#include "ros2_autonomous_nav/map_store.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>

namespace ros2_autonomous_nav {
namespace {

constexpr std::size_t kMaxNameLength = 64;

bool IsAllowedChar(char ch) {
  const auto lower = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  return std::isdigit(static_cast<unsigned char>(ch)) != 0 ||
         (lower >= 'a' && lower <= 'z') || ch == '_' || ch == '-';
}

}  // namespace

bool IsValidMapName(const std::string& name) {
  if (name.empty() || name.size() > kMaxNameLength) {
    return false;
  }
  return std::all_of(name.begin(), name.end(), IsAllowedChar);
}

std::string DefaultMapName(const std::string& world, const std::string& timestamp) {
  const std::string prefix = IsValidMapName(world) ? world : "map";
  return prefix + "_" + timestamp;
}

std::vector<std::string> ListMaps(const std::string& dir) {
  std::vector<std::string> names;
  std::error_code error;
  if (!std::filesystem::is_directory(dir, error)) {
    return names;
  }
  for (const auto& entry : std::filesystem::directory_iterator(dir, error)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    if (entry.path().extension() != ".yaml") {
      continue;
    }
    names.push_back(entry.path().stem().string());
  }
  std::sort(names.begin(), names.end());
  return names;
}

}  // namespace ros2_autonomous_nav

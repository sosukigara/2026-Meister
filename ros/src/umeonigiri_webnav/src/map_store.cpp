// SPDX-License-Identifier: MIT
#include "umeonigiri_webnav/map_store.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>

namespace umeonigiri_webnav {
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

std::string MapNameOrDefault(const std::string& requested, const std::string& world,
                             const std::string& timestamp) {
  return requested.empty() ? DefaultMapName(world, timestamp) : requested;
}

std::vector<std::string> ListMapNames(const std::string& dir) {
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

std::string DeleteMapFiles(const std::string& dir, const std::string& name) {
  if (!IsValidMapName(name)) {
    return "invalid map name";
  }
  std::error_code error;
  for (const char* extension : {".yaml", ".png", ".pgm"}) {
    std::filesystem::remove(std::filesystem::path(dir) / (name + extension), error);
    // 無い拡張子は失敗ではない。次の拡張子へ進む。
    error.clear();
  }
  return {};
}

}  // namespace umeonigiri_webnav

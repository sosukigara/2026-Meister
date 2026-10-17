// SPDX-License-Identifier: MIT
#include "meister_web_nav/map_cache.hpp"

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <stdexcept>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace meister_web_nav {
namespace {

constexpr uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr uint64_t kFnvPrime = 1099511628211ULL;

/// 未知セルのグレー値。Python 版 (PIL) と同じ 205。
constexpr uint8_t kUnknownGray = 205;

void HashBytes(uint64_t& h, const void* data, std::size_t len) {
  const auto* bytes = static_cast<const unsigned char*>(data);
  for (std::size_t i = 0; i < len; ++i) {
    h ^= bytes[i];
    h *= kFnvPrime;
  }
}

template <typename T>
void HashValue(uint64_t& h, const T& value) {
  HashBytes(h, &value, sizeof(T));
}

/// セル内容のハッシュを version（entity-tag の中身）にする。
///
/// メッセージの連番ではなく内容でハッシュする: 同じ地図を再送する publisher が
/// 連番を進めると If-None-Match が一致せず、毎回 200 が返って 304 の効果が消える。
std::string VersionOf(const nav_msgs::msg::OccupancyGrid& grid) {
  uint64_t h = kFnvOffset;
  HashBytes(h, grid.data.data(), grid.data.size());
  HashValue(h, grid.info.width);
  HashValue(h, grid.info.height);
  HashValue(h, grid.info.resolution);
  HashValue(h, grid.info.origin.position.x);
  HashValue(h, grid.info.origin.position.y);
  HashValue(h, grid.info.origin.orientation.x);
  HashValue(h, grid.info.origin.orientation.y);
  HashValue(h, grid.info.origin.orientation.z);
  HashValue(h, grid.info.origin.orientation.w);
  char buf[17];
  std::snprintf(buf, sizeof(buf), "%016" PRIx64, h);
  return buf;
}

}  // namespace

nlohmann::json MetadataOf(const nav_msgs::msg::OccupancyGrid& grid,
                          const std::optional<CropBounds>& bounds) {
  const auto& info = grid.info;
  int width = static_cast<int>(info.width);
  int height = static_cast<int>(info.height);
  double origin_x = info.origin.position.x;
  double origin_y = info.origin.position.y;
  if (bounds) {
    width = bounds->c1 - bounds->c0;
    height = bounds->r1 - bounds->r0;
    origin_x += bounds->c0 * info.resolution;
    origin_y += bounds->r0 * info.resolution;
  }
  return nlohmann::json{{"resolution", info.resolution},
                        {"width", width},
                        {"height", height},
                        {"origin", {{"x", origin_x}, {"y", origin_y}}}};
}

void MapCache::SetGrid(nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid) {
  // ハッシュは data 全体を走査するので state_lock の外で計算する。
  std::string version = grid ? VersionOf(*grid) : std::string();
  std::lock_guard<std::mutex> state(state_lock_);
  grid_ = std::move(grid);
  version_ = std::move(version);
}

MapCache::Stats MapCache::stats() const {
  return Stats{crop_calls_.load(), encode_calls_.load()};
}

std::optional<CropBounds> MapCache::CropBoundsOf(
    const nav_msgs::msg::OccupancyGrid& grid) {
  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);
  if (width == 0 || height == 0) {
    return std::nullopt;
  }
  // 壊れた grid（data が宣言サイズより短い）で範囲外を読まない。
  if (grid.data.size() < static_cast<std::size_t>(width) * height) {
    return std::nullopt;
  }
  int min_r = height;
  int max_r = -1;
  int min_c = width;
  int max_c = -1;
  for (int r = 0; r < height; ++r) {
    const int8_t* row = grid.data.data() + static_cast<std::size_t>(r) * width;
    for (int c = 0; c < width; ++c) {
      if (row[c] < 0) {
        continue;  // 未知セルは探索済みではない
      }
      min_r = std::min(min_r, r);
      max_r = std::max(max_r, r);
      min_c = std::min(min_c, c);
      max_c = std::max(max_c, c);
    }
  }
  if (max_r < 0) {
    return std::nullopt;
  }
  int r0 = min_r;
  int r1 = max_r + 1;
  int c0 = min_c;
  int c1 = max_c + 1;
  // 端のマーカーが欠けないよう 5% の余白を取る。
  const int pad = std::max(2, static_cast<int>(0.05 * std::min(r1 - r0, c1 - c0)));
  r0 = std::max(0, r0 - pad);
  r1 = std::min(height, r1 + pad);
  c0 = std::max(0, c0 - pad);
  c1 = std::min(width, c1 + pad);
  return CropBounds{r0, r1, c0, c1};
}

MapSnapshot MapCache::Snapshot() {
  std::lock_guard<std::mutex> memo(memo_lock_);
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid;
  std::string version;
  {
    std::lock_guard<std::mutex> state(state_lock_);
    grid = grid_;
    version = version_;
  }
  if (!grid) {
    return MapSnapshot{};
  }
  if (version != bounds_version_) {
    bounds_ = CropBoundsOf(*grid);
    bounds_version_ = version;
    crop_calls_.fetch_add(1);
  }
  return MapSnapshot{grid, version, bounds_};
}

std::string MapCache::EncodePng(const nav_msgs::msg::OccupancyGrid& grid,
                               const std::optional<CropBounds>& bounds) {
  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);
  const int r0 = bounds ? bounds->r0 : 0;
  const int r1 = bounds ? bounds->r1 : height;
  const int c0 = bounds ? bounds->c0 : 0;
  const int c1 = bounds ? bounds->c1 : width;
  cv::Mat gray(r1 - r0, c1 - c0, CV_8UC1);
  for (int r = r0; r < r1; ++r) {
    const int8_t* row = grid.data.data() + static_cast<std::size_t>(r) * width;
    // OccupancyGrid の row 0 は地図の下端、画像の row 0 は上端。
    uint8_t* out = gray.ptr<uint8_t>(r1 - 1 - r);
    for (int c = c0; c < c1; ++c) {
      const int8_t cell = row[c];
      if (cell < 0) {
        out[c - c0] = kUnknownGray;
      } else {
        // 占有 100 を黒、自由 0 を白。丸めは Python 版 (PIL) と同じく切り捨て。
        const float value = 255.0f - (static_cast<float>(cell) / 100.0f) * 255.0f;
        out[c - c0] = static_cast<uint8_t>(std::clamp(value, 0.0f, 255.0f));
      }
    }
  }
  std::vector<uint8_t> encoded;
  if (!cv::imencode(".png", gray, encoded)) {
    throw std::runtime_error("PNG の encode に失敗した");
  }
  return std::string(encoded.begin(), encoded.end());
}

std::optional<std::pair<std::string, std::string>> MapCache::RenderPng() {
  const MapSnapshot snapshot = Snapshot();
  // 0x0 の地図は encode できない。空の PNG を 200 で返すより「地図なし」に倒す。
  if (!snapshot.grid || snapshot.grid->info.width == 0 ||
      snapshot.grid->info.height == 0) {
    return std::nullopt;
  }
  std::lock_guard<std::mutex> memo(memo_lock_);
  if (snapshot.version != png_version_) {
    png_ = EncodePng(*snapshot.grid, snapshot.bounds);
    png_version_ = snapshot.version;
    encode_calls_.fetch_add(1);
  }
  return std::make_pair(png_, snapshot.version);
}

}  // namespace meister_web_nav

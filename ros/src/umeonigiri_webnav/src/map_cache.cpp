// SPDX-License-Identifier: MIT
#include "umeonigiri_webnav/map_cache.hpp"

#include <algorithm>
#include <array>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace umeonigiri_webnav {
namespace {

constexpr uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr uint64_t kFnvPrime = 1099511628211ULL;

/// 未知セルのグレー値。Python 版 (PIL) と同じ 205。
constexpr uint8_t kUnknownGray = 205;

/// OccupancyGrid の int8_t 値 (-128..127) を O(1) でグレースケール uint8_t に変換する
/// コンパイル時ルックアップテーブル（1200x1200 セル走査時の浮動小数点除算を完全排除）。
constexpr std::array<uint8_t, 256> BuildOccupancyGrayTable() noexcept {
  std::array<uint8_t, 256> table{};
  for (int i = 0; i < 256; ++i) {
    const int8_t cell = static_cast<int8_t>(static_cast<uint8_t>(i));
    if (cell < 0) {
      table[static_cast<std::size_t>(i)] = kUnknownGray;
    } else {
      const float value = 255.0f - (static_cast<float>(cell) / 100.0f) * 255.0f;
      const float clamped = value < 0.0f ? 0.0f : (value > 255.0f ? 255.0f : value);
      table[static_cast<std::size_t>(i)] = static_cast<uint8_t>(clamped);
    }
  }
  return table;
}

inline constexpr std::array<uint8_t, 256> kOccupancyGrayTable = BuildOccupancyGrayTable();

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
  if (!grid) {
    std::lock_guard<std::mutex> state(state_lock_);
    grid_.reset();
    version_.clear();
    bounds_.reset();
    return;
  }
  // ハッシュも crop も data 全体を走査するので state_lock の外で計算する。
  // ここは ROS の購読スレッド。/map は 1 Hz 程度なので、この場で crop まで済ませる。
  // 要求時に回すと、PNG を作り直している間 /api/map まで待たされる。
  const std::string version = VersionOf(*grid);
  {
    // 同一内容の再送 (SLAM は同じ地図を何度も流す) では crop をやり直さない。
    std::lock_guard<std::mutex> state(state_lock_);
    if (version == version_) {
      grid_ = std::move(grid);
      return;
    }
  }
  const std::optional<CropBounds> bounds = CropBoundsOf(*grid);
  crop_calls_.fetch_add(1);
  std::lock_guard<std::mutex> state(state_lock_);
  grid_ = std::move(grid);
  version_ = version;
  bounds_ = bounds;
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
  if (grid.data.size() < static_cast<std::size_t>(width) * height) {
    // 壊れた grid（data が宣言サイズより短い）で範囲外を読まない。
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
  std::lock_guard<std::mutex> state(state_lock_);
  return MapSnapshot{grid_, version_, bounds_};
}

std::shared_ptr<const std::string> MapCache::EncodePng(
    const nav_msgs::msg::OccupancyGrid& grid,
    const std::optional<CropBounds>& bounds) {
  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);
  const int r0 = bounds ? bounds->r0 : 0;
  const int r1 = bounds ? bounds->r1 : height;
  const int c0 = bounds ? bounds->c0 : 0;
  const int c1 = bounds ? bounds->c1 : width;
  const int crop_w = c1 - c0;
  cv::Mat gray(r1 - r0, crop_w, CV_8UC1);
  for (int r = r0; r < r1; ++r) {
    const int8_t* row = grid.data.data() + static_cast<std::size_t>(r) * width + c0;
    // OccupancyGrid の row 0 は地図の下端、画像の row 0 は上端。
    uint8_t* out = gray.ptr<uint8_t>(r1 - 1 - r);
    for (int x = 0; x < crop_w; ++x) {
      out[x] = kOccupancyGrayTable[static_cast<uint8_t>(row[x])];
    }
  }
  std::vector<uint8_t> encoded;
  // 圧縮レベル 0 (無圧縮ブロック)。配信先は同じホストのブラウザだけで、転送は
  // loopback なのでバイト数はほぼ無料。既定 (レベル 3) の 62 ms が 18 ms になり、
  // 地図が動いている間の HTTP スレッド占有が減る (1000x1000 の実測)。
  if (!cv::imencode(".png", gray, encoded, {cv::IMWRITE_PNG_COMPRESSION, 0})) {
    throw std::runtime_error("PNG の encode に失敗した");
  }
  return std::make_shared<const std::string>(
      reinterpret_cast<const char*>(encoded.data()), encoded.size());
}

std::optional<MapImage> MapCache::RenderPng() {
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid;
  std::string version;
  std::optional<CropBounds> bounds;
  {
    std::lock_guard<std::mutex> state(state_lock_);
    if (!grid_) {
      return std::nullopt;
    }
    if (grid_->info.width == 0 || grid_->info.height == 0) {
      return std::nullopt;
    }
    grid = grid_;
    version = version_;
    bounds = bounds_;
    if (version == png_version_ && png_) {
      // 変わっていない地図。encode も鍵待ちもしない。
      return MapImage{png_, version};
    }
  }

  // encode は 1 本に絞る。待っている間に別の要求が先に作り終えたら、それを配る。
  std::lock_guard<std::mutex> encode(encode_lock_);
  {
    std::lock_guard<std::mutex> state(state_lock_);
    if (version == png_version_ && png_) {
      return MapImage{png_, version};
    }
  }
  std::shared_ptr<const std::string> png = EncodePng(*grid, bounds);
  encode_calls_.fetch_add(1);
  {
    std::lock_guard<std::mutex> state(state_lock_);
    png_ = png;
    png_version_ = version;
  }
  return MapImage{std::move(png), std::move(version)};
}

}  // namespace umeonigiri_webnav

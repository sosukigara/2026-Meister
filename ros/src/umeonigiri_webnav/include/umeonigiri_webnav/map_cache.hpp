// SPDX-License-Identifier: MIT
/// 最新の OccupancyGrid を保持し、crop bounds と PNG を version ごとに 1 回だけ
/// 計算する。
///
/// Web UI は /api/map.png を 5 秒ごとに取り直す。1200x1200 の地図では crop と
/// PNG encode が 1 サイクル約 370 ms かかっていた。HTTP スレッドは ROS の配信
/// スレッドと CPU を奪い合うので、地図が変わった時だけ計算する。
///
/// crop は /map を受けたスレッドで済ませ、PNG だけを要求時に作る。両方を要求時に
/// やると、encode が終わるまで /api/map (メタデータ) まで待たされる。
///
/// rclcpp に依存しない (AGENTS.md R8)。HTTP 面は ROS グラフ無しで test できる。
#ifndef UMEONIGIRI_WEBNAV_MAP_CACHE_HPP_
#define UMEONIGIRI_WEBNAV_MAP_CACHE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <nlohmann/json.hpp>

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "umeonigiri_webnav/ports.hpp"

namespace umeonigiri_webnav {

/// 探索済みセルの bounding box。r1 / c1 は exclusive。
struct CropBounds {
  int r0;
  int r1;
  int c0;
  int c1;
};

/// grid が null なら「地図がまだ届いていない」。
struct MapSnapshot {
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid;
  std::string version;
  std::optional<CropBounds> bounds;
};

/// /api/map が返す地図メタデータ。画像は crop 済みなので、その原点は地図原点から
/// セル数だけずれる。ずれを返さないと Web UI のクリック座標が地図と合わない。
///
/// ROS ノードと HTTP の test が同じ計算を使うために、ノードの外に置く。
nlohmann::json MetadataOf(const nav_msgs::msg::OccupancyGrid& grid,
                          const std::optional<CropBounds>& bounds);

class MapCache {
 public:
  /// crop / encode の実行回数。304 が本当に再計算していないことを、応答時間では
  /// なく回数で検証するために持つ（時間は負荷で変わる）。
  struct Stats {
    uint64_t crop_calls = 0;
    uint64_t encode_calls = 0;
  };

  void SetGrid(nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid);

  /// state_lock_ は 1 回だけ取る。encode 中でも待たない。
  MapSnapshot Snapshot();

  /// 地図が未着なら nullopt。version は引用符無しの content hash で、entity-tag に
  /// 包むのは HTTP 面の責務。
  ///
  /// bytes はキャッシュと同じ実体を返す。返り値を値で持つ呼び手が増えても、
  /// 数百 KB のコピーは起きない。
  std::optional<MapImage> RenderPng();

  Stats stats() const;

 private:
  /// 既知セルの bounding box を求める。既知セルが 1 つも無ければ nullopt。
  static std::optional<CropBounds> CropBoundsOf(
      const nav_msgs::msg::OccupancyGrid& grid);
  static std::shared_ptr<const std::string> EncodePng(
      const nav_msgs::msg::OccupancyGrid& grid,
      const std::optional<CropBounds>& bounds);

  // 取得順は常に encode_lock_ -> state_lock_ に固定する。state_lock_ の中では
  // ファイル I/O も encode もしない (HTTP の別スレッドを待たせない)。

  std::mutex state_lock_;
  std::mutex encode_lock_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid_;
  std::string version_;
  std::optional<CropBounds> bounds_;
  std::shared_ptr<const std::string> png_;
  std::string png_version_;
  std::atomic<uint64_t> crop_calls_{0};
  std::atomic<uint64_t> encode_calls_{0};
};

}  // namespace umeonigiri_webnav

#endif  // UMEONIGIRI_WEBNAV_MAP_CACHE_HPP_

// SPDX-License-Identifier: MIT
/// nav_world.sdf の既知の壁ジオメトリから占有グリッド地図を作る。
///
/// SLAM を回さずに Nav2 を試すためのもの。以前は scripts/generate_map.py が
/// 同じ計算をしていた（2026-11-18 に C++ へ移行）。出力は Python 版と
/// バイト一致することをゴールデンファイルで確認している。
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace umeonigiri_autonomy {

/// PGM のピクセル値。
inline constexpr uint8_t kFreePixel = 254;
inline constexpr uint8_t kOccupiedPixel = 0;
inline constexpr uint8_t kUnknownPixel = 205;

struct MapSpec {
  double resolution{0.05};  // [m/px]
  double origin_x{-6.0};    // 左下のワールド座標
  double origin_y{-6.0};
  double width_m{12.0};
  double height_m{12.0};

  int width() const { return static_cast<int>(width_m / resolution); }
  int height() const { return static_cast<int>(height_m / resolution); }
};

/// グリッドは row-major で、row 0 が最小 y。PGM へ書くときは上下を反転する。
std::vector<uint8_t> BuildNavWorldGrid(const MapSpec& spec = MapSpec{});

/// Python 版と同じ表記で小数を書く（-6.0 を -6 にしない）。
std::string FormatDouble(double value);

/// P5 (binary) PGM を書く。書けなければ false。
bool WritePgm(const std::string& path, const MapSpec& spec,
              const std::vector<uint8_t>& grid);

/// nav2_map_server が読む YAML を書く。image はファイル名だけを持つ。書けなければ false。
bool WriteMapYaml(const std::string& path, const MapSpec& spec,
                  const std::string& image_name);

}  // namespace umeonigiri_autonomy

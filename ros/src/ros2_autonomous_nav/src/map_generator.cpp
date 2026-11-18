// SPDX-License-Identifier: MIT
#include "ros2_autonomous_nav/map_generator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <utility>

namespace ros2_autonomous_nav {
namespace {

struct Grid {
  int width;
  int height;
  std::vector<uint8_t> cells;

  Grid(int w, int h) : width(w), height(h),
                       cells(static_cast<std::size_t>(w) * h, kFreePixel) {}

  uint8_t& at(int px, int py) {
    return cells[static_cast<std::size_t>(py) * width + px];
  }
};

std::pair<int, int> WorldToPixel(const MapSpec& spec, double wx, double wy) {
  // 以前の Python 実装と同じく 0 方向への切り捨て。
  return {static_cast<int>((wx - spec.origin_x) / spec.resolution),
          static_cast<int>((wy - spec.origin_y) / spec.resolution)};
}

double CellCenterX(const MapSpec& spec, int px) {
  return spec.origin_x + (px + 0.5) * spec.resolution;
}

double CellCenterY(const MapSpec& spec, int py) {
  return spec.origin_y + (py + 0.5) * spec.resolution;
}

void DrawBox(Grid& grid, const MapSpec& spec, double cx, double cy, double sx,
             double sy, double yaw) {
  const double half_x = sx / 2.0;
  const double half_y = sy / 2.0;
  const double cos_a = std::cos(yaw);
  const double sin_a = std::sin(yaw);

  double min_x = 1e9;
  double max_x = -1e9;
  double min_y = 1e9;
  double max_y = -1e9;
  for (const double dx : {-half_x, half_x}) {
    for (const double dy : {-half_y, half_y}) {
      const double rx = cx + dx * cos_a - dy * sin_a;
      const double ry = cy + dx * sin_a + dy * cos_a;
      min_x = std::min(min_x, rx);
      max_x = std::max(max_x, rx);
      min_y = std::min(min_y, ry);
      max_y = std::max(max_y, ry);
    }
  }

  const auto [px_min, py_min] = WorldToPixel(spec, min_x, min_y);
  const auto [px_max, py_max] = WorldToPixel(spec, max_x, max_y);

  for (int py = std::max(0, py_min); py < std::min(grid.height, py_max + 1); ++py) {
    for (int px = std::max(0, px_min); px < std::min(grid.width, px_max + 1); ++px) {
      const double wx = CellCenterX(spec, px);
      const double wy = CellCenterY(spec, py);
      const double lx = (wx - cx) * cos_a + (wy - cy) * sin_a;
      const double ly = -(wx - cx) * sin_a + (wy - cy) * cos_a;
      if (std::abs(lx) <= half_x && std::abs(ly) <= half_y) {
        grid.at(px, py) = kOccupiedPixel;
      }
    }
  }
}

void DrawCylinder(Grid& grid, const MapSpec& spec, double cx, double cy, double radius) {
  const auto [px_c, py_c] = WorldToPixel(spec, cx, cy);
  const int r_px = static_cast<int>(radius / spec.resolution) + 1;
  for (int py = std::max(0, py_c - r_px); py < std::min(grid.height, py_c + r_px + 1); ++py) {
    for (int px = std::max(0, px_c - r_px); px < std::min(grid.width, px_c + r_px + 1); ++px) {
      const double wx = CellCenterX(spec, px);
      const double wy = CellCenterY(spec, py);
      if ((wx - cx) * (wx - cx) + (wy - cy) * (wy - cy) <= radius * radius) {
        grid.at(px, py) = kOccupiedPixel;
      }
    }
  }
}

}  // namespace

std::vector<uint8_t> BuildNavWorldGrid(const MapSpec& spec) {
  Grid grid(spec.width(), spec.height());

  // 外壁
  DrawBox(grid, spec, 0.0, 5.0, 10.2, 0.2, 0.0);
  DrawBox(grid, spec, 0.0, -5.0, 10.2, 0.2, 0.0);
  DrawBox(grid, spec, 5.0, 0.0, 0.2, 10.2, 0.0);
  DrawBox(grid, spec, -5.0, 0.0, 0.2, 10.2, 0.0);

  // 内部の壁
  DrawBox(grid, spec, -1.5, -3.0, 0.15, 4.0, 0.0);
  DrawBox(grid, spec, 1.5, 2.75, 0.15, 4.5, 0.0);
  DrawBox(grid, spec, -3.75, 1.5, 2.5, 0.15, 0.0);
  DrawBox(grid, spec, 3.5, -1.5, 3.0, 0.15, 0.0);

  // 柱
  DrawCylinder(grid, spec, -3.0, -3.0, 0.25);
  DrawCylinder(grid, spec, 3.0, 3.0, 0.25);

  // クレート
  DrawBox(grid, spec, -3.0, 3.2, 0.5, 0.5, 0.3);
  DrawBox(grid, spec, 3.7, -3.7, 0.6, 0.4, -0.5);

  // 囲いの外は未知として残す。
  for (int py = 0; py < grid.height; ++py) {
    for (int px = 0; px < grid.width; ++px) {
      const double wx = CellCenterX(spec, px);
      const double wy = CellCenterY(spec, py);
      if ((std::abs(wx) > 5.05 || std::abs(wy) > 5.05) &&
          grid.at(px, py) != kOccupiedPixel) {
        grid.at(px, py) = kUnknownPixel;
      }
    }
  }

  return std::move(grid.cells);
}

void WritePgm(const std::string& path, const MapSpec& spec,
              const std::vector<uint8_t>& grid) {
  std::ofstream out(path, std::ios::binary);
  out << "P5\n" << spec.width() << ' ' << spec.height() << "\n255\n";
  // PGM の row 0 は画像の上端 = ワールドの最大 y。グリッドは y 昇順なので反転する。
  for (int row = spec.height() - 1; row >= 0; --row) {
    const auto offset = static_cast<std::size_t>(row) * spec.width();
    out.write(reinterpret_cast<const char*>(grid.data() + offset), spec.width());
  }
}

std::string FormatDouble(double value) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.15g", value);
  std::string text(buffer);
  if (text.find('.') == std::string::npos && text.find('e') == std::string::npos &&
      text.find("inf") == std::string::npos && text.find("nan") == std::string::npos) {
    text += ".0";
  }
  return text;
}

void WriteMapYaml(const std::string& path, const MapSpec& spec,
                  const std::string& image_name) {
  std::ofstream out(path);
  out << "image: " << image_name << "\n";
  out << "mode: trinary\n";
  out << "resolution: " << FormatDouble(spec.resolution) << "\n";
  out << "origin: [" << FormatDouble(spec.origin_x) << ", "
      << FormatDouble(spec.origin_y) << ", 0.0]\n";
  out << "negate: 0\n";
  out << "occupied_thresh: 0.65\n";
  out << "free_thresh: 0.25\n";
}

}  // namespace ros2_autonomous_nav

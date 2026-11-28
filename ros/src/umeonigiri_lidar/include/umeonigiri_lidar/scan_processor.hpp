#pragma once

#include <cmath>
#include <limits>
#include <vector>

namespace umeonigiri_lidar {

struct ScanFilterConfig {
  bool speckle_filter_enabled = true;
  float speckle_max_neighbor_diff_m = 0.25f;
  bool deskew_enabled = true;
  double deskew_min_wz_rad_s = 0.05;
  double deskew_sign = 1.0;         // 実機で符号が逆なら -1.0 に反転
  int min_intensity = 15;           // 低反射・マルチパスノイズ除去しきい値（0 で無効）
  bool tilt_compensation_enabled = false;  // 将来の不整地走行用（IMU roll/pitch 水平射影）
  double max_tilt_rad = 0.25;              // これ以上傾いたスキャン点は 2D 地図破壊を防ぐため棄却
};

/// 10Hz スキャン中の旋回による角度歪みを、スキャン終了時刻基準で補正するオフセット [deg]。
inline double ComputeDeskewAngleOffsetDeg(
    double point_fraction,
    double scan_time_s,
    double wz_rad_s,
    double min_wz_rad_s = 0.05,
    double deskew_sign = 1.0) noexcept {
  if (std::fabs(wz_rad_s) < min_wz_rad_s || scan_time_s <= 0.0) {
    return 0.0;
  }
  const double clamped_frac = point_fraction < 0.0 ? 0.0 : (point_fraction > 1.0 ? 1.0 : point_fraction);
  const double dt_to_end_s = (1.0 - clamped_frac) * scan_time_s;
  constexpr double kRadToDeg = 180.0 / M_PI;
  return -(deskew_sign * wz_rad_s * dt_to_end_s) * kRadToDeg;
}

/// 不整地で車体が roll / pitch 傾いたとき、2D LiDAR 計測距離を水平面へ射影する。
/// 傾きが max_tilt_rad を超える場合は 2D SLAM 地図を壊さないよう +inf を返して棄却する。
inline float ProjectRangeToHorizontal(
    float range_m,
    double beam_angle_rad,
    double roll_rad,
    double pitch_rad,
    double max_tilt_rad = 0.25) noexcept {
  if (!std::isfinite(range_m) || range_m <= 0.0f) {
    return std::numeric_limits<float>::infinity();
  }
  if (std::hypot(roll_rad, pitch_rad) > max_tilt_rad) {
    return std::numeric_limits<float>::infinity();
  }
  // 車体フレーム単位ベクトル (cos(theta), sin(theta), 0) を R_y(pitch) * R_x(roll) で
  // 水平フレームへ回したときの z 成分: z = -sin(pitch)*cos(theta) + cos(pitch)*sin(roll)*sin(theta)
  const double ct = std::cos(beam_angle_rad);
  const double st = std::sin(beam_angle_rad);
  const double z_world =
      -std::sin(pitch_rad) * ct + std::cos(pitch_rad) * std::sin(roll_rad) * st;
  const double horiz_sq = 1.0 - z_world * z_world;
  if (horiz_sq <= 0.0) {
    return std::numeric_limits<float>::infinity();
  }
  return static_cast<float>(static_cast<double>(range_m) * std::sqrt(horiz_sq));
}

/// スクラッチバッファ再利用版の飛び点除去フィルタ（毎スキャンのヒープ確保を排除）。
inline size_t ApplySpeckleFilter(
    std::vector<float>& ranges,
    float max_neighbor_diff_m,
    std::vector<float>& scratch) {
  const size_t n = ranges.size();
  if (n < 3 || max_neighbor_diff_m <= 0.0f) {
    return 0;
  }

  scratch.assign(ranges.begin(), ranges.end());
  size_t removed = 0;

  for (size_t i = 0; i < n; ++i) {
    const float r = ranges[i];
    if (!std::isfinite(r)) {
      continue;
    }
    const float prev = ranges[(i + n - 1) % n];
    const float next = ranges[(i + 1) % n];

    const bool near_prev = std::isfinite(prev) && std::fabs(r - prev) <= max_neighbor_diff_m;
    const bool near_next = std::isfinite(next) && std::fabs(r - next) <= max_neighbor_diff_m;

    if (!near_prev && !near_next) {
      scratch[i] = std::numeric_limits<float>::infinity();
      ++removed;
    }
  }

  ranges.swap(scratch);
  return removed;
}

inline size_t ApplySpeckleFilter(
    std::vector<float>& ranges,
    float max_neighbor_diff_m) {
  std::vector<float> scratch;
  return ApplySpeckleFilter(ranges, max_neighbor_diff_m, scratch);
}

}  // namespace umeonigiri_lidar

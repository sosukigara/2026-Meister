// SPDX-License-Identifier: MIT
#include "meister_vision/target_selector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace meister_vision {
namespace {

float CenterX(const Detection& det) { return (det.x1 + det.x2) * 0.5f; }
float CenterY(const Detection& det) { return (det.y1 + det.y2) * 0.5f; }

float DistSq(float ax, float ay, float bx, float by) {
  const float dx = ax - bx;
  const float dy = ay - by;
  return dx * dx + dy * dy;
}

}  // namespace

TargetSelector::TargetSelector(TargetSelectorOptions options)
    : options_(options) {}

void TargetSelector::Reset() {
  has_lock_ = false;
  miss_count_ = 0;
  last_center_x_ = 0.0f;
  last_center_y_ = 0.0f;
  vel_x_ = 0.0f;
  vel_y_ = 0.0f;
  last_class_ = -1;
}

bool TargetSelector::IsCandidate(const Detection& det) const {
  if (det.confidence < options_.min_confidence) {
    return false;
  }
  if (options_.target_class >= 0 && det.class_id != options_.target_class) {
    return false;
  }
  // 幅か高さが 0 の枠は中心が定まらないので除く。
  return det.x2 > det.x1 && det.y2 > det.y1;
}

Target TargetSelector::Select(const std::vector<Detection>& detections,
                              int image_width, int image_height) {
  Target target;
  if (image_width <= 0 || image_height <= 0) {
    Reset();
    return target;
  }

  const float center_x = static_cast<float>(image_width) * 0.5f;
  const float center_y = static_cast<float>(image_height) * 0.5f;

  int chosen = -1;
  bool kept_lock = false;

  // α-β カルマン等速予測に基づく次フレーム期待中心位置
  const float pred_x = last_center_x_ + vel_x_;
  const float pred_y = last_center_y_ + vel_y_;

  // 1) 前フレームの対象が予測位置の近くに残っているならそれを追い続ける。
  if (has_lock_) {
    const float lock_radius_sq = options_.lock_radius_px * options_.lock_radius_px;
    float best_dist = std::numeric_limits<float>::max();
    for (std::size_t i = 0; i < detections.size(); ++i) {
      const Detection& det = detections[i];
      if (!IsCandidate(det) || det.class_id != last_class_) {
        continue;
      }
      const float dist = DistSq(CenterX(det), CenterY(det), pred_x, pred_y);
      if (dist <= lock_radius_sq && dist < best_dist) {
        best_dist = dist;
        chosen = static_cast<int>(i);
      }
    }
    kept_lock = chosen >= 0;
  }

  // 直前まで追っていた対象を見失った直後は、すぐ別の物体へ乗り換えず
  // 数フレーム保持する（検出の瞬断でアームが飛ばないように）。
  if (has_lock_ && chosen < 0) {
    if (miss_count_ + 1 < options_.lock_grace_frames) {
      ++miss_count_;
      last_center_x_ = pred_x;
      last_center_y_ = pred_y;
      vel_x_ *= 0.8f;
      vel_y_ *= 0.8f;
      return target;
    }
    Reset();
  }

  // 2) 追えていないときは画像中心に最も近いものへ乗り換える。
  if (chosen < 0) {
    float best_dist = std::numeric_limits<float>::max();
    for (std::size_t i = 0; i < detections.size(); ++i) {
      const Detection& det = detections[i];
      if (!IsCandidate(det)) {
        continue;
      }
      const float dist = DistSq(CenterX(det), CenterY(det), center_x, center_y);
      // 同じ距離なら信頼度が高い方を優先する。
      const bool better =
          dist < best_dist ||
          (dist == best_dist && chosen >= 0 &&
           det.confidence > detections[static_cast<std::size_t>(chosen)].confidence);
      if (better) {
        best_dist = dist;
        chosen = static_cast<int>(i);
      }
    }
  }

  if (chosen < 0) {
    Reset();
    return target;
  }

  const Detection& det = detections[static_cast<std::size_t>(chosen)];
  const float meas_cx = CenterX(det);
  const float meas_cy = CenterY(det);

  float filtered_cx = meas_cx;
  float filtered_cy = meas_cy;
  if (kept_lock) {
    const float rx = meas_cx - pred_x;
    const float ry = meas_cy - pred_y;
    const float alpha = std::clamp(options_.alpha_pos, 0.05f, 1.0f);
    const float beta = std::clamp(options_.beta_vel, 0.0f, 1.0f);
    filtered_cx = pred_x + alpha * rx;
    filtered_cy = pred_y + alpha * ry;
    vel_x_ += beta * rx;
    vel_y_ += beta * ry;
  } else {
    vel_x_ = 0.0f;
    vel_y_ = 0.0f;
  }

  target.valid = true;
  target.detection = det;
  target.offset_x = (filtered_cx - center_x) / center_x;
  target.offset_y = (filtered_cy - center_y) / center_y;
  target.locked = kept_lock;

  has_lock_ = true;
  miss_count_ = 0;
  last_center_x_ = filtered_cx;
  last_center_y_ = filtered_cy;
  last_class_ = det.class_id;
  return target;
}

}  // namespace meister_vision

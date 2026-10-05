// SPDX-License-Identifier: MIT
/// 検出枠とラベルの描画。
#ifndef MEISTER_VISION_DRAW_HPP_
#define MEISTER_VISION_DRAW_HPP_

#include <vector>

#include <opencv2/core/mat.hpp>

#include "meister_vision/yolo_detector.hpp"

namespace meister_vision {

inline constexpr double kLabelScale = 0.5;

/// 入力画像を複製し、検出枠とラベルを描いた画像を返す。
/// 枠は画像内に収まるよう座標だけクランプする（矩形なので線分は OpenCV が
/// 端で切ってくれる）。
cv::Mat DrawDetections(const cv::Mat& bgr,
                       const std::vector<Detection>& detections);

}  // namespace meister_vision

#endif  // MEISTER_VISION_DRAW_HPP_
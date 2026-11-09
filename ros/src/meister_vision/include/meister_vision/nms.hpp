// SPDX-License-Identifier: MIT
/// 重複する検出枠の抑制。
///
/// OpenCV の cv::dnn::NMSBoxes に素直に乗せる。IoU の式を自前実装すると
/// Python 版と僅かにずれて、移植のたびに検出数が変わる。
#ifndef MEISTER_VISION_NMS_HPP_
#define MEISTER_VISION_NMS_HPP_

#include <vector>

#include <opencv2/core/mat.hpp>

namespace meister_vision {

/// boxes (xyxy) と scores から、重複を除いた添字を返す。
/// 順序はスコアの降順。scores が空なら空を返す。
std::vector<int> NonMaximumSuppression(
    const std::vector<cv::Rect>& boxes, const std::vector<float>& scores,
    float score_threshold, float iou_threshold);

}  // namespace meister_vision

#endif  // MEISTER_VISION_NMS_HPP_
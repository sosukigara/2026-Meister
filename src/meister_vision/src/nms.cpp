// SPDX-License-Identifier: MIT
#include "meister_vision/nms.hpp"

#include <stdexcept>

#include <opencv2/dnn.hpp>

namespace meister_vision {

std::vector<int> NonMaximumSuppression(const std::vector<cv::Rect>& boxes,
                                       const std::vector<float>& scores,
                                       float score_threshold,
                                       float iou_threshold) {
  if (boxes.size() != scores.size()) {
    throw std::invalid_argument("NMS: boxes と scores の長さが違います");
  }
  std::vector<int> keep;
  if (boxes.empty()) {
    return keep;
  }

  // Python 版は conf をここで切らず、呼び出し側 (_postprocess) が先に
  // mask を作る。同じ順序を保つ。
  std::vector<cv::Rect> xywh;
  xywh.reserve(boxes.size());
  for (const cv::Rect& b : boxes) {
    xywh.emplace_back(b.x, b.y, b.width, b.height);
  }

  cv::dnn::NMSBoxes(xywh, scores, score_threshold, iou_threshold, keep);
  return keep;
}

}  // namespace meister_vision
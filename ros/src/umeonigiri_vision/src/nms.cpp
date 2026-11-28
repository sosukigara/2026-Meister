// SPDX-License-Identifier: MIT
#include "umeonigiri_vision/nms.hpp"

#include <stdexcept>

#include <opencv2/dnn.hpp>

namespace umeonigiri_vision {

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

  cv::dnn::NMSBoxes(boxes, scores, score_threshold, iou_threshold, keep);
  return keep;
}

}  // namespace umeonigiri_vision
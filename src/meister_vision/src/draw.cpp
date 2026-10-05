// SPDX-License-Identifier: MIT
#include "meister_vision/draw.hpp"

#include <algorithm>
#include <string>

#include <opencv2/imgproc.hpp>

namespace meister_vision {
namespace {

const cv::Scalar kBoxColor(0, 255, 0);  // BGR の緑
const cv::Scalar kTextColor(255, 255, 255);
const cv::Scalar kTextBg(0, 0, 0);
constexpr int kTextThickness = 1;
constexpr int kBoxThickness = 2;
constexpr int kLabelPadX = 4;

std::string FormatLabel(const Detection& det) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%s %.2f", det.class_name.c_str(),
                static_cast<double>(det.confidence));
  return std::string(buf);
}

}  // namespace

cv::Mat DrawDetections(const cv::Mat& bgr,
                       const std::vector<Detection>& detections) {
  cv::Mat out;
  bgr.copyTo(out);
  const int max_x = out.cols - 1;
  const int max_y = out.rows - 1;

  for (const Detection& det : detections) {
    const int x1 = std::clamp(static_cast<int>(det.x1), 0, max_x);
    const int y1 = std::clamp(static_cast<int>(det.y1), 0, max_y);
    const int x2 = std::clamp(static_cast<int>(det.x2), 0, max_x);
    const int y2 = std::clamp(static_cast<int>(det.y2), 0, max_y);
    cv::rectangle(out, cv::Point(x1, y1), cv::Point(x2, y2), kBoxColor,
                  kBoxThickness);

    const std::string label = FormatLabel(det);
    int baseline = 0;
    const cv::Size text_size =
        cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, kLabelScale,
                        kTextThickness, &baseline);
    // ラベルを枠の上へ置く。上端を越えるときは枠の上に押し戻す。
    const int top = std::max(y1 - text_size.height - baseline - kLabelPadX, 0);
    cv::rectangle(out, cv::Point(x1, top),
                  cv::Point(x1 + text_size.width + kLabelPadX, y1 - baseline),
                  kTextBg, -1);
    cv::putText(out, label, cv::Point(x1 + 2, y1 - baseline - 2),
                cv::FONT_HERSHEY_SIMPLEX, kLabelScale, kTextColor,
                kTextThickness, cv::LINE_AA);
  }
  return out;
}

}  // namespace meister_vision
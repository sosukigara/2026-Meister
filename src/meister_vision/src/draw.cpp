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
    // ラベルを枠の上へ置く。画像上端を越える枠 (y1 が小さい) では
    // 枠の内側へ落とす。上に押し戻すと負の y になり描画されない。
    const int label_h = text_size.height + baseline + 2 * kLabelPadX;
    int label_top = y1 - label_h;
    if (label_top < 0) {
      label_top = y1;
    }
    if (label_top + label_h > out.rows) {
      label_top = out.rows - label_h;
    }
    label_top = std::max(label_top, 0);
    cv::rectangle(out, cv::Point(x1, label_top),
                  cv::Point(x1 + text_size.width + 2 * kLabelPadX,
                            label_top + label_h),
                  kTextBg, -1);
    cv::putText(out, label,
                cv::Point(x1 + kLabelPadX, label_top + kLabelPadX + text_size.height),
                cv::FONT_HERSHEY_SIMPLEX, kLabelScale, kTextColor,
                kTextThickness, cv::LINE_AA);
  }
  return out;
}

}  // namespace meister_vision
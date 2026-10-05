// SPDX-License-Identifier: MIT
#include "meister_vision/letterbox.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace meister_vision {
namespace {

/// Python の round() は偶数側へ丸めるので banker's rounding を使う。
/// std::round は 0.5 で常に 0 側へ寄るため、640 等の偶数長で 1 ピクセルずれる。
int RoundHalfToEven(double v) {
  return static_cast<int>(std::nearbyint(v));
}

}  // namespace

LetterboxResult Letterbox(const cv::Mat& image, int side, int color) {
  if (image.empty()) {
    throw std::invalid_argument("letterbox: 入力画像が空です");
  }
  if (side <= 0) {
    throw std::invalid_argument("letterbox: 出力辺長は正でなければなりません");
  }

  const int h = image.rows;
  const int w = image.cols;
  LetterboxResult out;
  out.ratio = std::min(static_cast<double>(side) / h,
                       static_cast<double>(side) / w);
  const int new_h = RoundHalfToEven(h * out.ratio);
  const int new_w = RoundHalfToEven(w * out.ratio);

  cv::Mat resized;
  if (new_h == h && new_w == w) {
    resized = image;
  } else {
    cv::resize(image, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
  }

  out.padded = cv::Mat(side, side, CV_8UC3, cv::Scalar::all(color));
  out.pad_h = (side - new_h) / 2;
  out.pad_w = (side - new_w) / 2;
  const cv::Rect roi(out.pad_w, out.pad_h, new_w, new_h);
  resized.copyTo(out.padded(roi));
  return out;
}

}  // namespace meister_vision
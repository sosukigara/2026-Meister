// SPDX-License-Identifier: MIT
#include "meister_vision/letterbox.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace meister_vision {
namespace {

/// Python の round() は偶数側へ丸めるので banker's rounding を使う。
/// std::round は 0.5 で常に 0 側へ寄るため、640 等の偶数長で 1 ピクセルずれる。
int RoundHalfToEven(double v) {
  return static_cast<int>(std::nearbyint(v));
}

}  // namespace

LetterboxResult LetterboxInto(const cv::Mat& image, cv::Mat& canvas, int side,
                              int color) {
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
  out.pad_h = (side - new_h) / 2;
  out.pad_w = (side - new_w) / 2;

  if (canvas.rows != side || canvas.cols != side || canvas.type() != CV_8UC3) {
    canvas.create(side, side, CV_8UC3);
  }

  // リサイズ結果をキャンバス内の ROI へ直接書き込む（中間 cv::Mat の確保とコピーを削減）
  const cv::Rect roi(out.pad_w, out.pad_h, new_w, new_h);
  cv::Mat dst_roi = canvas(roi);
  if (new_h == h && new_w == w) {
    image.copyTo(dst_roi);
  } else {
    cv::resize(image, dst_roi, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
  }

  // 余白（上下左右のパディング帯）のみを color で塗潰す（中央75%領域の無駄な塗潰しを削減）
  const cv::Scalar pad_scalar = cv::Scalar::all(color);
  const int bottom_h = side - (out.pad_h + new_h);
  const int right_w = side - (out.pad_w + new_w);
  if (out.pad_h > 0) {
    canvas(cv::Rect(0, 0, side, out.pad_h)).setTo(pad_scalar);
  }
  if (bottom_h > 0) {
    canvas(cv::Rect(0, out.pad_h + new_h, side, bottom_h)).setTo(pad_scalar);
  }
  if (out.pad_w > 0) {
    canvas(cv::Rect(0, out.pad_h, out.pad_w, new_h)).setTo(pad_scalar);
  }
  if (right_w > 0) {
    canvas(cv::Rect(out.pad_w + new_w, out.pad_h, right_w, new_h)).setTo(pad_scalar);
  }

  out.padded = canvas;
  return out;
}

LetterboxResult Letterbox(const cv::Mat& image, int side, int color) {
  cv::Mat canvas;
  return LetterboxInto(image, canvas, side, color);
}

}  // namespace meister_vision

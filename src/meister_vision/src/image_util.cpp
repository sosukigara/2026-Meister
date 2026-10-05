// SPDX-License-Identifier: MIT
#include "meister_vision/image_util.hpp"

#include <string>

#include <opencv2/imgproc.hpp>

namespace meister_vision {
namespace {

/// step と data の整合を見る。壊れた step でそのまま cv::Mat の view に
/// すると、変換の失敗としてではなく未定義動作として落ちる。
/// bytes_per_pixel は 1 ピクセルあたりのバイト数で、step との比較は
/// 呼び出し側が幅と掛け合わせるので、ここでは掛け算しない。
void RequireLayout(const sensor_msgs::msg::Image& msg, size_t bytes_per_pixel) {
  const size_t min_step = msg.width * bytes_per_pixel;
  if (msg.width == 0 || msg.height == 0) {
    throw ImageConversionError("画像が壊れています: 幅か高さが 0 です");
  }
  if (msg.step < min_step) {
    throw ImageConversionError(
        "画像が壊れています: step=" + std::to_string(msg.step) + " が幅 " +
        std::to_string(msg.width) + " x " + std::to_string(bytes_per_pixel) +
        " = " + std::to_string(min_step) + " に足りない");
  }
  if (msg.data.size() < msg.step * msg.height) {
    throw ImageConversionError(
        "画像が壊れています: data が step x height に足りない (data=" +
        std::to_string(msg.data.size()) + ", 必要 " +
        std::to_string(msg.step * msg.height) + ")");
  }
}

}  // namespace

cv::Mat BgrFromImageMsg(const sensor_msgs::msg::Image& msg) {
  const std::string& enc = msg.encoding;
  const size_t w = msg.width;
  const int rows = static_cast<int>(msg.height);
  const int cols = static_cast<int>(w);
  // msg は const 参照だが読むだけなので view のポインタは const を外す。
  void* const raw = const_cast<uint8_t*>(msg.data.data());
  const size_t step = msg.step;

  if (enc == "bgr8") {
    RequireLayout(msg, 3);
    return cv::Mat(rows, cols, CV_8UC3, raw, step).clone();
  }
  if (enc == "rgb8") {
    RequireLayout(msg, 3);
    cv::Mat src(rows, cols, CV_8UC3, raw, step);
    cv::Mat bgr;
    cv::cvtColor(src, bgr, cv::COLOR_RGB2BGR);
    return bgr;
  }
  if (enc == "rgba8" || enc == "bgra8") {
    RequireLayout(msg, 4);
    cv::Mat src(rows, cols, CV_8UC4, raw, step);
    cv::Mat bgr;
    cv::cvtColor(src, bgr,
                 enc == "rgba8" ? cv::COLOR_RGBA2BGR : cv::COLOR_BGRA2BGR);
    return bgr;
  }
  if (enc == "mono8" || enc == "8UC1") {
    RequireLayout(msg, 1);
    cv::Mat src(rows, cols, CV_8UC1, raw, step);
    cv::Mat bgr;
    cv::cvtColor(src, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
  }
  if (enc == "yuv422_yuy2") {
    RequireLayout(msg, 2);
    cv::Mat src(rows, cols, CV_8UC2, raw, step);
    cv::Mat bgr;
    cv::cvtColor(src, bgr, cv::COLOR_YUV2BGR_YUY2);
    return bgr;
  }
  if (enc == "16UC1") {
    RequireLayout(msg, 2);
    return cv::Mat(rows, cols, CV_16UC1,
                   reinterpret_cast<uint16_t*>(raw), step)
        .clone();
  }
  if (enc == "32FC1") {
    RequireLayout(msg, 4);
    return cv::Mat(rows, cols, CV_32FC1,
                   reinterpret_cast<float*>(raw), step)
        .clone();
  }
  // 末尾一致のフォールバックは置かない。未知の encoding を「uint8 のまま」
  // と読むとチャンネル数がズレ、画像が静かに壊れる。
  throw ImageConversionError("未対応のエンコーディング: " + enc);
}

sensor_msgs::msg::Image ImageMsgFromBgr(const cv::Mat& bgr) {
  if (bgr.empty() || bgr.type() != CV_8UC3) {
    throw ImageConversionError("bgr8 で書き出せるのは CV_8UC3 だけです");
  }
  cv::Mat contiguous;
  bgr.copyTo(contiguous);

  sensor_msgs::msg::Image msg;
  msg.height = static_cast<uint32_t>(contiguous.rows);
  msg.width = static_cast<uint32_t>(contiguous.cols);
  msg.encoding = "bgr8";
  msg.is_bigendian = 0;
  msg.step = static_cast<uint32_t>(contiguous.step);
  msg.data.assign(contiguous.ptr<uint8_t>(),
                  contiguous.ptr<uint8_t>() +
                      static_cast<size_t>(contiguous.step) * contiguous.rows);
  return msg;
}

}  // namespace meister_vision
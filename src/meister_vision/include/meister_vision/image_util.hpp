// SPDX-License-Identifier: MIT
/// sensor_msgs/Image と OpenCV の cv::Mat の相互変換。
///
/// detection_node と hand_landmarks_node は同じ変換を使う。二重に持つと
/// 対応するエンコーディングの片方だけが直され、同じ画像に違う結果が出る
/// （AGENTS.md R2）。
#ifndef MEISTER_VISION_IMAGE_UTIL_HPP_
#define MEISTER_VISION_IMAGE_UTIL_HPP_

#include <stdexcept>
#include <string>

#include <opencv2/core/mat.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace meister_vision {

/// 画像メッセージの変換失敗。cv_bridge 経路も手動変換経路も、経路に
/// 依存せず同じ型で落とすため、派生は std::runtime_error に揃える。
class ImageConversionError : public std::runtime_error {
 public:
  explicit ImageConversionError(const std::string& what)
      : std::runtime_error(what) {}
};

/// sensor_msgs/Image を BGR の CV_8UC3 に変換する。
/// 未対応のエンコーディングは ImageConversionError。
cv::Mat BgrFromImageMsg(const sensor_msgs::msg::Image& msg);

/// BGR の CV_8UC3 を bgr8 の sensor_msgs/Image にする。
sensor_msgs::msg::Image ImageMsgFromBgr(const cv::Mat& bgr);

}  // namespace meister_vision

#endif  // MEISTER_VISION_IMAGE_UTIL_HPP_
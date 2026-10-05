// SPDX-License-Identifier: MIT
#include "meister_vision/hand_landmark_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgproc.hpp>

#include "meister_vision/letterbox.hpp"
#include "meister_vision/onnx_session.hpp"

namespace meister_vision {
namespace {

inline constexpr char kSourceDir[] = MEISTER_VISION_SOURCE_DIR;

std::vector<std::string> HandCandidatePaths() {
  std::vector<std::string> out;
  if (const char* env = std::getenv(kHandModelEnvVar);
      env != nullptr && *env != '\0') {
    out.emplace_back(env);
  }
  try {
    out.push_back(ament_index_cpp::get_package_share_directory(
                      "meister_vision") +
                  "/models/" + kHandModelFile);
  } catch (const std::exception&) {
    // 未インストール（テスト中のソース実行）では共有ディレクトリが無い。
  }
  out.push_back(std::string(kSourceDir) + "/models/" + kHandModelFile);
  return out;
}

}  // namespace

TensorLayout ProbeLayout(const std::vector<int64_t>& shape, int* input_size) {
  const auto static_dims = [&shape](size_t i) { return shape[i] > 0; };
  if (shape.size() != 4 || !static_dims(0) || !static_dims(1) || !static_dims(2) ||
      !static_dims(3)) {
    throw std::runtime_error(
        "未対応の入力形状。静的 4 次元でチャンネル次元が 3 のモデル"
        "（NHWC (1,S,S,3) / NCHW (1,3,S,S)）のみ受け付けます。");
  }
  if (shape[3] == 3) {
    *input_size = static_cast<int>(shape[1]);
    return TensorLayout::kNhwc;
  }
  if (shape[1] == 3) {
    *input_size = static_cast<int>(shape[2]);
    return TensorLayout::kNchw;
  }
  throw std::runtime_error(
      "未対応の入力形状。チャンネル次元が 3 でなければ受け付けません。");
}

std::optional<HandLandmarks> DecodeHandLandmarks(const std::vector<float>& output,
                                                 int num_landmarks,
                                                 int input_size, double ratio,
                                                 int pad_w, int pad_h,
                                                 int image_width,
                                                 int image_height,
                                                 float score_threshold) {
  const size_t expected =
      static_cast<size_t>(num_landmarks) * 3;  // 1 手分の (x, y, score)
  if (output.size() != expected) {
    throw std::runtime_error("未対応の出力要素数（期待 " +
                             std::to_string(expected) + ", 実際 " +
                             std::to_string(output.size()) +
                             "）。[x, y, score] を landmark ごとに並べた形のみ対応");
  }

  // landmark ごとに score を持つ形なので、最も低い点で hand 全体の信頼性を
  // 決める。平均だと 1 点だけ崩れた landmark を隠す。
  float confidence = 1.0f;
  for (int k = 0; k < num_landmarks; ++k) {
    confidence = std::min(confidence, output[static_cast<size_t>(k) * 3 + 2]);
  }
  if (confidence < score_threshold) {
    return std::nullopt;
  }
  if (!(ratio > 0.0)) {
    throw std::invalid_argument("letterbox の縮尺が 0 以下です");
  }

  HandLandmarks out;
  out.confidence = confidence;
  out.landmarks.reserve(static_cast<size_t>(num_landmarks));
  for (int k = 0; k < num_landmarks; ++k) {
    const size_t base = static_cast<size_t>(k) * 3;
    const double x = (output[base + 0] * input_size - pad_w) / ratio;
    const double y = (output[base + 1] * input_size - pad_h) / ratio;
    out.landmarks.emplace_back(
        static_cast<float>(std::clamp(x, 0.0, static_cast<double>(image_width))),
        static_cast<float>(std::clamp(y, 0.0, static_cast<double>(image_height))));
  }
  return out;
}

HandLandmarkDetector::HandLandmarkDetector(const std::string& model_path,
                                           float score_threshold,
                                           int num_landmarks,
                                           int intra_op_threads)
    : model_path_(model_path.empty() ? ResolveHandModelPath() : model_path),
      score_threshold_(score_threshold),
      num_landmarks_(num_landmarks) {
  if (!std::filesystem::exists(model_path_)) {
    std::string tried;
    for (const std::string& p : HandCandidatePaths()) {
      tried += "\n  - " + p;
    }
    throw std::runtime_error(
        "手 keypoint モデルが見つかりません: " + model_path_ +
        "\n探索したパス:" + tried + "\n配置方法は 3 つ: パラメータ "
        "model_path / 環境変数 MEISTER_VISION_HAND_MODEL / models/" +
        kHandModelFile +
        "\nyolov8n.onnx は COCO 80 クラスで手 keypoint を出さないため、"
        "このノードには別のモデルが必要です。");
  }

  session_ = std::make_shared<OnnxSession>(model_path_, intra_op_threads);
  layout_ = ProbeLayout(session_->input_shape(), &input_size_);
}

std::optional<HandLandmarks> HandLandmarkDetector::Detect(
    const cv::Mat& image_bgr) const {
  if (image_bgr.empty() || image_bgr.dims != 2 || image_bgr.channels() != 3) {
    throw std::invalid_argument("入力画像は (H, W, 3) の BGR 画像が必要です");
  }

  const LetterboxResult lb = Letterbox(image_bgr, input_size_, kYoloPadColor);
  // keypoint モデルは RGB で学習されている前提。入力名で分岐する手法は、
  // 名前が違う書き出しで静かに外れるので採らない。
  cv::Mat rgb;
  cv::cvtColor(lb.padded, rgb, cv::COLOR_BGR2RGB);
  rgb.convertTo(rgb, CV_32F, 1.0 / 255.0);

  const size_t plane = static_cast<size_t>(input_size_) * input_size_;
  std::vector<float> blob(3 * plane);
  for (int y = 0; y < input_size_; ++y) {
    const float* row = rgb.ptr<float>(y);
    const size_t row_base = static_cast<size_t>(y) * input_size_;
    for (int x = 0; x < input_size_; ++x) {
      const float* px = row + static_cast<size_t>(x) * 3;
      const size_t at = row_base + static_cast<size_t>(x);
      if (layout_ == TensorLayout::kNhwc) {
        blob[at * 3 + 0] = px[0];
        blob[at * 3 + 1] = px[1];
        blob[at * 3 + 2] = px[2];
      } else {
        blob[at] = px[0];
        blob[plane + at] = px[1];
        blob[2 * plane + at] = px[2];
      }
    }
  }

  return DecodeHandLandmarks(session_->Run(blob), num_landmarks_, input_size_,
                             lb.ratio, lb.pad_w, lb.pad_h, image_bgr.cols,
                             image_bgr.rows, score_threshold_);
}

std::string ResolveHandModelPath() {
  for (const std::string& p : HandCandidatePaths()) {
    if (std::filesystem::exists(p)) {
      return p;
    }
  }
  return HandCandidatePaths().back();
}

}  // namespace meister_vision
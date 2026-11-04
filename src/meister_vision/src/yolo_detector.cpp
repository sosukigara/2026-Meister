// SPDX-License-Identifier: MIT
#include "meister_vision/yolo_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include "meister_vision/letterbox.hpp"
#include "meister_vision/nms.hpp"
#include "meister_vision/onnx_session.hpp"

namespace meister_vision {
namespace {

const char* const kCocoNames[kYoloNumClasses] = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
};

inline constexpr char kModelEnvVar[] = "MEISTER_VISION_MODEL";
inline constexpr char kModelFile[] = "yolov8n.onnx";
inline constexpr char kSourceDir[] = MEISTER_VISION_SOURCE_DIR;

/// 候補を優先順位順に返す。実在判定はしない（呼び出し側に任せる）。
std::vector<std::string> YoloCandidatePaths() {
  std::vector<std::string> out;
  if (const char* env = std::getenv(kModelEnvVar);
      env != nullptr && *env != '\0') {
    out.emplace_back(env);
  }
  try {
    out.push_back(ament_index_cpp::get_package_share_directory(
                      "meister_vision") +
                  "/models/" + kModelFile);
  } catch (const std::exception&) {
    // 未インストール（テスト中のソース実行）では共有ディレクトリが無い。
  }
  out.push_back(std::string(kSourceDir) + "/models/" + kModelFile);
  return out;
}

void RequireBgr3(const cv::Mat& image) {
  if (image.empty() || image.dims != 2 || image.channels() != 3) {
    throw std::invalid_argument(
        "入力画像は (H, W, 3) の BGR 画像が必要です");
  }
}

std::string JoinTried(const std::vector<std::string>& paths) {
  std::string out;
  for (const std::string& p : paths) {
    out += "\n  - " + p;
  }
  return out;
}

}  // namespace

const std::vector<std::string>& CocoClasses() {
  static const std::vector<std::string> names(kCocoNames,
                                              kCocoNames + kYoloNumClasses);
  return names;
}

std::vector<Detection> YoloPostprocess(const std::vector<float>& output,
                                       int grid_cells, double ratio, int pad_w,
                                       int pad_h, int image_width,
                                       int image_height, float conf_threshold,
                                       float iou_threshold) {
  const size_t expected =
      static_cast<size_t>(kYoloNumClasses + kYoloBBoxCols) * grid_cells;
  if (output.size() != expected) {
    throw std::runtime_error("YOLO 出力の形が違う（期待 " +
                             std::to_string(expected) + " 要素, 実際 " +
                             std::to_string(output.size()) + "）");
  }
  if (!(ratio > 0.0)) {
    throw std::invalid_argument("letterbox の縮尺が 0 以下です");
  }

  std::vector<Detection> candidates;
  std::vector<cv::Rect> boxes;
  std::vector<float> scores;
  const double max_w = static_cast<double>(image_width);
  const double max_h = static_cast<double>(image_height);

  // ONNX 出力は [1, 84, 8400]（チャンネル優先）なので、anchor ごとの
  // 値は「チャンネル * kYoloGridCells + anchor」で取り出す。
  for (int cell = 0; cell < grid_cells; ++cell) {
    const size_t a = static_cast<size_t>(cell);
    const size_t stride = static_cast<size_t>(grid_cells);

    int best_class = 0;
    float best_score = output[4 * stride + a];
    for (int c = 1; c < kYoloNumClasses; ++c) {
      const float s = output[(4 + c) * stride + a];
      if (s > best_score) {
        best_score = s;
        best_class = c;
      }
    }
    if (best_score < conf_threshold) {
      continue;
    }

    // cx,cy,w,h（640x640 座標）から元画像座標の xyxy へ戻す。
    const double cx = output[0 * stride + a];
    const double cy = output[1 * stride + a];
    const double w = output[2 * stride + a];
    const double h = output[3 * stride + a];

    Detection det;
    det.class_id = best_class;
    det.class_name = CocoClasses()[best_class];
    det.confidence = best_score;
    det.x1 = static_cast<float>(std::clamp((cx - w / 2.0 - pad_w) / ratio, 0.0, max_w));
    det.y1 = static_cast<float>(std::clamp((cy - h / 2.0 - pad_h) / ratio, 0.0, max_h));
    det.x2 = static_cast<float>(std::clamp((cx + w / 2.0 - pad_w) / ratio, 0.0, max_w));
    det.y2 = static_cast<float>(std::clamp((cy + h / 2.0 - pad_h) / ratio, 0.0, max_h));

    boxes.emplace_back(static_cast<int>(det.x1), static_cast<int>(det.y1),
                       static_cast<int>(det.x2 - det.x1),
                       static_cast<int>(det.y2 - det.y1));
    scores.push_back(best_score);
    candidates.push_back(std::move(det));
  }

  // NMS は採用された添字をスコアの降順で返す。その順がそのまま出力順になる。
  const std::vector<int> keep =
      NonMaximumSuppression(boxes, scores, 0.0f, iou_threshold);

  std::vector<Detection> out;
  out.reserve(keep.size());
  for (const int idx : keep) {
    out.push_back(std::move(candidates[static_cast<size_t>(idx)]));
  }
  return out;
}

YoloDetector::YoloDetector(const std::string& model_path, float conf_threshold,
                           float iou_threshold, int intra_op_threads)
    : model_path_(model_path.empty() ? ResolveYoloModelPath() : model_path),
      conf_threshold_(conf_threshold),
      iou_threshold_(iou_threshold) {
  const std::vector<std::string> tried = YoloCandidatePaths();
  if (!std::filesystem::exists(model_path_)) {
    throw std::runtime_error("モデルが見つかりません: " + model_path_ +
                             "\n試したパス:" + JoinTried(tried) +
                             "\n配置方法は 2 つ: パラメータ model_path / "
                             "環境変数 " + kModelEnvVar + " にパスを設定");
  }

  session_ = std::make_shared<OnnxSession>(model_path_, intra_op_threads);
  const std::vector<int64_t>& shape = session_->input_shape();
  // 入力解像度はモデルから取る。640 以外 (320/416/480) も同じ経路で扱える。
  if (shape.size() != 4 || shape[0] != 1 || shape[1] != 3 ||
      shape[2] != shape[3]) {
    throw std::runtime_error(
        "YOLO モデルの入力形が想定と違う。NCHW の正方形を期待した");
  }
  input_size_ = static_cast<int>(shape[2]);
  if (input_size_ <= 0 || input_size_ % 32 != 0) {
    throw std::runtime_error("YOLO の入力解像度が 32 の倍数でない: " +
                             std::to_string(input_size_));
  }
  const std::vector<int64_t>& out_shape = session_->output_shape();
  // 出力は (1, 84, N)。N は入力解像度で変わる (640->8400, 480->4725)。
  if (out_shape.size() != 3 || out_shape[1] != kYoloNumClasses + kYoloBBoxCols) {
    throw std::runtime_error(
        "YOLO の出力形が想定と違う。[1, 84, N] を期待した");
  }
  grid_cells_ = static_cast<int>(out_shape[2]);
}

std::vector<Detection> YoloDetector::Detect(const cv::Mat& image_bgr) const {
  RequireBgr3(image_bgr);

  const LetterboxResult lb = Letterbox(image_bgr, input_size_, kYoloPadColor);

  // swapRB は無条件の BGR から RGB。チャンネル順は入力名でも入力 shape でも
  // 判らない（ONNX に RGB/BGR の表現が無く metadata もチャンネル数だけ）
  // ので、「Ultralytics は RGB で学習した」というモデル契約として固定する。
  // 名前や shape で推測すると別モデルで静かに壊れる。
  //
  // 手書きの 3 重ループ (約 9ms) は遅いので、OpenCV の最適化済み
  // blobFromImage に任せる (約 2ms)。letterbox 済みなので resize は無変換。
  cv::Mat blob_mat;
  cv::dnn::blobFromImage(lb.padded, blob_mat, 1.0 / 255.0,
                         cv::Size(input_size_, input_size_), cv::Scalar(),
                         /*swapRB=*/true, /*crop=*/false, CV_32F);
  std::vector<float> blob(blob_mat.ptr<float>(),
                          blob_mat.ptr<float>() + blob_mat.total());

  return YoloPostprocess(session_->Run(blob), grid_cells_, lb.ratio, lb.pad_w,
                         lb.pad_h, image_bgr.cols, image_bgr.rows,
                         conf_threshold_, iou_threshold_);
}

std::string ResolveYoloModelPath() {
  for (const std::string& p : YoloCandidatePaths()) {
    if (std::filesystem::exists(p)) {
      return p;
    }
  }
  return YoloCandidatePaths().back();
}

}  // namespace meister_vision
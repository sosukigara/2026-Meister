// SPDX-License-Identifier: MIT
#include "umeonigiri_vision/yolo_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <numeric>
#include <stdexcept>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include "umeonigiri_vision/letterbox.hpp"
#include "umeonigiri_vision/onnx_session.hpp"

namespace umeonigiri_vision {
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

std::vector<std::string> YoloCandidatePaths() {
  std::vector<std::string> out;
  if (const char* env = std::getenv(kModelEnvVar);
      env != nullptr && *env != '\0') {
    out.emplace_back(env);
  }
  try {
    out.push_back(ament_index_cpp::get_package_share_directory(
                      "umeonigiri_vision") +
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

// 整数 cv::Rect への丸め誤差と異クラス間の誤抑制（例: person 上の backpack）を防ぐため、
// float 座標のままクラス別 NMS を実行する（NPU 版 RknnYolo::Nms と同じ契約）。
std::vector<Detection> ClassAwareNms(std::vector<Detection>& candidates,
                                     float iou_threshold) {
  const size_t n = candidates.size();
  std::vector<size_t> order(n);
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
            [&candidates](size_t a, size_t b) {
              return candidates[a].confidence > candidates[b].confidence;
            });

  std::vector<float> areas(n);
  for (size_t i = 0; i < n; ++i) {
    const Detection& d = candidates[i];
    areas[i] = (d.x2 - d.x1) * (d.y2 - d.y1);
  }

  std::vector<uint8_t> suppressed(n, 0);
  std::vector<Detection> out;
  out.reserve(n);

  for (size_t oi = 0; oi < n; ++oi) {
    const size_t idx_i = order[oi];
    if (suppressed[idx_i]) {
      continue;
    }
    const Detection& di = candidates[idx_i];
    const float area_i = areas[idx_i];
    const int cls_i = di.class_id;

    for (size_t oj = oi + 1; oj < n; ++oj) {
      const size_t idx_j = order[oj];
      if (suppressed[idx_j] || candidates[idx_j].class_id != cls_i) {
        continue;
      }
      const Detection& dj = candidates[idx_j];
      const float iw = std::min(di.x2, dj.x2) - std::max(di.x1, dj.x1);
      if (iw <= 0.0f) {
        continue;
      }
      const float ih = std::min(di.y2, dj.y2) - std::max(di.y1, dj.y1);
      if (ih <= 0.0f) {
        continue;
      }
      const float inter = iw * ih;
      const float uni = area_i + areas[idx_j] - inter;
      if (uni > 0.0f && inter > iou_threshold * uni) {
        suppressed[idx_j] = 1;
      }
    }
    out.push_back(std::move(candidates[idx_i]));
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
  if (grid_cells <= 0) {
    throw std::runtime_error("YOLO 格子数が不正です");
  }
  const size_t stride = static_cast<size_t>(grid_cells);
  const size_t expected =
      static_cast<size_t>(kYoloNumClasses + kYoloBBoxCols) * stride;
  if (output.size() != expected) {
    throw std::runtime_error("YOLO 出力の形が違う（期待 " +
                             std::to_string(expected) + " 要素, 実際 " +
                             std::to_string(output.size()) + "）");
  }
  if (!(ratio > 0.0)) {
    throw std::invalid_argument("letterbox の縮尺が 0 以下です");
  }

  // 80 クラス × stride の大半は閾値未満なので、16 要素ブロックの最大値を
  // 読み取り専用で SIMD リダクションし、閾値を超えたセルのみ記録する。
  thread_local std::vector<float> max_scores;
  thread_local std::vector<uint8_t> best_classes;
  thread_local std::vector<uint8_t> active_flags;
  thread_local std::vector<size_t> active_cells;
  if (max_scores.size() < stride) {
    max_scores.resize(stride);
    best_classes.resize(stride);
    active_flags.assign(stride, 0);
  }
  active_cells.clear();

  const float* const raw = output.data();
  auto record_hit = [&](size_t idx, float score, uint8_t cid) {
    if (!active_flags[idx]) {
      active_flags[idx] = 1;
      active_cells.push_back(idx);
      max_scores[idx] = score;
      best_classes[idx] = cid;
    } else if (score > max_scores[idx]) {
      max_scores[idx] = score;
      best_classes[idx] = cid;
    }
  };

  for (int c = 0; c < kYoloNumClasses; ++c) {
    const float* const class_row = raw + static_cast<size_t>(4 + c) * stride;
    const uint8_t cid = static_cast<uint8_t>(c);
    size_t a = 0;
    for (; a + 16 <= stride; a += 16) {
      float blk_max = class_row[a];
      for (size_t k = 1; k < 16; ++k) {
        const float v = class_row[a + k];
        blk_max = v > blk_max ? v : blk_max;
      }
      if (blk_max >= conf_threshold) {
        for (size_t k = 0; k < 16; ++k) {
          const float s = class_row[a + k];
          if (s >= conf_threshold) {
            record_hit(a + k, s, cid);
          }
        }
      }
    }
    for (; a < stride; ++a) {
      const float s = class_row[a];
      if (s >= conf_threshold) {
        record_hit(a, s, cid);
      }
    }
  }

  if (active_cells.empty()) {
    return {};
  }

  std::vector<Detection> candidates;
  candidates.reserve(active_cells.size());

  const double max_w = static_cast<double>(image_width);
  const double max_h = static_cast<double>(image_height);
  const double inv_ratio = 1.0 / ratio;
  const auto& class_names = CocoClasses();

  const float* const cx_row = raw + 0 * stride;
  const float* const cy_row = raw + 1 * stride;
  const float* const w_row = raw + 2 * stride;
  const float* const h_row = raw + 3 * stride;

  for (const size_t a : active_cells) {
    active_flags[a] = 0;

    const double cx = cx_row[a];
    const double cy = cy_row[a];
    const double half_w = w_row[a] * 0.5;
    const double half_h = h_row[a] * 0.5;

    const float x1 = static_cast<float>(std::clamp((cx - half_w - pad_w) * inv_ratio, 0.0, max_w));
    const float y1 = static_cast<float>(std::clamp((cy - half_h - pad_h) * inv_ratio, 0.0, max_h));
    const float x2 = static_cast<float>(std::clamp((cx + half_w - pad_w) * inv_ratio, 0.0, max_w));
    const float y2 = static_cast<float>(std::clamp((cy + half_h - pad_h) * inv_ratio, 0.0, max_h));
    // クランプ後に幅または高さが 0 になる退化枠は中心や IoU が定まらないため除く。
    if (x2 <= x1 || y2 <= y1) {
      continue;
    }

    const int best_class = best_classes[a];
    Detection det;
    det.class_id = best_class;
    det.class_name = class_names[static_cast<size_t>(best_class)];
    det.confidence = max_scores[a];
    det.x1 = x1;
    det.y1 = y1;
    det.x2 = x2;
    det.y2 = y2;
    candidates.push_back(std::move(det));
  }

  if (candidates.empty()) {
    return {};
  }

  return ClassAwareNms(candidates, iou_threshold);
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
  if (out_shape.size() != 3 || out_shape[1] != kYoloNumClasses + kYoloBBoxCols) {
    throw std::runtime_error(
        "YOLO の出力形が想定と違う。[1, 84, N] を期待した");
  }
  grid_cells_ = static_cast<int>(out_shape[2]);
}

std::vector<Detection> YoloDetector::Detect(const cv::Mat& image_bgr) const {
  RequireBgr3(image_bgr);

  thread_local cv::Mat letterbox_canvas;
  thread_local cv::Mat blob_mat;
  thread_local std::vector<float> output_buf;

  static const bool profiling = std::getenv("MEISTER_VISION_PROFILE") != nullptr;
  auto mark = [](bool on) {
    return on ? std::chrono::steady_clock::now()
              : std::chrono::steady_clock::time_point{};
  };
  const auto t0 = mark(profiling);

  const LetterboxResult lb =
      LetterboxInto(image_bgr, letterbox_canvas, input_size_, kYoloPadColor);

  // Ultralytics モデルは RGB 学習のため swapRB=true を固定する。
  cv::dnn::blobFromImage(lb.padded, blob_mat, 1.0 / 255.0,
                         cv::Size(input_size_, input_size_), cv::Scalar(),
                         /*swapRB=*/true, /*crop=*/false, CV_32F);

  const auto t1 = mark(profiling);

  session_->RunInto(blob_mat.ptr<float>(), blob_mat.total(), output_buf);

  const auto t2 = mark(profiling);
  std::vector<Detection> detections =
      YoloPostprocess(output_buf, grid_cells_, lb.ratio, lb.pad_w,
                      lb.pad_h, image_bgr.cols, image_bgr.rows,
                      conf_threshold_, iou_threshold_);

  if (profiling) {
    auto ms = [](auto a, auto b) {
      return std::chrono::duration<double, std::milli>(b - a).count();
    };
    static double acc_pre = 0.0, acc_infer = 0.0, acc_post = 0.0;
    static int frames = 0;
    acc_pre += ms(t0, t1);
    acc_infer += ms(t1, t2);
    acc_post += ms(t2, std::chrono::steady_clock::now());
    if (++frames == 20) {
      std::fprintf(stderr,
                   "[vision-profile] pre %.1f ms  infer %.1f ms  post %.1f ms  "
                   "(input %d)\n",
                   acc_pre / frames, acc_infer / frames, acc_post / frames,
                   input_size_);
      acc_pre = acc_infer = acc_post = 0.0;
      frames = 0;
    }
  }
  return detections;
}

std::string ResolveYoloModelPath() {
  for (const std::string& p : YoloCandidatePaths()) {
    if (std::filesystem::exists(p)) {
      return p;
    }
  }
  return YoloCandidatePaths().back();
}

}  // namespace umeonigiri_vision

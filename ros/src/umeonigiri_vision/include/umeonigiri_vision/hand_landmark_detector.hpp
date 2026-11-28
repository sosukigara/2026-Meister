// SPDX-License-Identifier: MIT
#ifndef MEISTER_VISION_HAND_LANDMARK_DETECTOR_HPP_
#define MEISTER_VISION_HAND_LANDMARK_DETECTOR_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

namespace umeonigiri_vision {

inline constexpr char kHandModelEnvVar[] = "MEISTER_VISION_HAND_MODEL";
inline constexpr char kHandModelFile[] = "hand_landmarks.onnx";
inline constexpr int kHandDefaultInputSize = 224;
inline constexpr int kHandDefaultNumLandmarks = 21;

/// 1 隻の手の結果。landmarks は (x, y) のピクセル座標で、モデル的添字順のまま。
struct HandLandmarks {
  std::vector<cv::Point2f> landmarks;
  float confidence = 0.0f;
};

/// 入力テンソルの並べ方。session の入力 shape から決める（記憶で決めたくない）。
enum class TensorLayout { kNchw, kNhwc };

/// session の入力形から並べ方と入力辺長を求める。
/// 静的 4 次元かつチャンネル次元が 3 でなければ std::runtime_error。
TensorLayout ProbeLayout(const std::vector<int64_t>& shape, int* input_size);

/// (1, K, 3) の出力を元画像座標の HandLandmarks に変換する。
/// 手が無いときは nullopt。例外は「契約違反」だけに使う。
std::optional<HandLandmarks> DecodeHandLandmarks(
    const std::vector<float>& output, int num_landmarks, int input_size,
    double ratio, int pad_w, int pad_h, int image_width, int image_height,
    float score_threshold);

/// 手 keypoint ONNX モデルを包む検出器。
class HandLandmarkDetector {
 public:
  /// model_path が空なら ResolveHandModelPath() で解決する。
  /// 解決結果がファイル無しなら std::runtime_error。
  HandLandmarkDetector(const std::string& model_path, float score_threshold,
                       int num_landmarks, int intra_op_threads);

  /// BGR 画像から手の keypoint を推定する。手が映っていなければ nullopt。
  /// 「手なし」は正常系なので例外にしない。
  std::optional<HandLandmarks> Detect(const cv::Mat& image_bgr) const;

  const std::string& model_path() const { return model_path_; }

 private:
  std::string model_path_;
  float score_threshold_;
  int num_landmarks_;
  int input_size_ = kHandDefaultInputSize;
  TensorLayout layout_ = TensorLayout::kNchw;
  std::shared_ptr<class OnnxSession> session_;
};

/// モデルパスを 1) 環境変数 MEISTER_VISION_HAND_MODEL
/// 2) インストール先の share/umeonigiri_vision/models
/// 3) ソースツリーの models の順で解決する。どれも実在しなければ最後の
/// 候補を返す（検証は HandLandmarkDetector に任せる）。
std::string ResolveHandModelPath();

}  // namespace umeonigiri_vision

#endif  // MEISTER_VISION_HAND_LANDMARK_DETECTOR_HPP_
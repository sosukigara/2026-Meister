// SPDX-License-Identifier: MIT
/// 手の keypoint を推定する ONNX モデル。
///
/// ★要確認★ のモデル契約（一次情報を取りに行っていない）
/// -----------------------------------------------
/// 手 keypoint モデルはリポジトリに無く .gitignore（`*.onnx`）で除外されて
/// いるので、一次情報は未取得。下の形はモデル的都合から決めた契約であり、
/// 実モデルの定義と一致するかは未検証。規約を読み違えた壊れた keypoint を
/// 黙って流すくらいなら、例外で落とす方に倒す（AGENTS.md R4）。
///
///   入力  session の入力メタデータから静的 4 次元かつチャンネル次元が 3 の
///         ものを NHWC (1, S, S, 3) と NCHW (1, 3, S, S) として受け入れる。
///         S は入力辺長。動的形状や 3 チャンネルでない形はエラー。
///   出力  (1, K, 3) の float32。既定 K = 21。
///         [:, 0] = x … letterbox 済みキャンバス基準で 0.0-1.0 に正規化
///         [:, 1] = y … 同上
///         [:, 2] = score … 0.0-1.0
///
/// K 個の landmark の並び順はモデル的学習の規約に従う。21 点なら MediaPipe
/// 規約（0 手首 / 1-4 親指 / 5-8 人差し指 / 9-12 中指 / 13-16 薬指 /
/// 17-20 小指）を想定しているが、これも ★要確認★。添字順の規約を勝手に
/// 固定せず K だけ設定で変えられるようにしているのは、後から規約の違う
/// モデルへ差し替えても添字の読み違えを避けられるため。
///
/// 入力画像は BGR（OpenCV 形式）。
#ifndef MEISTER_VISION_HAND_LANDMARK_DETECTOR_HPP_
#define MEISTER_VISION_HAND_LANDMARK_DETECTOR_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

namespace meister_vision {

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
  int input_size() const { return input_size_; }
  TensorLayout layout() const { return layout_; }

 private:
  std::string model_path_;
  float score_threshold_;
  int num_landmarks_;
  int input_size_ = kHandDefaultInputSize;
  TensorLayout layout_ = TensorLayout::kNchw;
  std::shared_ptr<class OnnxSession> session_;
};

/// モデルパスを 1) 環境変数 MEISTER_VISION_HAND_MODEL
/// 2) インストール先の share/meister_vision/models
/// 3) ソースツリーの models の順で解決する。どれも実在しなければ最後の
/// 候補を返す（検証は HandLandmarkDetector に任せる）。
std::string ResolveHandModelPath();

}  // namespace meister_vision

#endif  // MEISTER_VISION_HAND_LANDMARK_DETECTOR_HPP_
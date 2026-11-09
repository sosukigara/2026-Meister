// SPDX-License-Identifier: MIT
/// YOLOv8n ONNX 物体検出器。
///
/// ONNX Runtime と YOLOv8n のモデル（COCO 80 クラス）だけを使う。torch や
/// ultralytics は入れない。onnxruntime は Python 版と同じ C API を呼ぶので、
/// 移植しても推論結果は変わらない。
///
/// 1. letterbox 前処理（アスペクト比を保ったまま 640x640 にパディング）
/// 2. ONNX 推論（入力 (1,3,640,640) float32 / 出力 (1,84,8400)）
/// 3. 出力をパース（cx,cy,w,h + 80 クラススコア）して xyxy に変換
/// 4. NMS で重複抑制
///
/// 入力画像は BGR（OpenCV 形式）。
#ifndef MEISTER_VISION_YOLO_DETECTOR_HPP_
#define MEISTER_VISION_YOLO_DETECTOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

namespace meister_vision {

/// COCO 80 クラス名。添字が class_id。
const std::vector<std::string>& CocoClasses();

/// 1 つの検出結果。xyxy は元画像座標系の [x1, y1, x2, y2]。
struct Detection {
  int class_id = 0;
  std::string class_name;
  float confidence = 0.0f;
  float x1 = 0.0f;
  float y1 = 0.0f;
  float x2 = 0.0f;
  float y2 = 0.0f;
};

/// YOLOv8 ONNX 出力の 1 予測あたりの要素数。
/// 出力形は (1, 84, N) = [4 bbox + 80 クラス] で、N は入力解像度で変わる
/// (640 なら 8400、320 なら 2100)。N はモデルの出力形から取る。
inline constexpr int kYoloBBoxCols = 4;
inline constexpr int kYoloNumClasses = 80;
/// 既定モデル (640 入力) の格子数。テストと既定値の基準。
inline constexpr int kYoloGridCells = 8400;

/// (1, 84, grid_cells) の出力を元画像座標の Detection リストに変換する。
///
/// 推論そのものは要らない。gtest からモデル無しで呼べるよう、letterbox の
/// 逆変換に必要な ratio / pad / 画像サイズを引数で受ける。
/// grid_cells は入力解像度から決まる (モデル依存) ため引数で渡す。
std::vector<Detection> YoloPostprocess(
    const std::vector<float>& output, int grid_cells, double ratio, int pad_w,
    int pad_h, int image_width, int image_height, float conf_threshold,
    float iou_threshold);

/// YOLOv8n ONNX モデルを包む検出器。
class YoloDetector {
 public:
  /// model_path が空なら ResolveYoloModelPath() で解決する。
  /// 解決結果がファイル無しなら std::runtime_error。
  YoloDetector(const std::string& model_path, float conf_threshold,
               float iou_threshold, int intra_op_threads);

  /// BGR 画像に対し検出を実行し、信頼度の高い順に Detection を返す。
  std::vector<Detection> Detect(const cv::Mat& image_bgr) const;

  const std::string& model_path() const { return model_path_; }

 private:
  std::string model_path_;
  float conf_threshold_;
  float iou_threshold_;
  /// 入力解像度 (モデルの入力形から決まる) と出力の格子数。
  int input_size_ = 640;
  int grid_cells_ = kYoloGridCells;
  std::shared_ptr<class OnnxSession> session_;
};

/// モデルパスを 1) 環境変数 MEISTER_VISION_MODEL
/// 2) インストール先の share/meister_vision/models
/// 3) ソースツリーの models の順で解決する。どれも無ければ最後の候補を返す。
/// 実在の判定は YoloDetector に任せ、解決と検証を混ぜない。
std::string ResolveYoloModelPath();

}  // namespace meister_vision

#endif  // MEISTER_VISION_YOLO_DETECTOR_HPP_
#ifndef MEISTER_VISION__RKNN_YOLO_HPP_
#define MEISTER_VISION__RKNN_YOLO_HPP_

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace umeonigiri_vision
{

/// 検出 1 件（入力画像の座標系）。
struct YoloBox
{
  float x0 = 0.0F;
  float y0 = 0.0F;
  float x1 = 0.0F;
  float y1 = 0.0F;
  float score = 0.0F;
  int class_id = 0;
};

/// Rockchip の RKNN で yolov8 系（検出ヘッドが reg/cls/sum の 3 系統 × 3 スケール）を
/// 回す薄いラッパ。NPU 出力は int8 のまま受け取り、必要なアンカーだけ逆量子化する。
class RknnYolo
{
public:
  /// モデルを読めなければ ok() が false になる（例外は投げない）。
  explicit RknnYolo(const std::string & model_path, int input_size = 640);
  ~RknnYolo();

  RknnYolo(const RknnYolo &) = delete;
  RknnYolo & operator=(const RknnYolo &) = delete;

  bool ok() const {return ok_;}
  int input_size() const {return input_size_;}
  double last_infer_ms() const {return last_infer_ms_;}

  /// image_bgr は任意サイズ。縦横比を保って入力サイズへ収め、結果を元座標で返す。
  std::vector<YoloBox> Detect(const cv::Mat & image_bgr, float conf_thres, float iou_thres);

private:
  struct OutputTensor
  {
    int index = 0;
    int channels = 0;
    int height = 0;
    int width = 0;
    float scale = 0.0F;
    int zp = 0;
  };

  bool Load(const std::string & model_path);
  bool QueryShapes();
  /// 1 スケール分をデコードする。boxes は入力座標（letterbox 済み）で返す。
  void DecodeScale(
    const OutputTensor & reg, const int8_t * reg_data,
    const OutputTensor & cls, const int8_t * cls_data,
    float logit_thres, std::vector<YoloBox> & boxes) const;

  static void Nms(std::vector<YoloBox> & boxes, float iou_thres);

  void * ctx_ = nullptr;   // rknn_context は int64 相当。ヘッダ依存を避けて隠す
  bool ok_ = false;
  int input_size_ = 640;
  int n_output_ = 0;
  std::vector<OutputTensor> outputs_;
  std::vector<int8_t> input_buffer_;
  double last_infer_ms_ = 0.0;
};

}  // namespace umeonigiri_vision

#endif  // MEISTER_VISION__RKNN_YOLO_HPP_

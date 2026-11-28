#ifndef MEISTER_VISION__RKNN_YOLO_HPP_
#define MEISTER_VISION__RKNN_YOLO_HPP_

#include <atomic>
#include <memory>
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

class RknnYolo
{
public:
  /// モデルを読めなければ ok() が false になる（例外は投げない）。
  /// core_mask: 0=AUTO, 1=CORE_0, 2=CORE_1, 4=CORE_2, 7=CORE_0_1_2 (RK3588)
  explicit RknnYolo(
    const std::string & model_path, int input_size = 640, int core_mask = 0);
  ~RknnYolo();

  RknnYolo(const RknnYolo &) = delete;
  RknnYolo & operator=(const RknnYolo &) = delete;

  bool ok() const {return ok_;}
  int input_size() const {return input_size_;}
  double last_infer_ms() const {return last_infer_ms_.load(std::memory_order_relaxed);}

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

  bool Load(const std::string & model_path, int core_mask);
  bool QueryShapes();
  /// 1 スケール分をデコードする。9 出力モデルでは score_sum で空セルを O(cells) 枝刈りする。
  void DecodeScale(
    const OutputTensor & reg, const int8_t * reg_data,
    const OutputTensor & cls, const int8_t * cls_data,
    const OutputTensor * score_sum, const int8_t * score_sum_data,
    float logit_thres, std::vector<YoloBox> & boxes) const;

  static void Nms(std::vector<YoloBox> & boxes, float iou_thres);

  void * ctx_ = nullptr;        // rknn_context は int64 相当。ヘッダ依存を避けて隠す
  void * input_mem_ = nullptr;  // rknn_tensor_mem* (DMAゼロコピー入力。非対応時は nullptr)
  bool ok_ = false;
  int input_size_ = 640;
  int n_output_ = 0;
  int branch_stride_ = 3;  // 9出力(Rockchip zoo)=3, 6出力(標準)=2
  std::vector<OutputTensor> outputs_;
  std::vector<int8_t> input_buffer_;
  std::atomic<double> last_infer_ms_{0.0};
};

/// RK3588 の 3 NPU コア（CORE_0/1/2）へ独立コンテキストを割り当て、
/// 複数ワーカーから同時推論できるようにする軽量プール。
class RknnYoloPool
{
public:
  RknnYoloPool(const std::string & model_path, int workers = 3, int input_size = 640);

  bool ok() const {return !workers_.empty();}
  size_t worker_count() const {return workers_.size();}
  int input_size() const {return workers_.empty() ? 0 : workers_.front()->input_size();}
  double last_infer_ms() const {return last_infer_ms_.load(std::memory_order_relaxed);}

  /// 指定ワーカー（呼び出しスレッド固定）で推論を実行する。
  std::vector<YoloBox> DetectOnWorker(
    size_t worker_idx, const cv::Mat & image_bgr, float conf_thres, float iou_thres);

  /// 単発呼び出し向け（ワーカー 0 を使用）。
  std::vector<YoloBox> Detect(const cv::Mat & image_bgr, float conf_thres, float iou_thres);

private:
  std::vector<std::unique_ptr<RknnYolo>> workers_;
  std::atomic<double> last_infer_ms_{0.0};
};

}  // namespace umeonigiri_vision

#endif  // MEISTER_VISION__RKNN_YOLO_HPP_

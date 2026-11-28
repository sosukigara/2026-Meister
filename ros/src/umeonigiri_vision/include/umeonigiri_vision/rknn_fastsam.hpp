#ifndef MEISTER_VISION__RKNN_FASTSAM_HPP_
#define MEISTER_VISION__RKNN_FASTSAM_HPP_

#include <string>

#include <opencv2/core.hpp>

namespace umeonigiri_vision
{

/// アーム停止時のオンデマンド把持用 FastSAM (RKNN) クラス非依存セグメンテーション。
/// COCO 80 クラス外の未知物体でも、指定 ROI（または中央）の前景マスクを 1 ショットで抽出する。
class RknnFastSam
{
public:
  explicit RknnFastSam(const std::string & model_path, int core_mask = 4);
  ~RknnFastSam();

  RknnFastSam(const RknnFastSam &) = delete;
  RknnFastSam & operator=(const RknnFastSam &) = delete;

  bool ok() const {return ok_;}
  int input_size() const {return input_size_;}

  /// image_bgr に対して 1 ショット推論し、roi（空なら画像中央）に最も重なる
  /// クラス非依存オブジェクトの二値マスク（CV_8UC1, 0/255, 元画像サイズ）を返す。
  bool SegmentBestMask(
    const cv::Mat & image_bgr, const cv::Rect & roi, cv::Mat & mask_out,
    float conf_thres = 0.35F);

private:
  bool Load(const std::string & model_path, int core_mask);
  bool QueryShapes();

  void * ctx_ = nullptr;
  bool ok_ = false;
  int input_size_ = 640;
  int n_output_ = 0;
  int proto_idx_ = -1;
  int proto_c_ = 32;
  int proto_h_ = 160;
  int proto_w_ = 160;
  float proto_scale_ = 1.0F;
  int proto_zp_ = 0;
  std::vector<int8_t> input_buffer_;
};

}  // namespace umeonigiri_vision

#endif  // MEISTER_VISION__RKNN_FASTSAM_HPP_

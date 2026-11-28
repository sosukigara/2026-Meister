#include "umeonigiri_vision/rknn_fastsam.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <rknn_api.h>

namespace umeonigiri_vision
{
namespace
{

constexpr int kPadValue = 114;

inline float Dequant(int8_t v, int zp, float scale)
{
  return (static_cast<float>(v) - static_cast<float>(zp)) * scale;
}

inline float Sigmoid(float x)
{
  return 1.0F / (1.0F + std::exp(-x));
}

}  // namespace

RknnFastSam::RknnFastSam(const std::string & model_path, int core_mask)
{
  if (model_path.empty()) {
    return;
  }
  ok_ = Load(model_path, core_mask) && QueryShapes();
}

RknnFastSam::~RknnFastSam()
{
  if (ctx_ != nullptr) {
    rknn_destroy(*static_cast<rknn_context *>(ctx_));
    delete static_cast<rknn_context *>(ctx_);
  }
}

bool RknnFastSam::Load(const std::string & model_path, int core_mask)
{
  FILE * file = std::fopen(model_path.c_str(), "rb");
  if (file == nullptr) {
    std::fprintf(stderr, "FastSAM RKNN モデルを開けません: %s\n", model_path.c_str());
    return false;
  }
  std::fseek(file, 0, SEEK_END);
  const long size = std::ftell(file);
  std::fseek(file, 0, SEEK_SET);
  std::vector<char> model(static_cast<size_t>(size));
  const size_t read = std::fread(model.data(), 1, model.size(), file);
  std::fclose(file);
  if (read != model.size()) {
    return false;
  }

  auto * ctx = new rknn_context(0);
  if (rknn_init(ctx, model.data(), model.size(), 0, nullptr) != RKNN_SUCC) {
    delete ctx;
    return false;
  }
  if (core_mask == 4) {
    rknn_set_core_mask(*ctx, RKNN_NPU_CORE_2);
  }
  ctx_ = ctx;
  return true;
}

bool RknnFastSam::QueryShapes()
{
  auto * ctx = static_cast<rknn_context *>(ctx_);
  rknn_input_output_num io_num{};
  if (rknn_query(*ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num)) != RKNN_SUCC) {
    return false;
  }
  if (io_num.n_input != 1 || io_num.n_output < 2) {
    return false;
  }
  n_output_ = static_cast<int>(io_num.n_output);

  rknn_tensor_attr in_attr{};
  in_attr.index = 0;
  if (rknn_query(*ctx, RKNN_QUERY_INPUT_ATTR, &in_attr, sizeof(in_attr)) != RKNN_SUCC) {
    return false;
  }
  input_size_ = (in_attr.fmt == RKNN_TENSOR_NCHW)
    ? static_cast<int>(in_attr.dims[2])
    : static_cast<int>(in_attr.dims[1]);
  input_buffer_.resize(static_cast<size_t>(input_size_) * input_size_ * 3);

  // YOLOv8-seg / FastSAM の最後尾プロトタイプマスク出力 [1, 32, H/4, W/4] を検出する
  for (int i = n_output_ - 1; i >= 0; --i) {
    rknn_tensor_attr out{};
    out.index = static_cast<uint32_t>(i);
    if (rknn_query(*ctx, RKNN_QUERY_OUTPUT_ATTR, &out, sizeof(out)) != RKNN_SUCC) {
      return false;
    }
    if (out.n_dims == 4 && out.dims[1] == 32 && out.dims[2] >= 40 && out.dims[3] >= 40) {
      proto_idx_ = i;
      proto_c_ = static_cast<int>(out.dims[1]);
      proto_h_ = static_cast<int>(out.dims[2]);
      proto_w_ = static_cast<int>(out.dims[3]);
      proto_scale_ = out.scale;
      proto_zp_ = out.zp;
      break;
    }
  }
  return proto_idx_ >= 0;
}

bool RknnFastSam::SegmentBestMask(
  const cv::Mat & image_bgr, const cv::Rect & roi, cv::Mat & mask_out, float conf_thres)
{
  (void)conf_thres;
  if (!ok_ || image_bgr.empty()) {
    return false;
  }

  const double scale = std::min(
    static_cast<double>(input_size_) / static_cast<double>(image_bgr.cols),
    static_cast<double>(input_size_) / static_cast<double>(image_bgr.rows));
  const int new_w = std::max(1, static_cast<int>(std::nearbyint(image_bgr.cols * scale)));
  const int new_h = std::max(1, static_cast<int>(std::nearbyint(image_bgr.rows * scale)));
  const int pad_x = (input_size_ - new_w) / 2;
  const int pad_y = (input_size_ - new_h) / 2;

  cv::Mat rgb_canvas(
    input_size_, input_size_, CV_8UC3, reinterpret_cast<uint8_t *>(input_buffer_.data()));
  rgb_canvas.setTo(cv::Scalar::all(kPadValue));
  cv::Mat resized;
  cv::resize(image_bgr, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
  cv::cvtColor(resized, rgb_canvas(cv::Rect(pad_x, pad_y, new_w, new_h)), cv::COLOR_BGR2RGB);

  auto * ctx = static_cast<rknn_context *>(ctx_);
  rknn_input input{};
  input.index = 0;
  input.type = RKNN_TENSOR_UINT8;
  input.size = static_cast<uint32_t>(input_buffer_.size());
  input.fmt = RKNN_TENSOR_NHWC;
  input.buf = input_buffer_.data();

  std::vector<rknn_output> outputs(static_cast<size_t>(n_output_));
  for (int i = 0; i < n_output_; ++i) {
    outputs[static_cast<size_t>(i)].index = static_cast<uint32_t>(i);
    outputs[static_cast<size_t>(i)].want_float = 0;
  }

  if (rknn_inputs_set(*ctx, 1, &input) != RKNN_SUCC ||
    rknn_run(*ctx, nullptr) != RKNN_SUCC ||
    rknn_outputs_get(*ctx, static_cast<uint32_t>(n_output_), outputs.data(), nullptr) != RKNN_SUCC)
  {
    return false;
  }

  // プロトタイプマスク群から ROI 中心の応答が最も強いチャネルを合成して前景マスクを生成する。
  const auto * proto = static_cast<const int8_t *>(outputs[static_cast<size_t>(proto_idx_)].buf);
  const size_t plane = static_cast<size_t>(proto_h_) * static_cast<size_t>(proto_w_);

  const cv::Rect target = roi.empty()
    ? cv::Rect(image_bgr.cols / 4, image_bgr.rows / 4, image_bgr.cols / 2, image_bgr.rows / 2)
    : (roi & cv::Rect(0, 0, image_bgr.cols, image_bgr.rows));
  const float cx_in = (target.x + 0.5F * target.width) * static_cast<float>(scale) + pad_x;
  const float cy_in = (target.y + 0.5F * target.height) * static_cast<float>(scale) + pad_y;
  const int px = std::clamp(
    static_cast<int>(cx_in * proto_w_ / static_cast<float>(input_size_)), 0, proto_w_ - 1);
  const int py = std::clamp(
    static_cast<int>(cy_in * proto_h_ / static_cast<float>(input_size_)), 0, proto_h_ - 1);
  const size_t center_idx = static_cast<size_t>(py) * proto_w_ + static_cast<size_t>(px);

  std::vector<float> weights(static_cast<size_t>(proto_c_), 0.0F);
  for (int c = 0; c < proto_c_; ++c) {
    weights[static_cast<size_t>(c)] =
      Dequant(proto[static_cast<size_t>(c) * plane + center_idx], proto_zp_, proto_scale_);
  }

  cv::Mat proto_mask(proto_h_, proto_w_, CV_8UC1);
  for (int y = 0; y < proto_h_; ++y) {
    uint8_t * row = proto_mask.ptr<uint8_t>(y);
    for (int x = 0; x < proto_w_; ++x) {
      const size_t idx = static_cast<size_t>(y) * proto_w_ + static_cast<size_t>(x);
      float acc = 0.0F;
      for (int c = 0; c < proto_c_; ++c) {
        acc += weights[static_cast<size_t>(c)] *
          Dequant(proto[static_cast<size_t>(c) * plane + idx], proto_zp_, proto_scale_);
      }
      row[x] = Sigmoid(acc) >= 0.55F ? 255 : 0;
    }
  }

  rknn_outputs_release(*ctx, static_cast<uint32_t>(n_output_), outputs.data());

  // letterbox 領域をクロップして元画像サイズへ戻す
  cv::Mat full_in;
  cv::resize(proto_mask, full_in, cv::Size(input_size_, input_size_), 0, 0, cv::INTER_NEAREST);
  const cv::Mat unpadded = full_in(cv::Rect(pad_x, pad_y, new_w, new_h));
  cv::resize(unpadded, mask_out, image_bgr.size(), 0, 0, cv::INTER_NEAREST);
  return cv::countNonZero(mask_out) > 0;
}

}  // namespace umeonigiri_vision

#include "umeonigiri_vision/rknn_yolo.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <rknn_api.h>

namespace umeonigiri_vision
{
namespace
{

constexpr int kBranchCount = 3;
constexpr int kDflBins = 16;
constexpr float kPadValue = 114.0F;

/// int8 の生値を実数へ。RKNN の量子化はテンソル単位の一様量子化。
inline float Dequant(int8_t value, int zp, float scale)
{
  return (static_cast<float>(value) - static_cast<float>(zp)) * scale;
}

}  // namespace

RknnYolo::RknnYolo(const std::string & model_path, int input_size)
: input_size_(input_size)
{
  ok_ = Load(model_path) && QueryShapes();
}

RknnYolo::~RknnYolo()
{
  if (ctx_ != nullptr) {
    rknn_destroy(*static_cast<rknn_context *>(ctx_));
    delete static_cast<rknn_context *>(ctx_);
  }
}

bool RknnYolo::Load(const std::string & model_path)
{
  FILE * file = std::fopen(model_path.c_str(), "rb");
  if (file == nullptr) {
    std::fprintf(stderr, "RKNN モデルを開けません: %s\n", model_path.c_str());
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
    std::fprintf(stderr, "rknn_init に失敗しました\n");
    delete ctx;
    return false;
  }
  ctx_ = ctx;
  return true;
}

bool RknnYolo::QueryShapes()
{
  auto * ctx = static_cast<rknn_context *>(ctx_);
  rknn_input_output_num io_num{};
  if (rknn_query(*ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num)) != RKNN_SUCC) {
    return false;
  }
  if (io_num.n_input != 1 || io_num.n_output < kBranchCount * 3) {
    std::fprintf(stderr, "想定外の入出力数です: in=%u out=%u\n", io_num.n_input, io_num.n_output);
    return false;
  }
  n_output_ = static_cast<int>(io_num.n_output);

  rknn_tensor_attr attr{};
  attr.index = 0;
  if (rknn_query(*ctx, RKNN_QUERY_INPUT_ATTR, &attr, sizeof(attr)) != RKNN_SUCC) {
    return false;
  }
  if (attr.fmt == RKNN_TENSOR_NCHW) {
    input_size_ = static_cast<int>(attr.dims[2]);
  } else {
    input_size_ = static_cast<int>(attr.dims[1]);
  }
  input_buffer_.resize(static_cast<size_t>(input_size_) * input_size_ * 3);

  outputs_.resize(static_cast<size_t>(n_output_));
  for (int i = 0; i < n_output_; ++i) {
    rknn_tensor_attr out{};
    out.index = static_cast<uint32_t>(i);
    if (rknn_query(*ctx, RKNN_QUERY_OUTPUT_ATTR, &out, sizeof(out)) != RKNN_SUCC) {
      return false;
    }
    outputs_[static_cast<size_t>(i)].index = i;
    outputs_[static_cast<size_t>(i)].channels = static_cast<int>(out.dims[1]);
    outputs_[static_cast<size_t>(i)].height = static_cast<int>(out.dims[2]);
    outputs_[static_cast<size_t>(i)].width = static_cast<int>(out.dims[3]);
    outputs_[static_cast<size_t>(i)].scale = out.scale;
    outputs_[static_cast<size_t>(i)].zp = out.zp;
  }
  return true;
}

void RknnYolo::DecodeScale(
  const OutputTensor & reg, const int8_t * reg_data,
  const OutputTensor & cls, const int8_t * cls_data,
  float logit_thres, std::vector<YoloBox> & boxes) const
{
  const int grid_h = cls.height;
  const int grid_w = cls.width;
  const int classes = cls.channels;
  const float stride_x = static_cast<float>(input_size_) / static_cast<float>(grid_w);
  const float stride_y = static_cast<float>(input_size_) / static_cast<float>(grid_h);

  // sigmoid は単調なので、しきい値は生ロジットのまま比較できる。
  const float cls_threshold_raw = logit_thres;  // ここでは確率しきい値として渡す

  for (int gy = 0; gy < grid_h; ++gy) {
    for (int gx = 0; gx < grid_w; ++gx) {
      const int base = (gy * grid_w + gx);
      // クラス方向だけ先に走査して、通らないアンカーは DFL を計算しない。
      int best = 0;
      int8_t best_raw = 0;
      bool first = true;
      for (int c = 0; c < classes; ++c) {
        const int8_t raw = cls_data[static_cast<size_t>(c) * grid_h * grid_w + base];
        if (first || raw > best_raw) {
          best_raw = raw;
          best = c;
          first = false;
        }
      }
      // このモデルの cls 出力は sigmoid 後（0〜1）を量子化したもの。
      // 生ロジット前提でしきい値を変換すると全部通ってしまう（実測で判明）。
      const float score = Dequant(best_raw, cls.zp, cls.scale);
      if (score < cls_threshold_raw) {
        continue;
      }

      // DFL: 4 辺 × 16 ビン を softmax して期待値を取る。
      float dist[4] = {0.0F, 0.0F, 0.0F, 0.0F};
      for (int side = 0; side < 4; ++side) {
        float max_v = -1e9F;
        float values[kDflBins];
        for (int bin = 0; bin < kDflBins; ++bin) {
          const int channel = side * kDflBins + bin;
          const int8_t raw = reg_data[static_cast<size_t>(channel) * grid_h * grid_w + base];
          values[bin] = Dequant(raw, reg.zp, reg.scale);
          max_v = std::max(max_v, values[bin]);
        }
        float sum = 0.0F;
        for (int bin = 0; bin < kDflBins; ++bin) {
          values[bin] = std::exp(values[bin] - max_v);
          sum += values[bin];
        }
        float expect = 0.0F;
        for (int bin = 0; bin < kDflBins; ++bin) {
          expect += values[bin] / sum * static_cast<float>(bin);
        }
        dist[side] = expect;
      }

      YoloBox box;
      box.x0 = (static_cast<float>(gx) + 0.5F - dist[0]) * stride_x;
      box.y0 = (static_cast<float>(gy) + 0.5F - dist[1]) * stride_y;
      box.x1 = (static_cast<float>(gx) + 0.5F + dist[2]) * stride_x;
      box.y1 = (static_cast<float>(gy) + 0.5F + dist[3]) * stride_y;
      box.score = score;
      box.class_id = best;
      boxes.push_back(box);
    }
  }
}

void RknnYolo::Nms(std::vector<YoloBox> & boxes, float iou_thres)
{
  std::sort(
    boxes.begin(), boxes.end(),
    [](const YoloBox & a, const YoloBox & b) {return a.score > b.score;});
  std::vector<YoloBox> kept;
  std::vector<bool> removed(boxes.size(), false);
  for (size_t i = 0; i < boxes.size(); ++i) {
    if (removed[i]) {
      continue;
    }
    kept.push_back(boxes[i]);
    for (size_t j = i + 1; j < boxes.size(); ++j) {
      if (removed[j] || boxes[j].class_id != boxes[i].class_id) {
        continue;
      }
      const float x0 = std::max(boxes[i].x0, boxes[j].x0);
      const float y0 = std::max(boxes[i].y0, boxes[j].y0);
      const float x1 = std::min(boxes[i].x1, boxes[j].x1);
      const float y1 = std::min(boxes[i].y1, boxes[j].y1);
      const float inter = std::max(0.0F, x1 - x0) * std::max(0.0F, y1 - y0);
      const float area_i = (boxes[i].x1 - boxes[i].x0) * (boxes[i].y1 - boxes[i].y0);
      const float area_j = (boxes[j].x1 - boxes[j].x0) * (boxes[j].y1 - boxes[j].y0);
      const float uni = area_i + area_j - inter;
      if (uni > 0.0F && inter / uni > iou_thres) {
        removed[j] = true;
      }
    }
  }
  boxes.swap(kept);
}

std::vector<YoloBox> RknnYolo::Detect(
  const cv::Mat & image_bgr, float conf_thres, float iou_thres)
{
  std::vector<YoloBox> result;
  if (!ok_) {
    return result;
  }
  const auto start = std::chrono::steady_clock::now();

  // 縦横比を保って入力サイズへ収める（潰すと精度が落ちる）。
  const float scale = std::min(
    static_cast<float>(input_size_) / static_cast<float>(image_bgr.cols),
    static_cast<float>(input_size_) / static_cast<float>(image_bgr.rows));
  const int new_w = static_cast<int>(static_cast<float>(image_bgr.cols) * scale);
  const int new_h = static_cast<int>(static_cast<float>(image_bgr.rows) * scale);
  const int pad_x = (input_size_ - new_w) / 2;
  const int pad_y = (input_size_ - new_h) / 2;

  cv::Mat letterboxed(input_size_, input_size_, CV_8UC3, cv::Scalar::all(kPadValue));
  cv::Mat resized;
  cv::resize(image_bgr, resized, cv::Size(new_w, new_h));
  resized.copyTo(letterboxed(cv::Rect(pad_x, pad_y, new_w, new_h)));

  cv::Mat rgb;
  cv::cvtColor(letterboxed, rgb, cv::COLOR_BGR2RGB);
  std::memcpy(input_buffer_.data(), rgb.data, input_buffer_.size());

  auto * ctx = static_cast<rknn_context *>(ctx_);
  rknn_input input{};
  input.index = 0;
  input.type = RKNN_TENSOR_UINT8;
  input.size = static_cast<uint32_t>(input_buffer_.size());
  input.fmt = RKNN_TENSOR_NHWC;
  input.buf = input_buffer_.data();
  input.pass_through = 0;

  std::vector<rknn_output> outputs(static_cast<size_t>(n_output_));
  for (int i = 0; i < n_output_; ++i) {
    outputs[static_cast<size_t>(i)].index = static_cast<uint32_t>(i);
    outputs[static_cast<size_t>(i)].want_float = 0;  // int8 のまま受け取る
    outputs[static_cast<size_t>(i)].is_prealloc = 0;
  }

  if (rknn_inputs_set(*ctx, 1, &input) != RKNN_SUCC ||
    rknn_run(*ctx, nullptr) != RKNN_SUCC ||
    rknn_outputs_get(*ctx, static_cast<uint32_t>(n_output_), outputs.data(), nullptr) != RKNN_SUCC)
  {
    return result;
  }

  // cls 出力は sigmoid 後の確率を量子化したものなので、しきい値はそのまま使う。
  const float logit_thres = conf_thres;

  for (int branch = 0; branch < kBranchCount; ++branch) {
    const OutputTensor & reg = outputs_[static_cast<size_t>(branch * 3)];
    const OutputTensor & cls = outputs_[static_cast<size_t>(branch * 3 + 1)];
    const auto * reg_data =
      static_cast<const int8_t *>(outputs[static_cast<size_t>(branch * 3)].buf);
    const auto * cls_data =
      static_cast<const int8_t *>(outputs[static_cast<size_t>(branch * 3 + 1)].buf);
    DecodeScale(reg, reg_data, cls, cls_data, logit_thres, result);
  }

  rknn_outputs_release(*ctx, static_cast<uint32_t>(n_output_), outputs.data());

  // 入力座標 → 元画像座標（パディングを外す）。
  for (auto & box : result) {
    box.x0 = (box.x0 - static_cast<float>(pad_x)) / scale;
    box.x1 = (box.x1 - static_cast<float>(pad_x)) / scale;
    box.y0 = (box.y0 - static_cast<float>(pad_y)) / scale;
    box.y1 = (box.y1 - static_cast<float>(pad_y)) / scale;
    box.x0 = std::max(0.0F, std::min(box.x0, static_cast<float>(image_bgr.cols - 1)));
    box.x1 = std::max(0.0F, std::min(box.x1, static_cast<float>(image_bgr.cols - 1)));
    box.y0 = std::max(0.0F, std::min(box.y0, static_cast<float>(image_bgr.rows - 1)));
    box.y1 = std::max(0.0F, std::min(box.y1, static_cast<float>(image_bgr.rows - 1)));
  }
  Nms(result, iou_thres);

  last_infer_ms_ = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  return result;
}

}  // namespace umeonigiri_vision

#include "umeonigiri_vision/rknn_yolo.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
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
constexpr int kPadValue = 114;

inline float Dequant(int8_t value, int zp, float scale)
{
  return (static_cast<float>(value) - static_cast<float>(zp)) * scale;
}

inline int RoundHalfToEven(double v)
{
  return static_cast<int>(std::nearbyint(v));
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
  if (grid_h <= 0 || grid_w <= 0 || classes <= 0) {
    return;
  }
  const size_t cells = static_cast<size_t>(grid_h) * static_cast<size_t>(grid_w);
  const float stride_x = static_cast<float>(input_size_) / static_cast<float>(grid_w);
  const float stride_y = static_cast<float>(input_size_) / static_cast<float>(grid_h);

  // cls.scale > 0 なら量子化は単調なので、int8 空間で 32 要素ブロック最大値を
  // 読み取り専用で走査し、閾値を超えたごく少数のセルのみ記録する。
  int8_t thres_i8 = -128;
  if (cls.scale > 0.0F) {
    const float raw_f = std::floor(logit_thres / cls.scale + static_cast<float>(cls.zp));
    if (raw_f > 127.0F) {
      return;
    }
    if (raw_f > -128.0F) {
      thres_i8 = static_cast<int8_t>(static_cast<int>(raw_f));
    }
  }

  thread_local std::vector<int8_t> max_raw;
  thread_local std::vector<uint16_t> best_cls;
  thread_local std::vector<uint8_t> active_flags;
  thread_local std::vector<size_t> active_cells;
  if (max_raw.size() < cells) {
    max_raw.resize(cells);
    best_cls.resize(cells);
    active_flags.assign(cells, 0);
  }
  active_cells.clear();

  auto record_hit = [&](size_t idx, int8_t v, uint16_t cid) {
    if (!active_flags[idx]) {
      active_flags[idx] = 1;
      active_cells.push_back(idx);
      max_raw[idx] = v;
      best_cls[idx] = cid;
    } else if (v > max_raw[idx]) {
      max_raw[idx] = v;
      best_cls[idx] = cid;
    }
  };

  for (int c = 0; c < classes; ++c) {
    const int8_t * const row = cls_data + static_cast<size_t>(c) * cells;
    const uint16_t cid = static_cast<uint16_t>(c);
    size_t base = 0;
    for (; base + 32 <= cells; base += 32) {
      int8_t blk_max = row[base];
      for (size_t k = 1; k < 32; ++k) {
        const int8_t v = row[base + k];
        blk_max = v > blk_max ? v : blk_max;
      }
      if (blk_max >= thres_i8) {
        for (size_t k = 0; k < 32; ++k) {
          const int8_t v = row[base + k];
          if (v >= thres_i8) {
            record_hit(base + k, v, cid);
          }
        }
      }
    }
    for (; base < cells; ++base) {
      const int8_t v = row[base];
      if (v >= thres_i8) {
        record_hit(base, v, cid);
      }
    }
  }

  if (active_cells.empty()) {
    return;
  }

  // DFL の int8->float 変換は高々 256 通りなので、候補が出たときだけ LUT を作る。
  bool lut_ready = false;
  std::array<float, 256> reg_lut{};

  for (const size_t base : active_cells) {
    active_flags[base] = 0;
    // このモデルの cls 出力は sigmoid 後（0〜1）を量子化したもの。
    const float score = Dequant(max_raw[base], cls.zp, cls.scale);
    if (score < logit_thres) {
      continue;
    }

    if (!lut_ready) {
      for (int i = 0; i < 256; ++i) {
        reg_lut[static_cast<size_t>(i)] =
          Dequant(static_cast<int8_t>(static_cast<uint8_t>(i)), reg.zp, reg.scale);
      }
      lut_ready = true;
    }

    float dist[4] = {0.0F, 0.0F, 0.0F, 0.0F};
    for (int side = 0; side < 4; ++side) {
      float max_v = -1e9F;
      float values[kDflBins];
      const size_t side_offset = static_cast<size_t>(side * kDflBins);
      for (int bin = 0; bin < kDflBins; ++bin) {
        const uint8_t u = static_cast<uint8_t>(
          reg_data[(side_offset + static_cast<size_t>(bin)) * cells + base]);
        values[bin] = reg_lut[u];
        max_v = std::max(max_v, values[bin]);
      }
      float sum = 0.0F;
      float weighted = 0.0F;
      for (int bin = 0; bin < kDflBins; ++bin) {
        const float e = std::exp(values[bin] - max_v);
        sum += e;
        weighted += e * static_cast<float>(bin);
      }
      dist[side] = sum > 0.0F ? (weighted / sum) : 0.0F;
    }

    const int gy = static_cast<int>(base / static_cast<size_t>(grid_w));
    const int gx = static_cast<int>(base % static_cast<size_t>(grid_w));
    YoloBox box;
    box.x0 = (static_cast<float>(gx) + 0.5F - dist[0]) * stride_x;
    box.y0 = (static_cast<float>(gy) + 0.5F - dist[1]) * stride_y;
    box.x1 = (static_cast<float>(gx) + 0.5F + dist[2]) * stride_x;
    box.y1 = (static_cast<float>(gy) + 0.5F + dist[3]) * stride_y;
    box.score = score;
    box.class_id = static_cast<int>(best_cls[base]);
    boxes.push_back(box);
  }
}

void RknnYolo::Nms(std::vector<YoloBox> & boxes, float iou_thres)
{
  if (boxes.size() <= 1) {
    return;
  }
  std::sort(
    boxes.begin(), boxes.end(),
    [](const YoloBox & a, const YoloBox & b) {return a.score > b.score;});

  const size_t n = boxes.size();
  std::vector<float> areas(n);
  for (size_t i = 0; i < n; ++i) {
    areas[i] = std::max(0.0F, boxes[i].x1 - boxes[i].x0) *
      std::max(0.0F, boxes[i].y1 - boxes[i].y0);
  }

  std::vector<YoloBox> kept;
  kept.reserve(n);
  std::vector<uint8_t> removed(n, 0);
  for (size_t i = 0; i < n; ++i) {
    if (removed[i]) {
      continue;
    }
    const YoloBox & bi = boxes[i];
    const float area_i = areas[i];
    kept.push_back(bi);
    for (size_t j = i + 1; j < n; ++j) {
      if (removed[j] || boxes[j].class_id != bi.class_id) {
        continue;
      }
      const YoloBox & bj = boxes[j];
      const float iw = std::min(bi.x1, bj.x1) - std::max(bi.x0, bj.x0);
      if (iw <= 0.0F) {
        continue;
      }
      const float ih = std::min(bi.y1, bj.y1) - std::max(bi.y0, bj.y0);
      if (ih <= 0.0F) {
        continue;
      }
      const float inter = iw * ih;
      const float uni = area_i + areas[j] - inter;
      if (uni > 0.0F && inter > iou_thres * uni) {
        removed[j] = 1;
      }
    }
  }
  boxes.swap(kept);
}

std::vector<YoloBox> RknnYolo::Detect(
  const cv::Mat & image_bgr, float conf_thres, float iou_thres)
{
  std::vector<YoloBox> result;
  if (!ok_ || image_bgr.empty()) {
    return result;
  }
  const auto start = std::chrono::steady_clock::now();

  const double scale = std::min(
    static_cast<double>(input_size_) / static_cast<double>(image_bgr.cols),
    static_cast<double>(input_size_) / static_cast<double>(image_bgr.rows));
  const int new_w = std::max(1, RoundHalfToEven(static_cast<double>(image_bgr.cols) * scale));
  const int new_h = std::max(1, RoundHalfToEven(static_cast<double>(image_bgr.rows) * scale));
  const int pad_x = (input_size_ - new_w) / 2;
  const int pad_y = (input_size_ - new_h) / 2;

  // input_buffer_ 上に直接 RGB letterbox を構築し、毎フレームの確保と全画素コピーを省く。
  cv::Mat rgb_canvas(
    input_size_, input_size_, CV_8UC3, reinterpret_cast<uint8_t *>(input_buffer_.data()));
  const cv::Scalar pad_scalar = cv::Scalar::all(kPadValue);
  const int bottom_h = input_size_ - (pad_y + new_h);
  const int right_w = input_size_ - (pad_x + new_w);
  if (pad_y > 0) {
    rgb_canvas(cv::Rect(0, 0, input_size_, pad_y)).setTo(pad_scalar);
  }
  if (bottom_h > 0) {
    rgb_canvas(cv::Rect(0, pad_y + new_h, input_size_, bottom_h)).setTo(pad_scalar);
  }
  if (pad_x > 0) {
    rgb_canvas(cv::Rect(0, pad_y, pad_x, new_h)).setTo(pad_scalar);
  }
  if (right_w > 0) {
    rgb_canvas(cv::Rect(pad_x + new_w, pad_y, right_w, new_h)).setTo(pad_scalar);
  }

  cv::Mat dst_roi = rgb_canvas(cv::Rect(pad_x, pad_y, new_w, new_h));
  if (new_w == image_bgr.cols && new_h == image_bgr.rows) {
    cv::cvtColor(image_bgr, dst_roi, cv::COLOR_BGR2RGB);
  } else {
    thread_local cv::Mat resized_bgr;
    cv::resize(image_bgr, resized_bgr, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
    cv::cvtColor(resized_bgr, dst_roi, cv::COLOR_BGR2RGB);
  }

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
    outputs[static_cast<size_t>(i)].want_float = 0;
    outputs[static_cast<size_t>(i)].is_prealloc = 0;
  }

  if (rknn_inputs_set(*ctx, 1, &input) != RKNN_SUCC ||
    rknn_run(*ctx, nullptr) != RKNN_SUCC ||
    rknn_outputs_get(*ctx, static_cast<uint32_t>(n_output_), outputs.data(), nullptr) != RKNN_SUCC)
  {
    return result;
  }

  result.reserve(64);
  for (int branch = 0; branch < kBranchCount; ++branch) {
    const OutputTensor & reg = outputs_[static_cast<size_t>(branch * 3)];
    const OutputTensor & cls = outputs_[static_cast<size_t>(branch * 3 + 1)];
    const auto * reg_data =
      static_cast<const int8_t *>(outputs[static_cast<size_t>(branch * 3)].buf);
    const auto * cls_data =
      static_cast<const int8_t *>(outputs[static_cast<size_t>(branch * 3 + 1)].buf);
    DecodeScale(reg, reg_data, cls, cls_data, conf_thres, result);
  }

  rknn_outputs_release(*ctx, static_cast<uint32_t>(n_output_), outputs.data());

  const float inv_scale = static_cast<float>(1.0 / scale);
  const float max_w = static_cast<float>(image_bgr.cols);
  const float max_h = static_cast<float>(image_bgr.rows);
  size_t write_idx = 0;
  for (size_t i = 0; i < result.size(); ++i) {
    YoloBox box = result[i];
    box.x0 = std::clamp((box.x0 - static_cast<float>(pad_x)) * inv_scale, 0.0F, max_w);
    box.x1 = std::clamp((box.x1 - static_cast<float>(pad_x)) * inv_scale, 0.0F, max_w);
    box.y0 = std::clamp((box.y0 - static_cast<float>(pad_y)) * inv_scale, 0.0F, max_h);
    box.y1 = std::clamp((box.y1 - static_cast<float>(pad_y)) * inv_scale, 0.0F, max_h);
    if (box.x1 > box.x0 && box.y1 > box.y0) {
      result[write_idx++] = box;
    }
  }
  result.resize(write_idx);

  Nms(result, iou_thres);

  last_infer_ms_ = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  return result;
}

}  // namespace umeonigiri_vision

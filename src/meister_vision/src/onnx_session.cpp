// SPDX-License-Identifier: MIT
#include "meister_vision/onnx_session.hpp"

#include <algorithm>
#include <functional>
#include <numeric>
#include <stdexcept>

#include <onnxruntime_cxx_api.h>

namespace meister_vision {
namespace {

/// 静的な 4 次元であることを確かめる。動的軸（負の値）が 1 つでもあれば
/// 前処理の形を決められないので、ここで落とす。
std::vector<int64_t> RequireStaticRank4(const std::vector<int64_t>& shape) {
  if (shape.size() != 4) {
    throw std::runtime_error("ONNX 入力は 4 次元である必要があります (rank=" +
                             std::to_string(shape.size()) + ")");
  }
  for (const int64_t d : shape) {
    if (d <= 0) {
      throw std::runtime_error("ONNX 入力は静的形でなければなりません (軸=" +
                               std::to_string(d) + ")");
    }
  }
  return shape;
}

/// Env はプロセスに 1 個だけ。Session より先に壊れてはいけないので
/// 関数内 static の共有ポインタで保持する。
std::shared_ptr<Ort::Env> SharedOrtEnv() {
  static const std::shared_ptr<Ort::Env> env =
      std::make_shared<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "meister_vision");
  return env;
}

}  // namespace

struct OnnxSession::Impl {
  std::shared_ptr<Ort::Env> env = SharedOrtEnv();
  std::unique_ptr<Ort::Session> session;
};

OnnxSession::OnnxSession(const std::string& model_path, int intra_op_threads)
    : model_path_(model_path), impl_(std::make_shared<Impl>()) {
  Ort::SessionOptions options;
  options.SetIntraOpNumThreads(std::max(1, intra_op_threads));
  options.SetInterOpNumThreads(1);

  try {
    impl_->session = std::make_unique<Ort::Session>(
        *impl_->env, model_path.c_str(), options);
  } catch (const std::exception& e) {
    throw std::runtime_error("ONNX モデルを読み込めません: " + model_path +
                             " (" + e.what() + ")");
  }

  Ort::AllocatorWithDefaultOptions allocator;
  const size_t in_count = impl_->session->GetInputCount();
  const size_t out_count = impl_->session->GetOutputCount();
  if (in_count != 1 || out_count != 1) {
    throw std::runtime_error(
        "ONNX モデルは入力 1 出力 1 である必要があります (in=" +
        std::to_string(in_count) + ", out=" + std::to_string(out_count) + ")");
  }

  const auto in_name = impl_->session->GetInputNameAllocated(0, allocator);
  const auto out_name = impl_->session->GetOutputNameAllocated(0, allocator);
  input_name_ = in_name.get();
  output_name_ = out_name.get();

  const auto type_info = impl_->session->GetInputTypeInfo(0);
  input_shape_ =
      RequireStaticRank4(type_info.GetTensorTypeAndShapeInfo().GetShape());
}

std::vector<float> OnnxSession::Run(const std::vector<float>& input) const {
  const size_t expect = static_cast<size_t>(std::accumulate(
      input_shape_.begin(), input_shape_.end(), int64_t{1},
      std::multiplies<int64_t>()));
  if (input.size() != expect) {
    throw std::runtime_error("ONNX 入力の大きさが違う (期待 " +
                             std::to_string(expect) + " 要素, 実際 " +
                             std::to_string(input.size()) + ")");
  }

  const char* input_names[] = {input_name_.c_str()};
  const char* output_names[] = {output_name_.c_str()};
  try {
    Ort::MemoryInfo mem =
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value in_tensor = Ort::Value::CreateTensor<float>(
        mem, const_cast<float*>(input.data()), input.size(),
        input_shape_.data(), input_shape_.size());
    auto outputs = impl_->session->Run(Ort::RunOptions{nullptr}, input_names,
                                       &in_tensor, 1, output_names, 1);
    const float* data = outputs.front().GetTensorData<float>();
    const size_t count =
        outputs.front().GetTensorTypeAndShapeInfo().GetElementCount();
    return std::vector<float>(data, data + count);
  } catch (const Ort::Exception& e) {
    throw std::runtime_error("ONNX 推論に失敗しました: " +
                             std::string(e.what()));
  }
}

}  // namespace meister_vision
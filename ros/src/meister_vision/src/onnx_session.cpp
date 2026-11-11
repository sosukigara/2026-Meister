// SPDX-License-Identifier: MIT
#include "meister_vision/onnx_session.hpp"

#include <algorithm>
#include <cstring>
#include <functional>
#include <numeric>
#include <stdexcept>

#include <onnxruntime_cxx_api.h>

namespace meister_vision {
namespace {

std::vector<int64_t> RequireStatic(const std::vector<int64_t>& shape) {
  for (const int64_t d : shape) {
    if (d <= 0) {
      throw std::runtime_error("ONNX の形に動的軸があります");
    }
  }
  return shape;
}

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

std::shared_ptr<Ort::Env> SharedOrtEnv() {
  static const std::shared_ptr<Ort::Env> env =
      std::make_shared<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "meister_vision");
  return env;
}

}  // namespace

struct OnnxSession::Impl {
  std::shared_ptr<Ort::Env> env = SharedOrtEnv();
  std::unique_ptr<Ort::Session> session;
  Ort::MemoryInfo mem_info =
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
};

OnnxSession::OnnxSession(const std::string& model_path, int intra_op_threads)
    : model_path_(model_path), impl_(std::make_shared<Impl>()) {
  Ort::SessionOptions options;
  // Orange Pi 5 / 5 Max (RK3588: 4x Cortex-A76 + 4x Cortex-A55) に合わせ、
  // 既定 4 スレッドで大コア群に収めクラスタ間キャッシュコヒーレンス遅延を防止
  options.SetIntraOpNumThreads(std::max(1, intra_op_threads));
  options.SetInterOpNumThreads(1);
  options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  options.EnableMemPattern();
  options.EnableCpuMemArena();
  options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);

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
  output_shape_ = RequireStatic(
      impl_->session->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape());
}

void OnnxSession::RunInto(const float* input_data, std::size_t input_count,
                          std::vector<float>& output) const {
  const size_t expect = static_cast<size_t>(std::accumulate(
      input_shape_.begin(), input_shape_.end(), int64_t{1},
      std::multiplies<int64_t>()));
  if (!input_data || input_count != expect) {
    throw std::runtime_error("ONNX 入力の大きさが違う (期待 " +
                             std::to_string(expect) + " 要素, 実際 " +
                             std::to_string(input_count) + ")");
  }

  const char* input_names[] = {input_name_.c_str()};
  const char* output_names[] = {output_name_.c_str()};
  try {
    Ort::Value in_tensor = Ort::Value::CreateTensor<float>(
        impl_->mem_info, const_cast<float*>(input_data), input_count,
        input_shape_.data(), input_shape_.size());
    auto outputs = impl_->session->Run(Ort::RunOptions{nullptr}, input_names,
                                       &in_tensor, 1, output_names, 1);
    const float* data = outputs.front().GetTensorData<float>();
    const size_t count =
        outputs.front().GetTensorTypeAndShapeInfo().GetElementCount();
    output.resize(count);
    std::memcpy(output.data(), data, count * sizeof(float));
  } catch (const Ort::Exception& e) {
    throw std::runtime_error("ONNX 推論に失敗しました: " +
                             std::string(e.what()));
  }
}

std::vector<float> OnnxSession::Run(const std::vector<float>& input) const {
  std::vector<float> out;
  RunInto(input.data(), input.size(), out);
  return out;
}

}  // namespace meister_vision

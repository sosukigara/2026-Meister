// SPDX-License-Identifier: MIT
/// ONNX Runtime の薄いラッパ（RK3588 最適化・出力バッファ再利用対応）。
///
/// Session と入出力名を生命期としてまとめ、モデルが無い・入力形が想定と違う
/// をどちらも std::runtime_error で致命扱いにする（AGENTS.md R4）。
#ifndef MEISTER_VISION_ONNX_SESSION_HPP_
#define MEISTER_VISION_ONNX_SESSION_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace meister_vision {

/// 単一入出力の ONNX モデル。
///
/// 入力は 4 次元静的テンソル、出力は 1 つに限定する。vision の 2 モデルは
/// どちらもこの形で、動的軸は無い。
class OnnxSession {
 public:
  /// モデルを読み込む。読み込み失敗は std::runtime_error。
  OnnxSession(const std::string& model_path, int intra_op_threads);

  const std::string& model_path() const { return model_path_; }
  const std::vector<int64_t>& input_shape() const { return input_shape_; }
  /// 出力の形。YOLO の格子数はここから取る (入力解像度で変わる)。
  const std::vector<int64_t>& output_shape() const { return output_shape_; }
  const std::string& input_name() const { return input_name_; }
  const std::string& output_name() const { return output_name_; }

  /// 生ポインタ入力 + 出力バッファ再利用によるゼロアロケーション推論。
  /// OpenCV の cv::Mat::ptr<float>() を直接渡すことで 4.91 MB の中間コピーを省く。
  /// input_count は input_shape() の積と一致していなければいけない。
  void RunInto(const float* input_data, std::size_t input_count,
               std::vector<float>& output) const;

  /// 入力テンソルを渡して 1 回推論し、出力を平坦化して返す（後方互換 API）。
  /// 入力元素数は input_shape() の積と一致していなければいけない。
  std::vector<float> Run(const std::vector<float>& input) const;

 private:
  struct Impl;
  std::string model_path_;
  std::vector<int64_t> input_shape_;
  std::vector<int64_t> output_shape_;
  std::string input_name_;
  std::string output_name_;
  std::shared_ptr<Impl> impl_;
};

}  // namespace meister_vision

#endif  // MEISTER_VISION_ONNX_SESSION_HPP_

// SPDX-License-Identifier: MIT
/// アスペクト比を保ったまま正方形へ収める letterbox 前処理。
///
/// 元画像に適用したスケールと、上下左右に追加したパディング幅を返す。
/// 検出結果を元画像座標へ戻すにはこの 3 つが要るので、戻り値で必ず
/// 呼び出し側へ渡す。
///
/// 最適化:
///   - LetterboxInto() を追加し、事前確保済み cv::Mat キャンバスへの直接リサイズと
///     上下または左右パディング帯のみの塗潰しを実現（毎フレームの 1.2MB 再確保と
///     中央 ROI 領域の二重書き込みを排除）。
#ifndef MEISTER_VISION_LETTERBOX_HPP_
#define MEISTER_VISION_LETTERBOX_HPP_

#include <opencv2/core/mat.hpp>

namespace meister_vision {

/// YOLOv8n の既定入力辺長。hand keypoint モデルは 224 を使う。
inline constexpr int kYoloInputSize = 640;

/// YOLO 標準のパディング色（灰色 114）。
inline constexpr int kYoloPadColor = 114;

struct LetterboxResult {
  cv::Mat padded;
  double ratio = 1.0;
  int pad_w = 0;
  int pad_h = 0;
};

/// 事前確保済み canvas を再利用して letterbox を行う（ゼロアロケーション版）。
LetterboxResult LetterboxInto(const cv::Mat& image, cv::Mat& canvas,
                              int side = kYoloInputSize,
                              int color = kYoloPadColor);

/// image を side x side に収める。縮尺が 1 で形が変わらないときは resize
/// を省く。resize を挟むと補間の丸め差だけ画像が変わり、同一入力でも
/// 同一出力にならない。
LetterboxResult Letterbox(const cv::Mat& image, int side = kYoloInputSize,
                          int color = kYoloPadColor);

}  // namespace meister_vision

#endif  // MEISTER_VISION_LETTERBOX_HPP_

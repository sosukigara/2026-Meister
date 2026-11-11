// SPDX-License-Identifier: MIT
/// 検出結果から「アームを向ける 1 個」を選ぶ純粋ロジック（α-βカルマン予測フィルタ対応）。
///
/// アーム先端カメラの視覚サーボでは、毎フレーム別の物体を掴むと制御が
/// 振動する。前フレームの対象が近くに残っている間は等速運動予測（α-βフィルタ）に
/// 基づいて追跡し続け、見失ったときだけ画像中心に最も近いものへ乗り換える。
#ifndef MEISTER_VISION_TARGET_SELECTOR_HPP_
#define MEISTER_VISION_TARGET_SELECTOR_HPP_

#include <vector>

#include "meister_vision/yolo_detector.hpp"

namespace meister_vision {

/// 画像中心へ寄せる制御に使う 1 個の対象。
struct Target {
  bool valid = false;
  Detection detection;
  /// 画像中心からの正規化オフセット。右と下が正で [-1, 1]。
  float offset_x = 0.0f;
  float offset_y = 0.0f;
  /// 前フレームから継続して追えているか。乗り換えた直後は false。
  bool locked = false;
};

struct TargetSelectorOptions {
  /// -1 ならクラス指定なし。指定時はそのクラスだけを対象にする。
  int target_class = -1;
  /// これ未満の信頼度は対象にしない。
  float min_confidence = 0.25f;
  /// 前フレームの予測中心からこの距離 [px] 以内なら同じ物体とみなす。
  float lock_radius_px = 120.0f;
  /// 見失ってから乗り換えるまでの猶予フレーム数。検出の瞬断で
  /// 別の物体へ飛ぶとアームが振動するため、数フレームは保持する。
  int lock_grace_frames = 3;
  /// α-β (Kalman) フィルタの位置更新ゲイン (1.0 = 生観測値そのまま)。
  float alpha_pos = 1.0f;
  /// α-β (Kalman) フィルタの速度更新ゲイン (0.0 = 速度予測無効、>0 で等速予測有効)。
  float beta_vel = 0.0f;
};

class TargetSelector {
 public:
  explicit TargetSelector(TargetSelectorOptions options);

  /// 候補から 1 個選ぶ。該当なしなら valid=false を返し追跡を解除する。
  Target Select(const std::vector<Detection>& detections, int image_width,
                int image_height);

  void Reset();

 private:
  bool IsCandidate(const Detection& det) const;

  TargetSelectorOptions options_;
  bool has_lock_ = false;
  int miss_count_ = 0;
  float last_center_x_ = 0.0f;
  float last_center_y_ = 0.0f;
  float vel_x_ = 0.0f;
  float vel_y_ = 0.0f;
  int last_class_ = -1;
};

}  // namespace meister_vision

#endif  // MEISTER_VISION_TARGET_SELECTOR_HPP_

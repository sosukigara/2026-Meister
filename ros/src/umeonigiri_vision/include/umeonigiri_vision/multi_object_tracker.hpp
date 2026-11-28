#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

namespace umeonigiri_vision
{

struct TrackerConfig
{
  float iou_threshold = 0.30F;      // クラス一致での対応付け最小 IoU
  int min_hits = 2;                 // 出力確定までの連続検出フレーム数（単発 FP 除去）
  int max_missed = 3;               // 見失い許容フレーム数（アーム移動中の瞬間ブレ吸収）
  float alpha = 0.65F;              // 位置・サイズ平滑化係数（大きいほど追従優先）
  float beta = 0.25F;               // 速度平滑化係数（定速度予測用）
  float high_score_thres = 0.40F;   // 1段目マッチ用の高信頼しきい値（ByteTrack 方式）
};

struct TrackInputBox
{
  float x0 = 0.0F;
  float y0 = 0.0F;
  float x1 = 0.0F;
  float y1 = 0.0F;
  float score = 0.0F;
  int class_id = 0;
};

struct TrackedBox
{
  uint64_t track_id = 0;
  float x0 = 0.0F;
  float y0 = 0.0F;
  float x1 = 0.0F;
  float y1 = 0.0F;
  float score = 0.0F;
  int class_id = 0;
  int hits = 0;
  int missed = 0;
};

/// アーム先端カメラの 25FPS 連続検出向け軽量マルチ物体トラッカ。
/// 単発の誤検出を min_hits で除去し、移動中のジッタを α-β フィルタで抑える。
class MultiObjectTracker
{
public:
  explicit MultiObjectTracker(const TrackerConfig & cfg = TrackerConfig{})
  : cfg_(cfg) {}

  void Reset()
  {
    tracks_.clear();
    next_id_ = 1;
  }

  std::vector<TrackedBox> Update(const std::vector<TrackInputBox> & detections)
  {
    // 1. 既存トラックを定速度モデルで1フレーム予測
    for (auto & tr : tracks_) {
      tr.x0 += tr.vx;
      tr.x1 += tr.vx;
      tr.y0 += tr.vy;
      tr.y1 += tr.vy;
      tr.missed += 1;
      tr.matched_this_frame = false;
    }

    std::vector<uint8_t> det_used(detections.size(), 0);

    // 2. 高信頼検出を優先して貪欲 IoU マッチング（ByteTrack 2段階方式）
    MatchStage(detections, det_used, /*high_only=*/ true);
    MatchStage(detections, det_used, /*high_only=*/ false);

    // 3. 未マッチの高信頼検出から新規トラック生成
    for (size_t i = 0; i < detections.size(); ++i) {
      if (det_used[i]) {
        continue;
      }
      const auto & d = detections[i];
      if (d.score < cfg_.high_score_thres && cfg_.min_hits > 1) {
        // 低信頼でも新規候補として開始し、min_hits 連続したら確定させる
      }
      TrackState tr{};
      tr.id = next_id_++;
      tr.x0 = d.x0;
      tr.y0 = d.y0;
      tr.x1 = d.x1;
      tr.y1 = d.y1;
      tr.score = d.score;
      tr.class_id = d.class_id;
      tr.hits = 1;
      tr.missed = 0;
      tr.matched_this_frame = true;
      tracks_.push_back(tr);
    }

    // 4. 古いトラックを破棄し、確定済みトラックをスコア順で返す
    tracks_.erase(
      std::remove_if(
        tracks_.begin(), tracks_.end(),
        [this](const TrackState & tr) {return tr.missed > cfg_.max_missed;}),
      tracks_.end());

    std::vector<TrackedBox> out;
    out.reserve(tracks_.size());
    for (const auto & tr : tracks_) {
      if (tr.hits >= cfg_.min_hits) {
        TrackedBox b{};
        b.track_id = tr.id;
        b.x0 = tr.x0;
        b.y0 = tr.y0;
        b.x1 = tr.x1;
        b.y1 = tr.y1;
        b.score = tr.score;
        b.class_id = tr.class_id;
        b.hits = tr.hits;
        b.missed = tr.missed;
        out.push_back(b);
      }
    }

    std::sort(
      out.begin(), out.end(),
      [](const TrackedBox & a, const TrackedBox & b) {return a.score > b.score;});
    return out;
  }

  static float ComputeIoU(
    float ax0, float ay0, float ax1, float ay1,
    float bx0, float by0, float bx1, float by1) noexcept
  {
    const float iw = std::min(ax1, bx1) - std::max(ax0, bx0);
    if (iw <= 0.0F) {
      return 0.0F;
    }
    const float ih = std::min(ay1, by1) - std::max(ay0, by0);
    if (ih <= 0.0F) {
      return 0.0F;
    }
    const float inter = iw * ih;
    const float area_a = std::max(0.0F, ax1 - ax0) * std::max(0.0F, ay1 - ay0);
    const float area_b = std::max(0.0F, bx1 - bx0) * std::max(0.0F, by1 - by0);
    const float uni = area_a + area_b - inter;
    return uni > 0.0F ? (inter / uni) : 0.0F;
  }

private:
  struct TrackState
  {
    uint64_t id = 0;
    float x0 = 0.0F;
    float y0 = 0.0F;
    float x1 = 0.0F;
    float y1 = 0.0F;
    float vx = 0.0F;
    float vy = 0.0F;
    float score = 0.0F;
    int class_id = 0;
    int hits = 0;
    int missed = 0;
    bool matched_this_frame = false;
  };

  void MatchStage(
    const std::vector<TrackInputBox> & detections,
    std::vector<uint8_t> & det_used,
    bool high_only)
  {
    for (size_t di = 0; di < detections.size(); ++di) {
      if (det_used[di]) {
        continue;
      }
      const auto & d = detections[di];
      if (high_only && d.score < cfg_.high_score_thres) {
        continue;
      }
      if (!high_only && d.score >= cfg_.high_score_thres) {
        continue;
      }

      int best_ti = -1;
      float best_iou = cfg_.iou_threshold;
      for (size_t ti = 0; ti < tracks_.size(); ++ti) {
        const auto & tr = tracks_[ti];
        if (tr.matched_this_frame || tr.class_id != d.class_id) {
          continue;
        }
        const float iou = ComputeIoU(tr.x0, tr.y0, tr.x1, tr.y1, d.x0, d.y0, d.x1, d.y1);
        if (iou >= best_iou) {
          best_iou = iou;
          best_ti = static_cast<int>(ti);
        }
      }

      if (best_ti >= 0) {
        auto & tr = tracks_[static_cast<size_t>(best_ti)];
        const float pred_cx = 0.5F * (tr.x0 + tr.x1);
        const float pred_cy = 0.5F * (tr.y0 + tr.y1);
        const float meas_cx = 0.5F * (d.x0 + d.x1);
        const float meas_cy = 0.5F * (d.y0 + d.y1);

        tr.vx = (1.0F - cfg_.beta) * tr.vx + cfg_.beta * (meas_cx - pred_cx);
        tr.vy = (1.0F - cfg_.beta) * tr.vy + cfg_.beta * (meas_cy - pred_cy);

        const float a = cfg_.alpha;
        tr.x0 = a * d.x0 + (1.0F - a) * tr.x0;
        tr.y0 = a * d.y0 + (1.0F - a) * tr.y0;
        tr.x1 = a * d.x1 + (1.0F - a) * tr.x1;
        tr.y1 = a * d.y1 + (1.0F - a) * tr.y1;
        tr.score = a * d.score + (1.0F - a) * tr.score;
        tr.hits += 1;
        tr.missed = 0;
        tr.matched_this_frame = true;
        det_used[di] = 1;
      }
    }
  }

  TrackerConfig cfg_;
  std::vector<TrackState> tracks_;
  uint64_t next_id_ = 1;
};

}  // namespace umeonigiri_vision

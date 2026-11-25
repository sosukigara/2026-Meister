// SPDX-License-Identifier: MIT
/// vision_msgs の検出 1 件に、クラス名と数値 ID の両方を載せる共通変換。
#ifndef MEISTER_VISION_DETECTION_MSG_HPP_
#define MEISTER_VISION_DETECTION_MSG_HPP_

#include <string>

#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

namespace umeonigiri_vision {

struct DetectionIdentity {
  int class_id = 0;
  std::string class_name;
  double score = 0.0;
  double x0 = 0.0;
  double y0 = 0.0;
  double x1 = 0.0;
  double y1 = 0.0;
};

/// id=クラス名, results[0].hypothesis.class_id=数値 ID。Web UI は id を
/// 表示ラベルとして使うので、どちらか欠けると認識モニターが unknown になる。
inline void FillDetection2D(const DetectionIdentity & identity,
                            vision_msgs::msg::Detection2D & out) {
  out.id = identity.class_name;
  out.bbox.center.position.x = (identity.x0 + identity.x1) / 2.0;
  out.bbox.center.position.y = (identity.y0 + identity.y1) / 2.0;
  out.bbox.size_x = identity.x1 - identity.x0;
  out.bbox.size_y = identity.y1 - identity.y0;
  vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
  hypothesis.hypothesis.class_id = std::to_string(identity.class_id);
  hypothesis.hypothesis.score = identity.score;
  out.results.clear();
  out.results.push_back(hypothesis);
}

}  // namespace umeonigiri_vision

#endif  // MEISTER_VISION_DETECTION_MSG_HPP_

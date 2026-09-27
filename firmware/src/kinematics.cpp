/*
 * kinematics.cpp — TwistCommand を各輪の舵角・速度へ展開する
 *
 * ★拘束ソルバ本体はまだ実装しない。★
 * 幾何係数（6 輪の配置・ボギー軸の意味）が未確定なので、正しい幾何を解いても
 * 数値が意味を持たない。代わりに「確定していない」ことを明示的に返し、
 * 指令を黙って前方適用しない。推測値で書いた値を確定値として扱うのを避けるため。
 *
 * ★呼び出し側の禁止★ 現状 Solve() / SolveInverse() は必ず
 * kGeometryNotFilled か kNoSolution を返し、DriveSetpoint はゼロ初期化される
 * だけで有効な指令を一切出さない。どこからも呼ばれていないが、
 * Phase 2 で拘束ソルバを実装するまでこの 2 関数を制御経路に
 * 繋いではならない。「動く関数」に見えるので最も危険。
 *
 * ここに実装しているのは 2 つだけ:
 *   1. vy の拒否（D5）— 車体に横方向の自由度がない
 *   2. 幾何が未確定なら kGeometryNotFilled を返す（解なしと区別する）
 *
 * 確定後に kinematics.h の RobotGeometry / RockerPosition に
 * config/meister_robot.yaml の kinematics 節の値を入れ、本体を実装する。
 */
#include "kinematics.h"

#include <math.h>

namespace meister {
namespace kin {
namespace {

/// 幾何が埋まっているか。★要確認★ のままなら false。
///
/// YAML の kinematics 節に実測値（図面または計測）が入るまで false。
/// tools/gen_config.py が kinematics.geometry_filled として出すので、
/// ここでは生成定数を見るだけにして値をコードに重複定義しない。
bool GeometryIsFilled() {
  return config::kKinematicsGeometryFilled;
}

}  // namespace

// 現在は未実装。制御経路から呼べない（ファイル冒頭を参照）。
bool TwistIsSupported(const TwistCommand& twist) {
  // 幾何が確定しているかどうかは Solve() の管轄。ここでは指令の
  // 内容だけを見る。両者を混ぜると 2 つの理由が 1 つの判定に戻る。
  return twist.vy == 0;
}

SolveStatus Solve(const TwistCommand& twist, const RobotGeometry& geo,
                  const RockerPosition* table, size_t table_size,
                  DriveSetpoint* out) {
  (void)geo;
  (void)table;
  (void)table_size;
  if (out == nullptr) {
    return SolveStatus::kGeometryNotFilled;
  }
  // 解を出す前に out を未確定な状態で残さない。row=-1 が「解なし」を表す。
  *out = DriveSetpoint{};

  if (twist.vy != 0) {
    // 黙って前方速度だけ適用すると、PC 側のモデルと実挙動が食い違い、
    // Nav2 側が「なぜ進まないか」を診断できなくなる。
    return SolveStatus::kLateralUnsupported;
  }
  if (!GeometryIsFilled()) {
    // 拘束を解く前に確定させる。推測値で解くと、機構に合う数値に
    // 見えるので誤った舵角が実際に出荷される。
    return SolveStatus::kGeometryNotFilled;
  }
  // ★TODO(Phase 2)★ 拘束ソルバ本体。
  // 1. twist の大きさと方向から rocker_positions の行を選ぶ
  // 2. 各輪の接地点速度 v_i = (vx - w*yi, vy + w*xi) を求める
  // 3. steerable な輪は v_i の方向に車輪を向ける（no-slip）
  // 4. 固定された輪は速度のみを決める
  return SolveStatus::kNoSolution;
}

SolveStatus SolveInverse(const int16_t* wheel_velocity,
                         const int16_t* wheel_steering, size_t count,
                         const RobotGeometry& geo, TwistCommand* out) {
  (void)wheel_velocity;
  (void)wheel_steering;
  (void)count;
  (void)geo;
  if (out == nullptr) {
    return SolveStatus::kGeometryNotFilled;
  }
  *out = TwistCommand{};
  if (!GeometryIsFilled()) {
    return SolveStatus::kGeometryNotFilled;
  }
  // ★TODO(Phase 2)★ 逆変換。
  // オドメトリは Nav2（PC 側）が担当する。ここが使うのは
  // スリップ検知と、PC 側のオドメトリとの整合検証だけ。
  // ESP 側に第 2 のオドメトリを作らない。
  return SolveStatus::kNoSolution;
}

}  // namespace kin
}  // namespace meister

/*
 * kinematics.h — TwistCommand を各輪の舵角・速度へ展開する
 *
 * docs/superpowers/specs/2026-09-27-command-and-kinematics-design.md の決定に基づく。
 *  - D1: 展開は ESP32 側で行う（PC は判断だけ担う）
 *  - D3: 拘束ソルバ + ボギー軸の位置表
 *
 * 幾何係数は config/meister_robot.yaml の kinematics 節から
 * tools/gen_config.py が kin_geometry.h に生成する。ここでは値を持たない。
 *
 * Arduino 非依存。pio test -e native で検証できる。
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "command.h"
#include "hal/generated_config.h"

namespace meister {
namespace kin {

/// 1 輪の幾何情報。車体座標、原点は車体中心、+x が前方、+y が左。
struct WheelGeometry {
  float x = 0.0f;  // m
  float y = 0.0f;  // m
  /// ボギー軸の位置によって舵角が変わる輪か。false なら舵角は机构が拘束する
  bool steerable = true;
  /// どのボギー軸の角度がこの輪の舵角を決めるか（steerable=false の輪は値を使わない）
  uint8_t rocker_group = 0;
};

/// 機体全体の幾何。YAML から生成する。
struct RobotGeometry {
  float wheel_radius = 0.0f;  // m
  WheelGeometry wheels[config::kNumDriveMotors];
};

/// ボギー軸を 1 組動かしたときの配置。
///
/// 拘束を解く対象はこの表の各行。どの行の制約が解けるかは機構設計で決めるので、
/// 表の定義が解の正しさの上限になる。
struct RockerPosition {
  int16_t servo_angle[2] = {0, 0};  // 前 / 後のボギー軸の指令角
  /// この行で各輪の舵角が自由になるか
  bool steerable[config::kNumDriveMotors] = {};
  /// この行が解ける運動の種別（ログと host テストの読みやすさのため）
  const char* name = "";
};

/// 展開結果。速度は千分率、舵角は 0.1 度。
struct DriveSetpoint {
  int16_t wheel_velocity[config::kNumDriveMotors] = {};
  int16_t wheel_steering[config::kNumDriveMotors] = {};
  int16_t rocker_angle[2] = {0, 0};
  /// 採用したボギー位置表の行。-1 はどれの解も無かった
  int8_t rocker_row = -1;
};

// 現状 kinematics.cpp の実装は必ず kGeometryNotFilled / kNoSolution を返す。
// 有効な指令は出ない。Phase 2 まで制御経路に繋がないこと。
/// 展開できない理由。指令側はこれで FB_ERROR を返す。
enum class SolveStatus : uint8_t {
  kOk = 0,
  kLateralUnsupported,  ///< vy != 0（横方向の自由度がない）
  kNoSolution,          ///< ボギー位置表のどの行も制約を満たさない
  kGeometryNotFilled,   ///< 幾何係数が未確定（★要確認★のまま）
};

/// TwistCommand を各輪の指令へ展開する。
///
/// 車体の 6 輪ロッカーボギーには横方向の自由度がないため、twist.vy != 0 は
/// kLateralUnsupported を返して**指令を適用しない**。黙って前方速度だけ
/// 適用すると PC 側のモデルと実挙動が食い-diffiji する。
SolveStatus Solve(const TwistCommand& twist, const RobotGeometry& geo,
                  const RockerPosition* table, size_t table_size,
                  DriveSetpoint* out);

/// Solve() の本体。幾何回が埋まったかを引数で取る。
///
/// 通常の制御経路は Solve() を使うこと（そちらが設定の
/// kinematics.geometry_filled で門を閉じる）。この関数は host テストが
/// 合成幾何でソルバの性質を検証するためだけに公開している。
SolveStatus SolveWithGeometry(const TwistCommand& twist, const RobotGeometry& geo,
                              const RockerPosition* table, size_t table_size,
                              DriveSetpoint* out, bool geometry_filled);

/// 直線速度の上限 [mm/s]。twist を千分率へ正規化する基準。
float MaxLinearMmPerSec();

/// 逆変換。各輪の実測から車体速度を推定する。
///
/// **オドメトリは Nav2（PC 側）が担当する。** ここで使うのは
/// スリップ検知と、PC 側のオドメトリとの整合検証にだけ使う。
/// ESP 側に第 2 のオドメトリを作らない。
SolveStatus SolveInverse(const int16_t* wheel_velocity,
                         const int16_t* wheel_steering, size_t count,
                         const RobotGeometry& geo, TwistCommand* out);

/// SolveInverse() の本体。host テストが合成幾何で使うためだけに公開。
SolveStatus SolveInverseWithGeometry(const int16_t* wheel_velocity,
                                     const int16_t* wheel_steering, size_t count,
                                     const RobotGeometry& geo, TwistCommand* out,
                                     bool geometry_filled);

/// 現在の姿勢で車体に垂直な横方向の速度指令が成立するか。
/// SupportsLateralMotion() が false の機械では常に false を返す。
bool TwistIsSupported(const TwistCommand& twist);

}  // namespace kin
}  // namespace meister

/*
 * kinematics.cpp — TwistCommand を各輪の舵角・速度へ展開する
 *
 * 拘束ソルバ本体。docs/superpowers/specs/2026-09-27-command-and-kinematics-design.md
 * の D1（展開は ESP32 側）/ D3（拘束ソルバ + ボギー軸の位置表）に基づく。
 *
 * ★呼び出し側の禁止★ config/meister_robot.yaml の
 * kinematics.geometry_filled が false の間、Solve() は必ず kGeometryNotFilled を
 * 返し、指令を一切出力しない。値を確定したら YAML の 1 行を true にするだけで
 * 動き出す。推測値で動かすと、comm_check の 3 手順は PASS したままで配線を誤る。
 * geometry_filled を false のままにして製造しないこと。
 */
#include "kinematics.h"

#include <math.h>

namespace meister {
namespace kin {
namespace {

/// 車体の速度ベクトルを mm/s から m/s へ。
constexpr float kMmToM = 0.001f;

/// 0.1 度単位を度へ。
constexpr float kTenthsToDeg = 0.1f;

/// 度を 0.1 度単位へ。
constexpr float kDegToTenths = 10.0f;

/// あるボギー位置の行で、1 輪の解が舵角の範囲に収まるか。
///
/// 返り値: 収まるなら true。収まらないなら false（その行では解けない）。
bool WheelWithinLimits(float heading_deg, float rocker_deg,
                        int16_t min_steer, int16_t max_steer) {
  // 舵角はボギー軸の角度を基準に取る。ボギーが 30° 傾いていれば、
  // 車輪が振れる範囲は 30°±90° になる。
  const float relative = heading_deg - rocker_deg;
  const float lo = static_cast<float>(min_steer) * kTenthsToDeg;
  const float hi = static_cast<float>(max_steer) * kTenthsToDeg;
  return relative >= lo - 0.01f && relative <= hi + 0.01f;
}

/// ボギー軸の角度を 0.1 度単位で返す。group は 0=前 / 1=後。
int16_t RockerDegForGroup(const RockerPosition& row, uint8_t group) {
  if (group < 2) {
    return row.servo_angle[group];
  }
  return 0;
}

}  // namespace

bool TwistIsSupported(const TwistCommand& twist) {
  // 幾何が確定しているかどうかは Solve() の管轄。ここでは指令の
  // 内容だけを見る。両者を混ぜると 2 つの理由が 1 つの判定に戻る。
  return twist.vy == 0;
}

float MaxLinearMmPerSec() { return config::kMaxLinearMmPerSec; }

SolveStatus Solve(const TwistCommand& twist, const RobotGeometry& geo,
                  const RockerPosition* table, size_t table_size,
                  DriveSetpoint* out) {
  return SolveWithGeometry(twist, geo, table, table_size, out,
                           config::kKinematicsGeometryFilled);
}

SolveStatus SolveWithGeometry(const TwistCommand& twist, const RobotGeometry& geo,
                              const RockerPosition* table, size_t table_size,
                              DriveSetpoint* out, bool geometry_filled) {
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
  if (!geometry_filled) {
    // 拘束を解く前に確定させる。推測値で解くと、機構に合う数値に
    // 見えるので誤った舵角が実際に出る。
    return SolveStatus::kGeometryNotFilled;
  }
  if (table == nullptr || table_size == 0) {
    return SolveStatus::kNoSolution;
  }

  const float vx = static_cast<float>(twist.vx) * kMmToM;  // m/s
  const float wz = static_cast<float>(twist.wz) * kTenthsToDeg * (float)M_PI / 180.0f;
  const float max_speed = config::kMaxLinearMmPerSec * kMmToM;
  if (max_speed <= 0.0f) {
    return SolveStatus::kNoSolution;
  }

  // 表の行は「優先順」。最初に解けた行を使う。
  for (size_t r = 0; r < table_size; ++r) {
    const RockerPosition& row = table[r];
    bool feasible = true;
    float speed[config::kNumDriveMotors] = {};
    float heading[config::kNumDriveMotors] = {};

    for (uint8_t i = 0; i < config::kNumDriveMotors && feasible; ++i) {
      const WheelGeometry& w = geo.wheels[i];
      // 車体中心から見た接地点の速度。回転が外側/内側に差を出す。
      const float vix = vx - wz * w.y;
      const float viy = wz * w.x;
      const float rocker =
          static_cast<float>(RockerDegForGroup(row, w.rocker_group)) * kTenthsToDeg;

      if (w.steerable) {
        // no-slip 条件: 車輪は接地点の速度方向にを向く。
        heading[i] = atan2f(viy, vix) * 180.0f / (float)M_PI;
        speed[i] = sqrtf(vix * vix + viy * viy);
        if (!WheelWithinLimits(heading[i], rocker, config::kMinSteering,
                               config::kMaxSteering)) {
          // 舵角の範囲外。この行では解けない。次の行を試す。
          feasible = false;
        }
      } else {
        // 舵角が固定された輪は向きを動かせない。固定方向に射影した
        // 速度だけ取出、横成分は滑る（避けられない）。
        const float rad = rocker * (float)M_PI / 180.0f;
        heading[i] = rocker;
        speed[i] = vix * cosf(rad) + viy * sinf(rad);
      }
    }
    if (!feasible) {
      continue;
    }

    for (uint8_t i = 0; i < config::kNumDriveMotors; ++i) {
      float permille = speed[i] / max_speed * 1000.0f;
      if (permille > 1000.0f) {
        permille = 1000.0f;
      }
      if (permille < -1000.0f) {
        permille = -1000.0f;
      }
      out->wheel_velocity[i] = static_cast<int16_t>(permille >= 0 ? permille + 0.5f
                                                                  : permille - 0.5f);
      float rel = heading[i] - static_cast<float>(RockerDegForGroup(row, geo.wheels[i].rocker_group)) * kTenthsToDeg;
      out->wheel_steering[i] =
          static_cast<int16_t>(rel * kDegToTenths >= 0 ? rel * kDegToTenths + 0.5f
                                                      : rel * kDegToTenths - 0.5f);
    }
    out->rocker_angle[0] = row.servo_angle[0];
    out->rocker_angle[1] = row.servo_angle[1];
    out->rocker_row = static_cast<int8_t>(r);
    return SolveStatus::kOk;
  }

  // どの行も舵角の範囲に収まらなかった。
  out->rocker_row = -1;
  return SolveStatus::kNoSolution;
}

SolveStatus SolveInverse(const int16_t* wheel_velocity,
                         const int16_t* wheel_steering, size_t count,
                         const RobotGeometry& geo, TwistCommand* out) {
  return SolveInverseWithGeometry(wheel_velocity, wheel_steering, count, geo,
                                  out, config::kKinematicsGeometryFilled);
}

SolveStatus SolveInverseWithGeometry(const int16_t* wheel_velocity,
                                     const int16_t* wheel_steering, size_t count,
                                     const RobotGeometry& geo, TwistCommand* out,
                                     bool geometry_filled) {
  if (out == nullptr || wheel_velocity == nullptr || wheel_steering == nullptr) {
    return SolveStatus::kGeometryNotFilled;
  }
  *out = TwistCommand{};
  if (!geometry_filled) {
    return SolveStatus::kGeometryNotFilled;
  }
  if (count != config::kNumDriveMotors) {
    return SolveStatus::kNoSolution;
  }

  // 最小二乗で vx と ω を求める。
  // 各輪の測定値 u_i は車体速度と回転の寄与の和:
  //   u_i = vx * c_i + ω * p_i
  //   c_i = cos(θ_i)                 車輪の向きに沿った前進の成分
  //   p_i = -y_i*c_i + x_i*sin(θ_i)   回転が車輪方向に落ちた成分
  float scc = 0.0f, scp = 0.0f, spp = 0.0f, scu = 0.0f, spu = 0.0f;
  for (uint8_t i = 0; i < config::kNumDriveMotors; ++i) {
    const WheelGeometry& w = geo.wheels[i];
    const float theta =
        static_cast<float>(wheel_steering[i]) * kTenthsToDeg * (float)M_PI / 180.0f;
    const float c = cosf(theta);
    const float p = -w.y * c + w.x * sinf(theta);
    const float u = static_cast<float>(wheel_velocity[i]);
    scc += c * c;
    scp += c * p;
    spp += p * p;
    scu += c * u;
    spu += p * u;
  }
  const float det = scc * spp - scp * scp;
  if (fabsf(det) < 1e-6f) {
    // 幾何が退化（全輪が 1 軸に乗る等）すると解が定まらない。
    return SolveStatus::kNoSolution;
  }
  const float vx = (scu * spp - spu * scp) / det;
  const float w = (scc * spu - scp * scu) / det;

  const float max_speed = config::kMaxLinearMmPerSec * kMmToM;
  if (max_speed <= 0.0f) {
    return SolveStatus::kNoSolution;
  }
  // 入力 wheel_velocity は per mille なので、mm/s へ戻す。
  const float to_mm_per_sec = config::kMaxLinearMmPerSec / 1000.0f;
  out->vx = static_cast<int16_t>(vx * to_mm_per_sec);
  out->vy = 0;
  out->wz = static_cast<int16_t>(w * 180.0f / (float)M_PI * 10.0f);
  return SolveStatus::kOk;
}

}  // namespace kin
}  // namespace meister

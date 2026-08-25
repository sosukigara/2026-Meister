"""cmd_vel (target velocity) → ステアリング角 + 駆動 PWM 変換.

ロボットは 6 輪ステアリング (ロッカーボギー) を想定。全ステアリングチャネルに
同一の舵角、全駆動チャネルに同一の速度百分率を送る単純な自転車モデル変換。

  steering_tenths = atan(wheelbase * wz / vx)   (0.1 度単位)
  vel_permille    = vx / max_linear_vel * 1000  (千分率)

注意: アッカーマン系はその場旋回できない。|vx| がほぼ 0 で |wz| がある場合は
最大舵角を切って前進 0 % とする (実機 nav 側では use_rotate_to_heading: false
を推奨)。
"""

from __future__ import annotations

import math

MIN_VELOCITY = -1000
MAX_VELOCITY = 1000

#: |vx| がこの値 (m/s) 未満のときは「ほぼ停止」とみなす
EPSILON_VX = 0.01


def twist_to_actuators(
    vx: float,
    wz: float,
    wheelbase: float,
    max_linear_vel: float,
    max_steering_deg: float,
    steer_invert: bool = False,
    drive_invert: bool = False,
) -> tuple[int, int]:
    """Twist (vx, wz) を (steering_tenths, velocity_permille) に変換する。

    steering_tenths: 6ch 分の舵角コマンド値 (0.1 度単位、正 = 左旋回)
    velocity_permille: 6ch 分の速度コマンド値 (千分率、正 = 前進)

    steer_invert / drive_invert はサーボ・モータの取り付け向きの反転用。
    """
    if max_linear_vel <= 0:
        raise ValueError("max_linear_vel must be positive")
    if max_steering_deg <= 0:
        raise ValueError("max_steering_deg must be positive")

    max_steer_tenths = int(round(max_steering_deg * 10))

    vel_permille = int(round(vx / max_linear_vel * 1000))
    vel_permille = max(MIN_VELOCITY, min(MAX_VELOCITY, vel_permille))
    if drive_invert:
        vel_permille = -vel_permille

    if abs(vx) >= EPSILON_VX:
        # 自転車モデル: tan(steer) = wheelbase * wz / vx
        steer_deg = math.degrees(math.atan2(wheelbase * wz, vx))
        steer_tenths = int(round(steer_deg * 10))
    elif wz > 0:
        steer_tenths = max_steer_tenths
    elif wz < 0:
        steer_tenths = -max_steer_tenths
    else:
        steer_tenths = 0

    steer_tenths = max(-max_steer_tenths, min(max_steer_tenths, steer_tenths))
    if steer_invert:
        steer_tenths = -steer_tenths

    return steer_tenths, vel_permille

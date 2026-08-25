"""kinematics.py のテスト: cmd_vel -> 舵角/速度変換。"""

import math

from meister_serial_bridge.kinematics import twist_to_actuators


def test_straight_forward():
    steer, vel = twist_to_actuators(0.5, 0.0, wheelbase=0.3,
                                    max_linear_vel=1.0, max_steering_deg=30.0)
    assert steer == 0
    assert vel == 500  # 0.5 m/s / 1.0 m/s


def test_left_turn_positive_wz_positive_steer():
    # vx=0.5, wz=0.5, wheelbase=0.3 -> atan(0.3) = 16.699 deg
    steer, vel = twist_to_actuators(0.5, 0.5, 0.3, 1.0, 30.0)
    expected = int(round(math.degrees(math.atan(0.3 * 0.5 / 0.5)) * 10))
    assert steer == expected
    assert steer > 0  # 左旋回 = 正
    assert vel == 500


def test_steer_clamped_to_max():
    steer, _ = twist_to_actuators(0.1, 5.0, 0.3, 1.0, 30.0)
    assert steer == 300  # ±30.0 deg


def test_near_zero_vx_full_deflection_no_drive():
    # ほぼ停止 + 旋回指示 -> 最大舵角 + 前進 0 (アッカーマンはその場旋回不可)
    steer, vel = twist_to_actuators(0.0, 1.0, 0.3, 1.0, 30.0)
    assert steer == 300
    assert vel == 0
    steer, vel = twist_to_actuators(0.0, -1.0, 0.3, 1.0, 30.0)
    assert steer == -300
    assert vel == 0


def test_stop_is_all_zero():
    steer, vel = twist_to_actuators(0.0, 0.0, 0.3, 1.0, 30.0)
    assert steer == 0
    assert vel == 0


def test_velocity_clamped():
    _, vel = twist_to_actuators(2.0, 0.0, 0.3, 1.0, 30.0)
    assert vel == 1000
    _, vel = twist_to_actuators(-2.0, 0.0, 0.3, 1.0, 30.0)
    assert vel == -1000


def test_invert_flags():
    steer, vel = twist_to_actuators(0.5, 0.0, 0.3, 1.0, 30.0,
                                    steer_invert=False, drive_invert=True)
    assert vel == -500
    steer, vel = twist_to_actuators(0.5, 0.5, 0.3, 1.0, 30.0, steer_invert=True)
    assert steer == -int(round(math.degrees(math.atan(0.3)) * 10))

#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
mecanum_controller — Twist → 4-wheel velocity converter.

Subscribes to /cmd_vel (geometry_msgs/Twist) and publishes individual wheel
angular-velocity commands to Gazebo joint topics.

Kinematics (mecanum):
    LF = Vx + Vy + ω·L    RF = Vx − Vy − ω·L
    LR = Vx + Vy − ω·L    RR = Vx − Vy + ω·L

    L = half_wheelbase + half_track  (≈ 0.25 m)
    Output is angular velocity (rad/s) = linear wheel speed / wheel_radius.

Gazebo topic pattern:
    /model/<robot_name>/joint/<joint_name>/cmd_vel
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class MecanumController(Node):
    """Convert /cmd_vel Twist to four wheel angular velocities."""

    def __init__(self):
        super().__init__('mecanum_controller')

        # ── Robot parameters ────────────────────────────────────────────
        self.declare_parameter('robot_name', 'meister_robot')
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('half_wheelbase', 0.15)   # lx
        self.declare_parameter('half_track', 0.10)       # ly
        self.declare_parameter('loop_rate', 50.0)

        robot_name: str = str(self.get_parameter('robot_name').value or 'meister_robot')
        wh_r: float = float(self.get_parameter('wheel_radius').value or 0.05)
        lx: float = float(self.get_parameter('half_wheelbase').value or 0.15)
        ly: float = float(self.get_parameter('half_track').value or 0.10)
        rate: float = float(self.get_parameter('loop_rate').value or 50.0)

        # Combined kinematic constant L = lx + ly
        self._L: float = lx + ly
        self._inv_r: float = 1.0 / wh_r if wh_r > 0 else 1.0

        self.get_logger().info(
            f'MecanumController — robot={robot_name}  r={wh_r}  '
            f'lx={lx}  ly={ly}  L={self._L:.3f}  rate={rate} Hz'
        )

        # ── Publishers (one Float64 per wheel joint) ───────────────────
        base = f'/model/{robot_name}/joint'
        self._pubs: dict[str, Publisher] = {
            'lf': self.create_publisher(Float64, f'{base}/lf_wheel_joint/cmd_vel', 10),
            'rf': self.create_publisher(Float64, f'{base}/rf_wheel_joint/cmd_vel', 10),
            'lr': self.create_publisher(Float64, f'{base}/lr_wheel_joint/cmd_vel', 10),
            'rr': self.create_publisher(Float64, f'{base}/rr_wheel_joint/cmd_vel', 10),
        }

        # ── Subscriber ─────────────────────────────────────────────────
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._sub: Subscription = self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, qos)

        # ── Timer for periodic publishing ──────────────────────────────
        self._last_cmd: Twist = Twist()  # zero by default
        self._timer: Timer = self.create_timer(1.0 / rate, self._timer_cb)

    def _cmd_cb(self, msg: Twist) -> None:
        """Cache latest velocity command."""
        self._last_cmd = msg

    def _timer_cb(self) -> None:
        """Periodically compute wheel speeds and publish."""
        cmd = self._last_cmd
        Vx = cmd.linear.x
        Vy = cmd.linear.y
        W  = cmd.angular.z

        # Mecanum inverse kinematics (linear wheel speeds in m/s)
        lf = Vx + Vy + W * self._L
        rf = Vx - Vy - W * self._L
        lr = Vx + Vy - W * self._L
        rr = Vx - Vy + W * self._L

        # Convert to angular velocities (rad/s) for Gazebo joints
        self._pubs['lf'].publish(Float64(data=lf * self._inv_r))
        self._pubs['rf'].publish(Float64(data=rf * self._inv_r))
        self._pubs['lr'].publish(Float64(data=lr * self._inv_r))
        self._pubs['rr'].publish(Float64(data=rr * self._inv_r))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MecanumController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])

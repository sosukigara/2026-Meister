#!/usr/bin/env python3
"""
fake_scan_publisher.py — Synthetic LaserScan via raycasting against maze walls.

Publishes a fake /scan at 10 Hz computed by raycasting from the robot's
odometry pose against the known maze wall layout.  Useful when the real
laser scanner is unavailable (e.g. hardware bringup or headless testing).

Subscribed topics
─────────────────
  /odom  nav_msgs/Odometry  — robot pose in the map frame

Published topics
────────────────
  /scan  sensor_msgs/LaserScan  — 360° raycast range data

Parameters
──────────
  use_sim_time  bool  — use simulation clock  (default: true)
  frame_id      str   — LaserScan frame id    (default: "laser_frame")
"""

import math
import os

import rclpy
import rclpy.duration
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


# ── Geometry helpers ────────────────────────────────────────────────────

def _box_segments(cx: float, cy: float, sx: float, sy: float,
                  yaw: float = 0.0):
    """Return list of 4 ((x1,y1),(x2,y2)) edge segments for a box.

    Args:
        cx, cy:  Centre of box (world frame).
        sx, sy:  Full extents along local x and y axes.
        yaw:     Rotation about centre (radians).
    """
    hx, hy = sx / 2.0, sy / 2.0
    # Corners in the box-local frame (centred at origin)
    local = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
    c = math.cos(yaw)
    s = math.sin(yaw)
    world = [
        (cx + x * c - y * s, cy + x * s + y * c)
        for (x, y) in local
    ]
    return [(world[i], world[(i + 1) % 4]) for i in range(4)]


def _ray_segment_intersect(ox: float, oy: float,
                           dx: float, dy: float,
                           p1, p2):
    # type: (...) -> float | None
    """Return ray parameter *t* (distance) if ray hits segment, else None.

    Ray:  O + t·D   (t > 0)
    Seg:  P1 + s·(P2 - P1)   (0 ≤ s ≤ 1)
    """
    ex = p2[0] - p1[0]
    ey = p2[1] - p1[1]
    denom = dx * ey - dy * ex
    if abs(denom) < 1e-12:
        return None  # ray and segment are parallel
    t = ((p1[0] - ox) * ey - (p1[1] - oy) * ex) / denom
    s = ((p1[0] - ox) * dy - (p1[1] - oy) * dx) / denom
    if t > 0.01 and 0.0 <= s <= 1.0:
        return t
    return None


def _quat_to_yaw(q) -> float:
    """Extract yaw (z-axis rotation) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


# ── Maze wall definitions ───────────────────────────────────────────────
# Format: (cx, cy, sx, sy, yaw)  — see _box_segments
_MAZE_WALLS = [
    # Outer boundary (20 × 20 m arena)
    (0.0, 10.0, 20.0, 0.2, 0.0),     # wall_north
    (0.0, -10.0, 20.0, 0.2, 0.0),    # wall_south
    (10.0, 0.0, 0.2, 20.0, 0.0),     # wall_east
    (-10.0, 0.0, 0.2, 20.0, 0.0),    # wall_west
    # Interior walls (axis-aligned boxes)
    (-2.5, 0.0, 15.0, 0.2, 0.0),     # h_div_left
    (-5.0, 4.5, 0.2, 11.0, 0.0),     # v_div_left
    (3.0, 5.5, 0.2, 9.0, 0.0),       # v_div_right_top
    (3.0, -5.5, 0.2, 9.0, 0.0),      # br_wall_v
    (6.5, -3.0, 7.0, 0.2, 0.0),      # br_wall_h
    (-7.5, 6.0, 3.0, 3.0, 0.0),      # tl_box
    (0.0, 3.0, 1.5, 1.5, 0.0),       # pillar_a
    (0.0, -4.0, 1.5, 1.5, 0.0),      # pillar_b
    # Diagonal obstacles (rotated)
    (6.0, 6.0, 3.0, 0.3, 0.785),     # diag_block_1
    (7.5, 4.5, 3.0, 0.3, 0.785),     # diag_block_2
]


# ── ROS2 node ───────────────────────────────────────────────────────────

class FakeScanPublisher(Node):
    """Publish synthetic LaserScan by raycasting against maze walls."""

    def __init__(self) -> None:
        super().__init__('fake_scan_publisher')

        # ── Parameters ────────────────────────────────────────────────
        # use_sim_time is typically provided by the launch file's --params-file;
        # only declare if not already declared by the launch system.
        try:
            self.declare_parameter('use_sim_time', True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass
        self.declare_parameter('frame_id', 'laser_frame')

        frame_id = self.get_parameter('frame_id').value

        # Log ROS_DOMAIN_ID (set or default)
        domain_id = os.environ.get('ROS_DOMAIN_ID', '42')
        self.get_logger().info(f'ROS_DOMAIN_ID={domain_id}')

        # ── Build wall edge segments ──────────────────────────────────
        self._segments = []
        for wall in _MAZE_WALLS:
            self._segments.extend(_box_segments(*wall))
        self.get_logger().info(
            f'Loaded {len(self._segments)} wall segments from {len(_MAZE_WALLS)} wall definitions',
        )

        # ── Scan geometry ─────────────────────────────────────────────
        self._num_samples = 360
        self._angle_min = -math.pi
        self._angle_max = math.pi
        self._angle_increment = 2.0 * math.pi / self._num_samples
        self._range_min = 0.3
        self._range_max = 12.0

        # Message skeleton (filled each tick)
        self._scan_msg = LaserScan()
        self._scan_msg.header.frame_id = frame_id
        self._scan_msg.angle_min = self._angle_min
        self._scan_msg.angle_max = self._angle_max
        self._scan_msg.angle_increment = self._angle_increment
        self._scan_msg.time_increment = 0.1 / self._num_samples
        self._scan_msg.scan_time = 0.1
        self._scan_msg.range_min = self._range_min
        self._scan_msg.range_max = self._range_max

        # Pre-computed ray direction cosines (one per sample, robot-local)
        self._ray_cos = [0.0] * self._num_samples
        self._ray_sin = [0.0] * self._num_samples
        for i in range(self._num_samples):
            theta = self._angle_min + i * self._angle_increment
            self._ray_cos[i] = math.cos(theta)
            self._ray_sin[i] = math.sin(theta)

        # ── Robot pose state (updated from /odom) ─────────────────────
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._have_odom = False

        # ── Publishers / Subscribers ──────────────────────────────────
        self._scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self._odom_sub = self.create_subscription(
            Odometry, 'odom', self._on_odom, 10,
        )

        # 10 Hz scan timer
        self._timer = self.create_timer(0.1, self._publish_scan)

        self.get_logger().info(
            f'FakeScanPublisher ready — publishing /scan at 10 Hz '
            f'({self._num_samples} rays, {self._range_min:.1f}–{self._range_max:.1f} m)',
        )

    # ── Odometry callback ──────────────────────────────────────────────

    def _on_odom(self, msg: Odometry) -> None:
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        self._robot_yaw = _quat_to_yaw(msg.pose.pose.orientation)
        self._have_odom = True

    # ── Raycasting (single ray) ────────────────────────────────────────

    def _cast_ray(self, angle: float) -> float:
        """Cast one ray at *angle* (world frame), return range in [min, max].

        Uses current stored robot pose.
        """
        dx = math.cos(angle)
        dy = math.sin(angle)
        best_t = self._range_max  # default when nothing hit

        for seg in self._segments:
            t = _ray_segment_intersect(
                self._robot_x, self._robot_y, dx, dy, seg[0], seg[1],
            )
            if t is not None and t < best_t:
                best_t = t

        return max(self._range_min, min(best_t, self._range_max))

    # ── Scan publisher (timer callback) ────────────────────────────────

    def _publish_scan(self) -> None:
        if not self._have_odom:
            # No odometry yet — any timestamp works before first pose
            self._scan_msg.header.stamp = self.get_clock().now().to_msg()
            self._scan_msg.ranges = [self._range_max] * self._num_samples
            self._scan_msg.intensities = [0.0] * self._num_samples
            self._scan_pub.publish(self._scan_msg)
            return

        # Stamp 200ms in the past so tf2_ros::MessageFilter always finds
        # a valid transform in the cache. odom_tf_broadcaster publishes
        # TF at now() — the 200ms offset gives the costmap's TF buffer
        # enough margin to not drop the scan as "newer than the cache".
        now = self.get_clock().now()
        if now.nanoseconds >= 200_000_000:
            stamp = now - rclpy.duration.Duration(seconds=0.2)
        else:
            stamp = now
        self._scan_msg.header.stamp = stamp.to_msg()

        ranges = [0.0] * self._num_samples
        for i in range(self._num_samples):
            global_angle = self._robot_yaw + self._angle_min + i * self._angle_increment
            ranges[i] = self._cast_ray(global_angle)

        self._scan_msg.ranges = ranges
        self._scan_msg.intensities = [0.0] * self._num_samples
        self._scan_pub.publish(self._scan_msg)


# ── Entry point ─────────────────────────────────────────────────────────

def main() -> None:
    rclpy.init()
    node = FakeScanPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

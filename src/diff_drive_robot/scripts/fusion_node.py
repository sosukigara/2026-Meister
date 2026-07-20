#!/usr/bin/env python3
"""
fusion_node.py — Perception → Navigation FSM bridge.

Bridges perception layer (human tracking, object detection) to navigation
layer (Nav2).  Runs a finite state machine that decides when to follow a
person, search for a lost person, pause, or let frontier exploration run.

FSM states
──────────
  IDLE       Default — no active behavior, pass-through to Nav2.
  FOLLOWING  Sends NavigateToPose goals to person's last known position.
             Goal is refreshed on each /tracked_humans update.
  PAUSED     Stops sending goals; Nav2 finishes current goal naturally.
  SEARCHING  Person lost for 5+ seconds — expanding spiral search.
  EXPLORING  Let Nav2 / mission_server handle frontier exploration.

Topics
──────
  Sub /tracked_humans      PoseArray         — tracked humans (camera_link px)
  Sub /classified_objects  Detection2DArray  — all tracked objects
  Sub /mission/state       String (JSON)     — mission_server state
  Sub /fusion/command      String (JSON)     — external commands
  Pub /fusion/state        String (JSON)     — FSM state at 1 Hz

Action client
─────────────
  NavigateToPose on /navigate_to_pose  — Nav2 goal interface
"""

import json
import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

# ── FSM state constants ────────────────────────────────────────────────────────
IDLE       = 'IDLE'
FOLLOWING  = 'FOLLOWING'
PAUSED     = 'PAUSED'
SEARCHING  = 'SEARCHING'
EXPLORING  = 'EXPLORING'

_PERSON_LOST_TIMEOUT = 5.0        # seconds before FOLLOWING → SEARCHING
_STATE_PUBLISH_RATE  = 1.0        # Hz
_FSM_TICK            = 0.5        # FSM loop interval (seconds)
_SPIRAL_PTS          = 8          # waypoints per spiral pattern
_SPIRAL_RADIUS_STEP  = 2.0        # base radius (m); expands each point


def _make_pose_stamped(
    x: float, y: float, yaw: float, stamp, frame_id: str = 'map',
) -> PoseStamped:
    """Build a PoseStamped for a NavigateToPose goal."""
    p = PoseStamped()
    p.header.frame_id = frame_id
    p.header.stamp = stamp
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    yaw_rad = math.radians(float(yaw))
    p.pose.orientation.z = math.sin(yaw_rad / 2.0)
    p.pose.orientation.w = math.cos(yaw_rad / 2.0)
    return p


def _spiral_waypoints(
    cx: float, cy: float,
    radius_step: float = _SPIRAL_RADIUS_STEP,
    count: int = _SPIRAL_PTS,
) -> list[tuple[float, float]]:
    """Generate expanding spiral search pattern centered at (cx, cy)."""
    pts: list[tuple[float, float]] = []
    for i in range(count):
        angle = i * math.pi / 4.0          # 45° increments
        radius = radius_step * (1.0 + i * 0.5)  # expand outward
        pts.append((cx + radius * math.cos(angle),
                     cy + radius * math.sin(angle)))
    return pts


# ══════════════════════════════════════════════════════════════════════════════
# Fusion node
# ══════════════════════════════════════════════════════════════════════════════

class FusionNode(Node):
    """Perception → navigation FSM bridge node."""

    def __init__(self):
        super().__init__('fusion_node')

        # ── Protected state ──────────────────────────────────────────────
        self._lock = threading.Lock()

        self._state: str = IDLE
        self._state_start_ts = time.time()

        # Person tracking
        self._target_id: Optional[int] = None
        self._last_person_ts: Optional[float] = None     # time of last /tracked_humans
        self._last_person_pose: Optional[tuple[float, float]] = None  # (x, y) ≈ bbox centre

        # Nav goal handle
        self._goal_handle = None
        self._goal_gen = 0                     # bumped each cancel → invalidates stale handles

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(PoseArray, '/tracked_humans',
                                 self._humans_cb, 10)
        self.create_subscription(Detection2DArray, '/classified_objects',
                                 self._objects_cb, 10)
        self.create_subscription(String, '/mission/state',
                                 self._mission_cb, 10)
        self.create_subscription(String, '/fusion/command',
                                 self._command_cb, 10)

        # ── Publishers ───────────────────────────────────────────────────
        self._state_pub = self.create_publisher(String, '/fusion/state', 10)
        self.create_timer(_STATE_PUBLISH_RATE, self._publish_state)

        # ── Action client ────────────────────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose,
                                        '/navigate_to_pose')

        # ── Background FSM thread ────────────────────────────────────────
        self._fsm_thread = threading.Thread(target=self._fsm_loop,
                                            daemon=True)
        self._fsm_thread.start()

        self.get_logger().info(
            'FusionNode ready — state IDLE.  Awaiting /fusion/command.')

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _humans_cb(self, msg: PoseArray):
        """Store latest tracked human position.

        PoseArray from human_tracker uses:
          position.x  = bbox centre x (pixel)
          position.y  = bbox centre y (pixel)
          position.z  = track_id (int)

        NOTE: pixel coords are not map-frame; a full implementation would
        transform through camera_info + depth + TF.  For now the FSM loop
        uses these as-is.
        """
        if not msg.poses:
            return

        # Primary target = first tracked person
        p = msg.poses[0]
        track_id = int(p.position.z)
        now = time.time()

        with self._lock:
            self._target_id = track_id
            self._last_person_pose = (p.position.x, p.position.y)
            self._last_person_ts = now

    def _objects_cb(self, msg: Detection2DArray):
        """Store classified objects for future context-aware behaviour."""
        _ = msg  # reserved for later use (e.g. obstacle-aware following)

    def _mission_cb(self, msg: String):
        """Monitor mission_server state to coordinate EXPLORING mode."""
        _ = msg  # reserved

    def _command_cb(self, msg: String):
        """External command:  {\"command\": \"follow\"|\"stop\"|\"explore\"|\"idle\"}."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Bad command JSON: {e}')
            return

        cmd = data.get('command', '').lower()
        with self._lock:
            if cmd == 'follow':
                self._transition(FOLLOWING)
                self.get_logger().info('Cmd → FOLLOWING')
            elif cmd == 'stop':
                self._cancel_goal()
                self._transition(PAUSED)
                self.get_logger().info('Cmd → PAUSED')
            elif cmd == 'explore':
                self._cancel_goal()
                self._transition(EXPLORING)
                self.get_logger().info('Cmd → EXPLORING')
            elif cmd == 'idle':
                self._cancel_goal()
                self._transition(IDLE)
                self.get_logger().info('Cmd → IDLE')
            else:
                self.get_logger().warn(f'Unknown command: {cmd!r}')

    # ── State transitions ──────────────────────────────────────────────────────

    def _transition(self, new: str):
        if self._state != new:
            old = self._state
            self._state = new
            self._state_start_ts = time.time()
            self.get_logger().info(f'FSM: {old} → {new}')

    # ── Goal helpers ───────────────────────────────────────────────────────────

    def _cancel_goal(self):
        """Cancel current Nav2 goal and bump generation counter."""
        with self._lock:
            gh = self._goal_handle
            self._goal_handle = None
            self._goal_gen += 1
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception:
                pass

    def _send_goal(self, x: float, y: float, yaw: float = 0.0) -> bool:
        """Send a NavigateToPose goal asynchronously.

        Returns True iff Nav2 accepted the goal.
        """
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 server unavailable')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = _make_pose_stamped(x, y, yaw,
                                       self.get_clock().now().to_msg())

        send_future = self._nav_client.send_goal_async(goal)
        try:
            rclpy.spin_until_future_complete(self, send_future,
                                             timeout_sec=5.0)
        except Exception:
            self.get_logger().warn('Goal accept timed out')
            return False
        if not send_future.done():
            self.get_logger().warn('Goal accept timed out')
            return False

        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            return False

        with self._lock:
            self._goal_handle = handle
        return True

    # ── FSM loop (background thread) ───────────────────────────────────────────

    def _fsm_loop(self):
        """Run every _FSM_TICK second — dispatch state actions."""
        spiral_wps: list[tuple[float, float]] = []
        spiral_idx = 0
        last_follow_goal: Optional[tuple[float, float]] = None

        while rclpy.ok():
            time.sleep(_FSM_TICK)

            with self._lock:
                state = self._state
                last_ts = self._last_person_ts
                last_pose = self._last_person_pose
                goal_gen = self._goal_gen
                now = time.time()

            # ── FOLLOWING ────────────────────────────────────────────────
            if state == FOLLOWING:
                person_lost = (last_ts is None
                               or (now - last_ts) > _PERSON_LOST_TIMEOUT)

                if person_lost:
                    self._cancel_goal()
                    with self._lock:
                        if last_pose is not None:
                            spiral_center = last_pose
                        else:
                            spiral_center = (0.0, 0.0)
                        spiral_wps = _spiral_waypoints(*spiral_center)
                        spiral_idx = 0
                        self._transition(SEARCHING)
                    last_follow_goal = None
                    self.get_logger().info(
                        'Person lost — switching to SEARCHING')
                    continue

                # Re-send goal if person has moved (each detection update)
                if last_pose is not None and last_pose != last_follow_goal:
                    self._cancel_goal()
                    ok = self._send_goal(last_pose[0], last_pose[1])
                    if ok:
                        last_follow_goal = last_pose
                        self.get_logger().info(
                            f'Follow goal → ({last_pose[0]:.1f}, '
                            f'{last_pose[1]:.1f})')
                    else:
                        self.get_logger().warn('Follow goal failed — retrying')

            # ── SEARCHING ─────────────────────────────────────────────────
            elif state == SEARCHING:
                # If person reappeared, switch back to FOLLOWING
                if last_ts is not None and (now - last_ts) <= _PERSON_LOST_TIMEOUT:
                    with self._lock:
                        self._transition(FOLLOWING)
                    spiral_wps = []
                    last_follow_goal = None
                    continue

                if not spiral_wps:
                    continue

                with self._lock:
                    has_active = self._goal_handle is not None

                # Only send next waypoint if no active goal
                if not has_active:
                    wx, wy = spiral_wps[spiral_idx % len(spiral_wps)]
                    spiral_idx += 1
                    ok = self._send_goal(wx, wy)
                    if ok:
                        self.get_logger().info(
                            f'Search waypoint {spiral_idx} → '
                            f'({wx:.1f}, {wy:.1f})')
                    else:
                        self.get_logger().warn('Search goal failed')

            # Other states (IDLE, PAUSED, EXPLORING) — no action

    # ── State publisher (timer, 1 Hz) ──────────────────────────────────────────

    def _publish_state(self):
        with self._lock:
            state = self._state
            target = self._target_id or 0
            since = round(time.time() - self._state_start_ts, 1)
            last_det = (round(time.time() - self._last_person_ts, 1)
                        if self._last_person_ts is not None else -1.0)

        payload = {
            'state': state,
            'target_id': target,
            'since_sec': since,
            'last_detection_sec': last_det,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._state_pub.publish(msg)


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

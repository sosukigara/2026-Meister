"""Owns the BasicNavigator on a single dedicated thread.

nav2_simple_commander's blocking calls (followWaypoints, isTaskComplete,
cancelTask, ...) spin the navigator node internally via
rclpy.spin_until_future_complete(self, ...). Calling them concurrently from
multiple threads on the same node is unsafe, so every navigator interaction
here happens on this one worker thread; other threads only touch the
thread-safe `status` dict and `queue`/`cancel_event`.
"""
import math
import queue
import threading

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class NavWorker(threading.Thread):

    def __init__(self):
        super().__init__(daemon=True)
        self._queue: queue.Queue = queue.Queue()
        self._cancel_event = threading.Event()
        self._status_lock = threading.Lock()
        self._status = {
            'state': 'starting',
            'nav2_ready': False,
            'waypoint_count': 0,
            'current_waypoint': None,
            'message': 'ナビゲータを初期化しています...',
        }

    def status(self) -> dict:
        with self._status_lock:
            return dict(self._status)

    def _set_status(self, **kwargs) -> None:
        with self._status_lock:
            self._status.update(kwargs)

    def submit(self, points: list[dict]) -> None:
        """Queue a list of {x, y, yaw} dicts (map frame) as a waypoint tour."""
        self._cancel_event.clear()
        self._queue.put(points)

    def cancel(self) -> None:
        self._cancel_event.set()

    def _to_pose(self, navigator: BasicNavigator, point: dict) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(point['x'])
        pose.pose.position.y = float(point['y'])
        yaw = float(point.get('yaw', 0.0))
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def run(self) -> None:
        navigator = BasicNavigator('web_nav_commander')

        self._set_status(message='Nav2の起動を待っています...')
        # This project's mapping+nav flow uses slam_toolbox for map->odom, not AMCL.
        # localizer='robot_localization' is nav2_simple_commander's special case that
        # skips the AMCL activation/initial-pose wait (there is no amcl node to wait for).
        navigator.waitUntilNav2Active(localizer='robot_localization')
        self._set_status(state='idle', nav2_ready=True, message='待機中')

        while True:
            points = self._queue.get()
            if points is None:
                break

            poses = [self._to_pose(navigator, p) for p in points]
            self._set_status(
                state='sending', waypoint_count=len(poses),
                message=f'{len(poses)} 地点へのルートを送信中...',
            )
            # followWaypoints は waypoint_follower ノードが必要だが、このスタックには
            # 含まれていない (RViz が動くのは /navigate_to_pose 直行のため)。
            # ここでも同じ /navigate_to_pose を地点ごとに順番に呼ぶ方式にする。
            self._set_status(state='navigating', message='移動中...')
            result = None
            for idx, pose in enumerate(poses):
                if self._cancel_event.is_set():
                    break
                self._set_status(
                    current_waypoint=idx,
                    message=f'地点 {idx + 1} へ移動中...',
                )
                navigator.goToPose(pose)
                while not navigator.isTaskComplete():
                    if self._cancel_event.is_set():
                        navigator.cancelTask()
                        break
                result = navigator.getResult()
                if self._cancel_event.is_set() or result != TaskResult.SUCCEEDED:
                    break

            if self._cancel_event.is_set():
                self._set_status(state='canceled', message='キャンセルしました')
            elif result == TaskResult.SUCCEEDED:
                self._set_status(state='succeeded', message='全地点に到着しました')
            else:
                self._set_status(state='failed', message='ナビゲーションに失敗しました')

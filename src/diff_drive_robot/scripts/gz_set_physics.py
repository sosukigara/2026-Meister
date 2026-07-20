#!/usr/bin/env python3
"""
gz_set_physics.py
==================
One-shot node: wait until Gazebo's /world/<name>/set_physics service is ready,
then call it with real_time_factor=1.0 so the simulation runs at wall-clock
speed.  Gazebo does not always enforce real_time_factor from the world SDF
when the scene contains few or no dynamic bodies; an explicit service call
after startup forces the physics engine to respect it.
"""

import subprocess
import time

import rclpy
from rclpy.node import Node


class GzSetPhysics(Node):
    def __init__(self) -> None:
        super().__init__('gz_set_physics')
        self.declare_parameter('world_name', 'maze')
        self._world = self.get_parameter('world_name').value
        self._service = f'/world/{self._world}/set_physics'
        self._log = self.get_logger()

    def _call_gz_service(self) -> bool:
        args = [
            'gz', 'service', '-s', self._service,
            '--reqtype', 'gz.msgs.Physics',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000',
            '--req', 'real_time_factor: 1.0  max_step_size: 0.001  profile_name: "1ms"',
        ]
        try:
            result = subprocess.run(args, capture_output=True, text=True, timeout=10)
        except subprocess.TimeoutExpired:
            return False
        ok = result.returncode == 0 and 'data: true' in result.stdout
        if not ok:
            self._log.warn(
                'set_physics returned unexpected result:\n'
                f'  stdout: {result.stdout.strip()}\n'
                f'  stderr: {result.stderr.strip()}'
            )
        return ok

    def run(self) -> None:
        """Retry the service call until it succeeds, then shut down."""
        retry_interval = 1.0  # seconds
        max_attempts = 30     # give up after ~30 s
        for attempt in range(1, max_attempts + 1):
            self._log.info(f'Attempt {attempt}/{max_attempts}: calling {self._service} …')
            if self._call_gz_service():
                self._log.info(f'{self._service} → real_time_factor=1.0 applied OK')
                return
            if attempt < max_attempts:
                time.sleep(retry_interval)
        self._log.error(
            f'Failed to call {self._service} after {max_attempts} attempts. '
            'Sim speed may not be constrained to 1×.'
        )


def main() -> None:
    rclpy.init()
    node = GzSetPhysics()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

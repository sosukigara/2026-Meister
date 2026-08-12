"""Entry point: serves the localhost web UI for sending multi-point Nav2 goals."""
import threading
from http.server import ThreadingHTTPServer
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import SingleThreadedExecutor

from meister_web_nav.http_app import make_handler
from meister_web_nav.map_listener import MapListener
from meister_web_nav.nav_worker import NavWorker

DEFAULT_PORT = 8088


def _webui_dir() -> Path:
    # Prefer the installed share/ copy; fall back to the source tree so
    # `python3 web_nav_server.py` also works without a colcon build.
    try:
        return Path(get_package_share_directory('meister_web_nav')) / 'webui'
    except Exception:
        return Path(__file__).resolve().parent.parent / 'webui'


def main(args=None):
    rclpy.init(args=args)

    map_listener = MapListener()
    # nav2_simple_commander's BasicNavigator always spins via rclpy's global
    # executor (spin_until_future_complete(self, future) with no explicit
    # executor). Give MapListener its own dedicated executor so its continuous
    # rclpy.spin() doesn't collide with the navigator's blocking calls on a
    # shared global executor from another thread ("Executor is already
    # spinning").
    map_executor = SingleThreadedExecutor()
    map_spin_thread = threading.Thread(
        target=rclpy.spin, args=(map_listener,), kwargs={'executor': map_executor},
        daemon=True,
    )
    map_spin_thread.start()

    nav_worker = NavWorker()
    nav_worker.start()

    handler_cls = make_handler(_webui_dir(), map_listener, nav_worker)
    # Bind to localhost only, matching this project's localhost-only ROS/Gazebo setup.
    server = ThreadingHTTPServer(('127.0.0.1', DEFAULT_PORT), handler_cls)

    print(f'[meister_web_nav] Web UI ready at http://localhost:{DEFAULT_PORT}')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        map_listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

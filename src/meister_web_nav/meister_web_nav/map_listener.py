"""Subscribes to /map and hands out thread-safe snapshots for the HTTP server."""
import io
import threading

import numpy as np
from nav_msgs.msg import OccupancyGrid
from PIL import Image
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                        QoSReliabilityPolicy)

MAP_QOS = QoSProfile(
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class MapListener(Node):
    """Keeps the latest /map OccupancyGrid and renders it to PNG on demand."""

    def __init__(self):
        super().__init__('web_nav_map_listener')
        self._lock = threading.Lock()
        self._grid: OccupancyGrid | None = None
        self.create_subscription(OccupancyGrid, '/map', self._on_map, MAP_QOS)

    def _on_map(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._grid = msg

    def metadata(self) -> dict | None:
        with self._lock:
            grid = self._grid
        if grid is None:
            return None
        info = grid.info
        return {
            'resolution': info.resolution,
            'width': info.width,
            'height': info.height,
            'origin': {
                'x': info.origin.position.x,
                'y': info.origin.position.y,
            },
        }

    def render_png(self) -> bytes | None:
        """Render the occupancy grid as a PNG (free=white, occupied=black, unknown=gray)."""
        with self._lock:
            grid = self._grid
        if grid is None:
            return None

        width, height = grid.info.width, grid.info.height
        cells = np.array(grid.data, dtype=np.int16).reshape((height, width))

        gray = np.where(cells < 0, 205, 255 - (cells.astype(np.float32) / 100.0) * 255)
        gray = gray.astype(np.uint8)
        # OccupancyGrid row 0 is the bottom of the map; image row 0 is the top.
        gray = np.flipud(gray)

        buf = io.BytesIO()
        Image.fromarray(gray, mode='L').save(buf, format='PNG')
        return buf.getvalue()

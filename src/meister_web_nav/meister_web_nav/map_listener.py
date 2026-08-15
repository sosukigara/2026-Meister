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

    @staticmethod
    def _crop_bounds(grid: OccupancyGrid) -> tuple | None:
        """探索済み(既知)セルの bounding box を (r0, r1, c0, c1) で返す。

        OccupancyGrid の行 0 は地図の下端 (y=origin.y)。ロボットの走行跡が
        ワールド外に伸びると地図が巨大化して Web 表示が偏るため、既知領域
        (自由/障害物セル) にのみクロップして表示する。
        """
        height, width = grid.info.height, grid.info.width
        if width == 0 or height == 0:
            return None
        cells = np.array(grid.data, dtype=np.int16).reshape((height, width))
        known = cells >= 0
        if not known.any():
            return None
        rows, cols = np.nonzero(known)
        r0, r1 = int(rows.min()), int(rows.max()) + 1
        c0, c1 = int(cols.min()), int(cols.max()) + 1
        # 端のマーカーが欠けないよう 5% の余白を取る
        pad = max(2, int(0.05 * min(r1 - r0, c1 - c0)))
        r0 = max(0, r0 - pad)
        r1 = min(height, r1 + pad)
        c0 = max(0, c0 - pad)
        c1 = min(width, c1 + pad)
        return r0, r1, c0, c1

    def metadata(self) -> dict | None:
        with self._lock:
            grid = self._grid
        if grid is None:
            return None
        info = grid.info
        bounds = self._crop_bounds(grid)
        if bounds is None:
            width, height = info.width, info.height
            ox, oy = info.origin.position.x, info.origin.position.y
        else:
            r0, r1, c0, c1 = bounds
            res = info.resolution
            width, height = c1 - c0, r1 - r0
            ox = info.origin.position.x + c0 * res
            oy = info.origin.position.y + r0 * res
        return {
            'resolution': info.resolution,
            'width': width,
            'height': height,
            'origin': {
                'x': ox,
                'y': oy,
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

        bounds = self._crop_bounds(grid)
        if bounds is not None:
            r0, r1, c0, c1 = bounds
            cells = cells[r0:r1, c0:c1]

        gray = np.where(cells < 0, 205, 255 - (cells.astype(np.float32) / 100.0) * 255)
        gray = gray.astype(np.uint8)
        # OccupancyGrid row 0 is the bottom of the map; image row 0 is the top.
        gray = np.flipud(gray)

        buf = io.BytesIO()
        Image.fromarray(gray, mode='L').save(buf, format='PNG')
        return buf.getvalue()

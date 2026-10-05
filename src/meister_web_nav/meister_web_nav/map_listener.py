"""Subscribes to /map and hands out thread-safe snapshots for the HTTP server."""
import math
import threading

from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                        QoSReliabilityPolicy)
from tf2_msgs.msg import TFMessage

from meister_web_nav.map_cache import MapCache

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
        self._lock = threading.Lock()  # tf スナップショットのみ。地図は MapCache が持つ
        self._map_odom = None  # latest /tf map->odom (TransformStamped)
        self._odom_base = None  # latest /tf odom->base_footprint (TransformStamped)
        self._maps = MapCache()
        self.create_subscription(OccupancyGrid, '/map', self._on_map, MAP_QOS)
        self.create_subscription(TFMessage, '/tf', self._on_tf, 10)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._maps.set_grid(msg)

    def _on_tf(self, msg: TFMessage) -> None:
        """ロボット位置オーバーレイ用に map->odom / odom->base を追跡する。"""
        for t in msg.transforms:
            if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                with self._lock:
                    self._map_odom = t
            elif t.header.frame_id == 'odom' and t.child_frame_id in (
                    'base_footprint', 'base_link'):
                with self._lock:
                    self._odom_base = t

    def robot_pose(self) -> dict | None:
        """ロボットの地図座標系での位置 {x, y, yaw} を返す (tf 未受信なら None)。"""
        with self._lock:
            mo, ob = self._map_odom, self._odom_base
        if mo is None or ob is None:
            return None
        yaw_mo = 2.0 * math.atan2(mo.transform.rotation.z, mo.transform.rotation.w)
        yaw_ob = 2.0 * math.atan2(ob.transform.rotation.z, ob.transform.rotation.w)
        tx, ty = ob.transform.translation.x, ob.transform.translation.y
        # map→base_footprint = map→odom ∘ odom→base_footprint (平面 2D)
        x = mo.transform.translation.x + math.cos(yaw_mo) * tx - math.sin(yaw_mo) * ty
        y = mo.transform.translation.y + math.sin(yaw_mo) * tx + math.cos(yaw_mo) * ty
        return {'x': x, 'y': y, 'yaw': yaw_mo + yaw_ob}

    def metadata(self) -> dict | None:
        grid, _version, bounds = self._maps.snapshot()
        if grid is None:
            return None
        info = grid.info
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

    def render_png(self) -> tuple[bytes, str] | None:
        """(PNG bytes, version) を返す (地図未着なら None)。version は引用符無しの content hash。

        同じ version なら PIL encode を再利用するので、SLAM 中も地図が変わらない
        あいだは 5 秒ごとのポーリングがほぼ 0 コストになる。
        """
        return self._maps.render_png()

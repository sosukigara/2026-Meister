"""Latest-grid store that memoizes the crop bounds and the PNG per map version.

rclpy を意図的に import しない (AGENTS.md R8)。HTTP Facing のキャッシュは
ROS グラフ無しで単体テストできる必要がある。

この層を分けた理由: Web UI は /api/map.png を 5 秒ごとに取り直し、1200x1200 の
地図では crop + PIL encode が 1 サイクル約 370 ms かかっていた。ThreadingHTTPServer
は ROS 配信スレッドと GIL を奪い合うので、encode は地図が変わった時だけ paying する。
"""
import hashlib
import io
import threading
from typing import TYPE_CHECKING

import numpy as np
from PIL import Image

if TYPE_CHECKING:  # 型注釈のみ。rclpy 無しでも import できるようにする
    from nav_msgs.msg import OccupancyGrid


def _version_of(grid: 'OccupancyGrid') -> str:
    """セルの内容ハッシュを version (entity-tag の中身) にする。

    メッセージ|Sport の連番ではなく内容でハッシュする: 同じ地図を再送する
    publisher が versions を進めてしまうと、If-None-Match が一致せず毎回
    200 が返って 304 の効果が消える。
    """
    h = hashlib.blake2b(digest_size=8)
    h.update(bytes(grid.data))
    info = grid.info
    o = info.origin
    h.update(repr((info.width, info.height, info.resolution,
                   o.position.x, o.position.y,
                   o.orientation.x, o.orientation.y,
                   o.orientation.z, o.orientation.w)).encode('utf-8'))
    return h.hexdigest()


class MapCache:
    """最新の grid を保持し、crop bounds と PNG を version ごとに 1 回だけ計算する。"""

    def __init__(self) -> None:
        # state_lock は grid ポインタの差し替えだけ、memo_lock は重い計算の直列化。
        # 取得順は常に memo_lock -> state_lock に固定する。
        self._state_lock = threading.Lock()
        self._memo_lock = threading.Lock()
        self._grid: 'OccupancyGrid | None' = None
        self._version = ''
        self._bounds = None
        self._bounds_version = None
        self._png: bytes | None = None
        self._png_version = None

    def set_grid(self, grid: 'OccupancyGrid') -> None:
        with self._state_lock:
            self._grid = grid
            self._version = _version_of(grid)

    @staticmethod
    def _crop_bounds(grid: 'OccupancyGrid') -> tuple | None:
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

    def snapshot(self) -> tuple:
        """(grid, version, bounds) を返す。bounds は version ごとに 1 回だけ計算する。"""
        with self._memo_lock:
            with self._state_lock:
                grid, version = self._grid, self._version
            if grid is None:
                return None, None, None
            if version != self._bounds_version:
                self._bounds = self._crop_bounds(grid)
                self._bounds_version = version
            return grid, version, self._bounds

    @staticmethod
    def _encode_png(grid: 'OccupancyGrid', bounds: tuple | None) -> bytes:
        width, height = grid.info.width, grid.info.height
        cells = np.array(grid.data, dtype=np.int16).reshape((height, width))
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

    def render_png(self) -> tuple[bytes, str] | None:
        """(png, version) を返す。version は引用符無しの content hash。地図が未着なら None。

        bytes と ETag を 1 つのスナップショットから返す。別々に取得すると、地図が
        更新された隙に「新しい bytes に古い ETag」を貼る race になる。
        """
        grid, version, bounds = self.snapshot()
        if grid is None:
            return None
        with self._memo_lock:
            if version != self._png_version:
                self._png = self._encode_png(grid, bounds)
                self._png_version = version
            return self._png, version

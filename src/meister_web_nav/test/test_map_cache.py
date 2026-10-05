"""MapCache の単体テスト (ROS グラフ不要)。

- (c) _crop_bounds が version ごとに高々 1 回しか呼ばれないことを、呼び出し
    回数で検証する。時間では測らない (CI 負荷で結果が変わるため)
- 地図が変わっていなければ PIL encode も再実行されない
- 地図が変われば ETag (version) が変わる
"""
import array
import io
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pytest  # noqa: E402
from PIL import Image  # noqa: E402

from meister_web_nav.map_cache import MapCache  # noqa: E402

from conftest import make_grid  # noqa: E402


@pytest.fixture
def counted_cache(monkeypatch):
    """_crop_bounds と _encode_png の呼び出し回数を数える MapCache を返す。"""
    calls = {'crop': 0, 'encode': 0}
    real_crop = MapCache._crop_bounds
    real_encode = MapCache._encode_png

    def counting_crop(grid):
        calls['crop'] += 1
        return real_crop(grid)

    def counting_encode(grid, bounds):
        calls['encode'] += 1
        return real_encode(grid, bounds)

    monkeypatch.setattr(MapCache, '_crop_bounds', staticmethod(counting_crop))
    monkeypatch.setattr(MapCache, '_encode_png', staticmethod(counting_encode))
    return MapCache(), calls


class TestCropBoundsCalledOncePerVersion:
    def test_repeated_polls_crop_once(self, counted_cache):
        """同じ地図を何度読んでも _crop_bounds は 1 回だけ。"""
        cache, calls = counted_cache
        cache.set_grid(make_grid())
        for _ in range(10):
            cache.snapshot()
            cache.render_png()
        assert calls['crop'] == 1
        assert calls['encode'] == 1

    def test_metadata_and_render_share_one_crop(self, counted_cache):
        """旧実装は 1 ポーリングで 2 回 crop していた (/api/map と /api/map.png)。"""
        cache, calls = counted_cache
        cache.set_grid(make_grid())
        cache.snapshot()
        cache.render_png()
        assert calls['crop'] == 1

    def test_new_version_crops_again(self, counted_cache):
        """地図が変われば次の version で 1 回だけ再計算する。"""
        cache, calls = counted_cache
        cache.set_grid(make_grid())
        cache.render_png()
        assert calls['crop'] == 1

        cache.set_grid(make_grid(wall_row=14))
        cache.render_png()
        assert calls['crop'] == 2
        assert calls['encode'] == 2

    def test_identical_content_keeps_version(self, counted_cache):
        """内容が同じなら version が変わらない (ETag を安定させるため)。"""
        cache, calls = counted_cache
        cache.set_grid(make_grid())
        _grid, version_a, _b = cache.snapshot()
        cache.set_grid(make_grid())  # 別インスタンスだが内容は同一
        _grid, version_b, _b = cache.snapshot()
        assert version_a == version_b
        assert calls['crop'] == 1

    def test_concurrent_polls_crop_once(self, counted_cache):
        """複数スレッドが同時に読んでも crop も encode も 1 回だけ。"""
        cache, calls = counted_cache
        cache.set_grid(make_grid(size=64))
        barrier = threading.Barrier(8)

        def worker():
            barrier.wait()
            for _ in range(5):
                cache.render_png()

        threads = [threading.Thread(target=worker) for _ in range(8)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        assert calls['crop'] == 1
        assert calls['encode'] == 1


class TestRenderPng:
    def test_png_is_stable_for_same_version(self):
        cache = MapCache()
        cache.set_grid(make_grid())
        first = cache.render_png()
        second = cache.render_png()
        assert first is not None
        assert first[0] == second[0]
        assert first[1] == second[1]
        assert first[0].startswith(b'\x89PNG')

    def test_etag_changes_with_content(self):
        cache = MapCache()
        cache.set_grid(make_grid())
        _png_a, etag_a = cache.render_png()
        cache.set_grid(make_grid(wall_row=20))
        _png_b, etag_b = cache.render_png()
        assert etag_a != etag_b

    def test_etag_changes_with_resolution(self):
        """セルの色が変わらなくても resolution が変われば PNG は別物。"""
        cache = MapCache()
        cache.set_grid(make_grid())
        _png_a, etag_a = cache.render_png()
        moved = make_grid()
        moved.info.resolution = 0.02
        cache.set_grid(moved)
        _png_b, etag_b = cache.render_png()
        assert etag_a != etag_b

    def test_no_grid_yet(self):
        assert MapCache().render_png() is None

    def test_png_is_cropped_to_known_cells(self):
        """既知セルが狭い地図では PNG が crop 後の大きさになる。"""
        cache = MapCache()
        grid = make_grid(size=40, wall_row=20)
        grid.data = array.array('b', [-1]) * (40 * 40)
        for c in range(10, 21):
            grid.data[20 * 40 + c] = 0
        cache.set_grid(grid)
        png, _etag = cache.render_png()
        with Image.open(io.BytesIO(png)) as im:
            width, height = im.size
        r0, r1, c0, c1 = cache.snapshot()[2]
        assert (width, height) == (c1 - c0, r1 - r0)
        assert width < 40 and height < 40

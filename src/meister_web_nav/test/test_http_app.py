"""/api/map.png の ETag / 304 経路のテスト。

実際の ThreadingHTTPServer を ephemerical port で立て、http.client で実際の
リクエストを流す。MapListener の __init__ は ROS グラフ (rclpy.init と購読登録)
を要するのでスタブするが、HTTP が触る 3 メソッドは本物の MapListener を
そのまま借りる。ロジックを複製しないので、本物のコード経路を検証できる。

- (a) 地図が変わらない連続リクエストで 2 回目が 304
- (b) 地図を変えると 200 かつ別の bytes
- /api/map の JSON 形状が変わっていないこと (他のコードが依存している)
"""
import http.client
import json
import sys
import threading
from http.server import ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pytest  # noqa: E402

from meister_web_nav.http_app import (_entity_tag, _etag_matches,  # noqa: E402
                                      make_handler)
from meister_web_nav.map_cache import MapCache  # noqa: E402
from meister_web_nav.map_listener import MapListener  # noqa: E402

from conftest import make_grid  # noqa: E402

WEBUI_DIR = Path(__file__).resolve().parents[1] / 'webui'


class StubMapListener:
    """ROS グラフ無しで MapListener の HTTP 面だけを差し出す。"""

    def __init__(self) -> None:
        self._maps = MapCache()

    metadata = MapListener.metadata
    render_png = MapListener.render_png

    def set_grid(self, grid) -> None:
        self._maps.set_grid(grid)

    def robot_pose(self):
        return None


class StubNavWorker:
    def status(self):
        return {'state': 'idle', 'message': 'stub'}

    def submit(self, points):
        pass

    def cancel(self):
        pass


@pytest.fixture
def server():
    listener = StubMapListener()
    handler = make_handler(WEBUI_DIR, listener, StubNavWorker())
    httpd = ThreadingHTTPServer(('127.0.0.1', 0), handler)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    try:
        yield httpd.server_address[1], listener
    finally:
        httpd.shutdown()
        httpd.server_close()
        thread.join(timeout=5)


@pytest.fixture
def counted_calls(monkeypatch):
    """MapCache の crop / encode 呼び出し回数を数える。

    304 の経路が本当に encode を再実行していないことは、応答時間ではなく
    呼び出し回数で見る (時間は負荷で変わるため)。
    """
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
    return calls


def get(port, path, if_none_match=None):
    """if_none_match に list を渡すと If-None-Match を複数ヘッダ行で送る。

    http.client は headers= に list を受け付けないので、その経路だけ putheader を
    使う (同名フィールドの連結を実際に送る)。
    """
    conn = http.client.HTTPConnection('127.0.0.1', port, timeout=10)
    if isinstance(if_none_match, (list, tuple)):
        conn.putrequest('GET', path)
        for value in if_none_match:
            conn.putheader('If-None-Match', value)
        conn.endheaders()
    else:
        headers = {}
        if if_none_match is not None:
            headers['If-None-Match'] = if_none_match
        conn.request('GET', path, headers=headers)
    res = conn.getresponse()
    body = res.read()
    conn.close()
    return res.status, dict(res.getheaders()), body


class TestConditionalGet:
    def test_unchanged_map_yields_304(self, server):
        """(a) 地図が変わらなければ 2 回目は 304 かつ本文なし。"""
        port, listener = server
        listener.set_grid(make_grid())

        status, headers, body = get(port, '/api/map.png')
        assert status == 200
        etag = headers['ETag']
        assert etag
        assert body.startswith(b'\x89PNG')
        assert headers['Cache-Control'] == 'no-cache'

        status2, headers2, body2 = get(port, '/api/map.png', if_none_match=etag)
        assert status2 == 304
        assert body2 == b''
        assert headers2['ETag'] == etag

    def test_emitted_etag_is_rfc_quoted(self, server):
        """RFC 9110 8.8.3: entity-tag は引用符付き。引用符無しだと規格に準ずる
        クライアント (または tag を正規化する proxy) では 304 が成立しない。"""
        port, listener = server
        listener.set_grid(make_grid())
        _status, headers, _body = get(port, '/api/map.png')
        etag = headers['ETag']
        assert etag.startswith('"') and etag.endswith('"'), etag
        assert etag[1:-1] and ' ' not in etag

    def test_rfc_forms_all_match(self, server):
        """強引用符 / weak / 混在リスト / 重複ヘッダ行 / 旧形式の未引用符。"""
        port, listener = server
        listener.set_grid(make_grid())
        _status, headers, _body = get(port, '/api/map.png')
        etag = headers['ETag']
        raw = etag.strip('"')

        assert get(port, '/api/map.png', if_none_match=etag)[0] == 304
        assert get(port, '/api/map.png', if_none_match=f'W/{etag}')[0] == 304
        assert get(port, '/api/map.png', if_none_match=f'  W/{etag}  ')[0] == 304
        assert get(port, '/api/map.png',
                   if_none_match=f'"other", {etag}')[0] == 304
        assert get(port, '/api/map.png',
                   if_none_match=f'"other", W/"x", {etag}')[0] == 304
        assert get(port, '/api/map.png', if_none_match='*')[0] == 304
        assert get(port, '/api/map.png', if_none_match=raw)[0] == 304
        assert get(port, '/api/map.png',
                   if_none_match=['"other"', etag])[0] == 304

    def test_stale_tag_yields_full_body(self, server):
        """不一致は 200 + 本文。「常に 304」にすると地図が白紙で固まる。"""
        port, listener = server
        listener.set_grid(make_grid())
        for stale in ('"deadbeefdeadbeef"', 'deadbeefdeadbeef',
                      'W/"deadbeefdeadbeef"', '"0"'):
            status, headers, body = get(port, '/api/map.png', if_none_match=stale)
            assert status == 200, stale
            assert body.startswith(b'\x89PNG'), stale
            assert headers['Content-Type'] == 'image/png'

    def test_changed_map_yields_new_bytes(self, server):
        """(b) 地図を変えると 200 かつ新しい bytes と新しい ETag。"""
        port, listener = server
        listener.set_grid(make_grid())
        status1, headers1, body1 = get(port, '/api/map.png')
        assert status1 == 200

        listener.set_grid(make_grid(wall_row=20))
        status2, headers2, body2 = get(port, '/api/map.png',
                                       if_none_match=headers1['ETag'])
        assert status2 == 200
        assert headers2['ETag'] != headers1['ETag']
        assert body2 != body1
        assert body2.startswith(b'\x89PNG')

    def test_304_does_not_recompute_png(self, server, counted_calls):
        """304 は encode も _crop_bounds も再実行しない。"""
        port, listener = server
        listener.set_grid(make_grid())
        status, headers, _body = get(port, '/api/map.png')
        assert status == 200
        assert counted_calls == {'crop': 1, 'encode': 1}

        for _ in range(5):
            assert get(port, '/api/map.png', if_none_match=headers['ETag'])[0] == 304
        assert counted_calls == {'crop': 1, 'encode': 1}

        listener.set_grid(make_grid(wall_row=20))
        assert get(port, '/api/map.png', if_none_match=headers['ETag'])[0] == 200
        assert counted_calls == {'crop': 2, 'encode': 2}

    def test_repeat_polls_stay_304(self, server):
        """連続 5 回ポーリングしても 2 回目以降は 304 のまま。"""
        port, listener = server
        listener.set_grid(make_grid())
        _status, headers, _body = get(port, '/api/map.png')
        etag = headers['ETag']
        for _ in range(5):
            assert get(port, '/api/map.png', if_none_match=etag)[0] == 304

    def test_no_store_is_gone(self, server):
        """no-store のままだとブラウザが If-None-Match を送らない。"""
        port, listener = server
        listener.set_grid(make_grid())
        _status, headers, _body = get(port, '/api/map.png')
        assert headers['Cache-Control'] != 'no-store'

    def test_query_string_still_routes(self, server):
        """?t= 付きでも同じ route (旧 cache-buster との互換)。"""
        port, listener = server
        listener.set_grid(make_grid())
        status, headers, _body = get(port, '/api/map.png?t=12345')
        assert status == 200
        assert get(port, '/api/map.png?t=99999', if_none_match=headers['ETag'])[0] == 304

    def test_no_map_is_404(self, server):
        port, _listener = server
        assert get(port, '/api/map.png')[0] == 404


class TestMapJsonShapeUnchanged:
    def test_metadata_keys(self, server):
        port, listener = server
        listener.set_grid(make_grid(size=20))
        status, headers, body = get(port, '/api/map')
        assert status == 200
        assert headers['Content-Type'] == 'application/json'
        payload = json.loads(body)
        assert payload['has_map'] is True
        assert payload['image_url'] == '/api/map.png'
        assert set(payload) == {
            'resolution', 'width', 'height', 'origin', 'has_map', 'image_url'}
        assert set(payload['origin']) == {'x', 'y'}

    def test_no_map_shape(self, server):
        port, _listener = server
        status, _headers, body = get(port, '/api/map')
        assert status == 200
        assert json.loads(body) == {'has_map': False}


class TestEtagMatchesUnit:
    STRONG = '"923a0a5ea697ccb5"'
    RAW = '923a0a5ea697ccb5'

    def test_strong_matches(self):
        assert _etag_matches(self.STRONG, self.STRONG)

    def test_stale_does_not_match(self):
        assert not _etag_matches('"deadbeefdeadbeef"', self.STRONG)
        assert not _etag_matches(self.RAW[:-1], self.STRONG)
        assert not _etag_matches(f'"{self.RAW}x"', self.STRONG)
        assert not _etag_matches(f'x{self.RAW}', self.STRONG)

    def test_weak_matches_strong(self):
        """RFC 9110 13.1.2: If-None-Match は weak 比較。weakness marker は
        ABNF 由来で大文字小文字を区別しない。"""
        assert _etag_matches(f'W/{self.STRONG}', self.STRONG)
        assert _etag_matches(f'w/{self.STRONG}', self.STRONG)
        assert _etag_matches(f'  W/  {self.STRONG}  ', self.STRONG)

    def test_star_matches(self):
        assert _etag_matches('*', self.STRONG)

    def test_list_with_whitespace(self):
        assert _etag_matches(f'"aaaaaaaaaaaaaaa", W/{self.STRONG}', self.STRONG)
        assert _etag_matches(f'W/{self.STRONG},"a","b"', self.STRONG)

    def test_legacy_unquoted_form(self):
        """旧サーバが出していた未引用符の形。rolling deploy の隙間に browser が
        送ってこうるので受容する (http_app の _etag_matches に理由がある)。"""
        assert _etag_matches(self.RAW, self.STRONG)
        assert _etag_matches(f'W/{self.RAW}', self.STRONG)
        assert _etag_matches(f'"other", {self.RAW}', self.STRONG)

    def test_absent_header(self):
        assert not _etag_matches(None, self.STRONG)
        assert not _etag_matches('', self.STRONG)

    def test_unparsable_elements(self):
        assert not _etag_matches('""', self.STRONG)
        assert not _etag_matches('W/', self.STRONG)
        assert not _etag_matches(',,', self.STRONG)
        assert not _etag_matches('"unclosed', self.STRONG)

    def test_entity_tag_is_quoted(self):
        assert _entity_tag('deadbeef') == '"deadbeef"'
        assert _entity_tag('deadbeef') != 'deadbeef'

"""HTTP request handler: serves the web UI and the map/nav JSON API."""
import json
import mimetypes
from http.server import BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlsplit

MAX_BODY_BYTES = 1_000_000


def _entity_tag(version: str) -> str:
    """地図の content hash を RFC 9110 8.8.3 の entity-tag に包む。

    引用符を落とすと、規格に準ずるクライアントや tag を正規化する中間 proxy が
    引用符付きの If-None-Match を返してこなくなる。引用符なしの形を解釈するのは
    このサーバーだけになり、正規化を経由した経路では 304 が永久に成立せず、
    毎回 200 と全バイトに戻る = この変更の目的が潰れるので引用符は省略しない。
    """
    return f'"{version}"'


def _opaque_tag(candidate: str) -> str | None:
    """If-None-Match の 1 要素を、比較に使う中身だけ取り出す。

    None は「読めない要素」で比較対象外。weakness marker は ABNF の
    大文字小文字非区別 (RFC 5234 2.3) に従い w/ も受理する。引用符が無い値は
    旧形式の文字列をそのまま返す (受容の理由は _etag_matches)。
    """
    tag = candidate.strip()
    if tag[:2] in ('W/', 'w/'):
        tag = tag[2:].strip()
    if len(tag) >= 2 and tag.startswith('"') and tag.endswith('"'):
        return tag[1:-1]
    return tag or None


def _etag_matches(header: str | None, etag: str) -> bool:
    """If-None-Match が etag に一致するかを判定する。

    etag にはこのサーバーが ETag ヘッダに送る文字列をそのまま渡す。
    RFC 9110 13.1.2 は If-None-Match に weak 比較 (opaque-tag だけ見比べる) を
    要求するので、weak なタグは strong な同じ値に一致する。entity-tag 全体の
    文字列で比較すると weakness marker が差として残り、weak なクライアントを
    取りこぼす。1 ヘッダにカンマ区切りで複数タグ送的ることも許されているので
    要素ごとに割る。

    引用符無しの値も比較対象に残す。旧サーバは引用符なしで ETag を送っていた
    ので、サーバだけを更新してブラウザの HTTP キャッシュは古いままという
    rolling deploy の隙間では旧形式をそのまま If-None-Match に戻してくる。
    これを無視すると更新直後の 1 リクエストだけ地図を全バイト再送し、この変更
    が潰そうとしていたコストを同じ手で戻す。値は地図内容そのものの hash なので
    形式が不正でも一致判定そのものは正しい。旧形式を持つクライアントが
    無くなった版で外せる。
    """
    if not header:
        return False
    target = _opaque_tag(etag)
    if target is None:
        return False
    for candidate in header.split(','):
        tag = _opaque_tag(candidate)
        if tag == '*' or (tag is not None and tag == target):
            return True
    return False


def make_handler(webui_dir: Path, map_listener, nav_worker):
    class Handler(BaseHTTPRequestHandler):
        server_version = 'MeisterWebNav/0.1'

        def log_message(self, fmt, *args):
            pass  # keep stdout quiet; rely on the ROS logger elsewhere

        def _send_json(self, payload: dict, status: int = 200) -> None:
            body = json.dumps(payload).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_file(self, path: Path, content_type: str) -> None:
            data = path.read_bytes()
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _read_json_body(self) -> dict:
            length = int(self.headers.get('Content-Length', '0'))
            if length <= 0 or length > MAX_BODY_BYTES:
                raise ValueError('invalid content length')
            return json.loads(self.rfile.read(length))

        def _if_none_match(self) -> str | None:
            """If-None-Match を全ヘッダ行まとめて 1 本にして返す。

            RFC 9110 5.2 のフィールド合成 (同名フィールドの連結) をそのままやる。
            get() だけだと 2 行目以降を落とし、正規クライアントが 2 行送った
            ときに対象のタグを見失う。
            """
            values = self.headers.get_all('If-None-Match')
            return ','.join(values) if values else None

        def do_GET(self):
            # クエリ文字列 (?t=...) はパス比較から除外する (キャッシュ対策の cache-buster 対応)
            path = urlsplit(self.path).path
            if path in ('/', '/index.html'):
                self._send_file(webui_dir / 'index.html', 'text/html; charset=utf-8')
            elif path == '/app.js':
                self._send_file(webui_dir / 'app.js', 'text/javascript; charset=utf-8')
            elif path == '/style.css':
                self._send_file(webui_dir / 'style.css', 'text/css; charset=utf-8')
            elif path == '/api/map':
                meta = map_listener.metadata()
                if meta is None:
                    self._send_json({'has_map': False}, status=200)
                else:
                    meta['has_map'] = True
                    meta['image_url'] = '/api/map.png'
                    self._send_json(meta)
            elif path == '/api/map.png':
                result = map_listener.render_png()
                if result is None:
                    self.send_response(404)
                    self.end_headers()
                else:
                    png, version = result
                    # 引用符の付与は HTTP 面 (_entity_tag) の責務。map_cache は
                    # entity-tag ではなく生の content hash を返すだけ。
                    etag = _entity_tag(version)
                    # no-cache は「保存してよいが必ず再検証せよ」の意味。no-store の
                    # ままだとブラウザは If-None-Match を送らず、毎回 200 + 全バイト
                    # ダウンロードになる。SLAM 中は地図が動くので stale は許さない。
                    if _etag_matches(self._if_none_match(), etag):
                        self.send_response(304)
                        self.send_header('ETag', etag)
                        self.send_header('Cache-Control', 'no-cache')
                        self.end_headers()
                    else:
                        self.send_response(200)
                        self.send_header('Content-Type', 'image/png')
                        self.send_header('Content-Length', str(len(png)))
                        self.send_header('ETag', etag)
                        self.send_header('Cache-Control', 'no-cache')
                        self.end_headers()
                        self.wfile.write(png)
            elif path == '/api/pose':
                pose = map_listener.robot_pose()
                self._send_json(
                    {'has_pose': pose is not None, **(pose or {})})
            elif path == '/api/status':
                self._send_json(nav_worker.status())
            else:
                mime = mimetypes.guess_type(path)[0] or 'application/octet-stream'
                candidate = webui_dir / path.lstrip('/')
                try:
                    resolved = candidate.resolve()
                    resolved.relative_to(webui_dir.resolve())
                except (ValueError, OSError):
                    resolved = None
                if resolved is not None and resolved.is_file():
                    self._send_file(resolved, mime)
                else:
                    self.send_response(404)
                    self.end_headers()

        def do_POST(self):
            if self.path == '/api/nav':
                try:
                    body = self._read_json_body()
                    points = body['waypoints']
                    if not isinstance(points, list) or not points:
                        raise ValueError('waypoints must be a non-empty list')
                    for p in points:
                        float(p['x'])
                        float(p['y'])
                except (ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
                    self._send_json({'error': str(exc)}, status=400)
                    return
                nav_worker.submit(points)
                self._send_json({'status': 'submitted', 'count': len(points)})
            elif self.path == '/api/cancel':
                nav_worker.cancel()
                self._send_json({'status': 'cancel_requested'})
            else:
                self.send_response(404)
                self.end_headers()

    return Handler

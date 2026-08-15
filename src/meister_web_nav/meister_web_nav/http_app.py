"""HTTP request handler: serves the web UI and the map/nav JSON API."""
import json
import mimetypes
from http.server import BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlsplit

MAX_BODY_BYTES = 1_000_000


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
                png = map_listener.render_png()
                if png is None:
                    self.send_response(404)
                    self.end_headers()
                else:
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/png')
                    self.send_header('Content-Length', str(len(png)))
                    self.send_header('Cache-Control', 'no-store')
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

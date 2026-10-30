from __future__ import annotations

import json
import socket
import socketserver
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse


MIME = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".svg": "image/svg+xml",
    ".ico": "image/x-icon",
    ".png": "image/png",
    ".woff2": "font/woff2",
}


def web_root() -> Path:
    return Path(__file__).resolve().parents[1] / "web"


class ThreadingHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class StudioHandler(BaseHTTPRequestHandler):
    mission_bytes: bytes = b"{}"
    root: Path = web_root()

    def log_message(self, fmt: str, *args) -> None:
        return

    def do_GET(self) -> None:
        path = unquote(urlparse(self.path).path)
        if path in ("/mission.json",):
            self._send(200, "application/json; charset=utf-8", self.mission_bytes)
            return
        if path in ("/", "/index.html"):
            self._file(self.root / "index.html")
            return
        rel = path.lstrip("/").replace("\\", "/")
        if not rel or ".." in rel.split("/"):
            self._send(404, "text/plain; charset=utf-8", b"not found")
            return
        target = (self.root / rel).resolve()
        try:
            target.relative_to(self.root.resolve())
        except ValueError:
            self._send(403, "text/plain; charset=utf-8", b"forbidden")
            return
        if not target.is_file():
            self._send(404, "text/plain; charset=utf-8", b"not found")
            return
        self._file(target)

    def _file(self, path: Path) -> None:
        data = path.read_bytes()
        mime = MIME.get(path.suffix.lower(), "application/octet-stream")
        self._send(200, mime, data)

    def _send(self, code: int, mime: str, data: bytes) -> None:
        self.send_response(code)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)


def _bind(host: str, port: int, handler) -> ThreadingHTTPServer:
    if port == 0:
        return ThreadingHTTPServer((host, 0), handler)
    last_err: OSError | None = None
    for p in range(port, port + 12):
        try:
            return ThreadingHTTPServer((host, p), handler)
        except OSError as exc:
            last_err = exc
            continue
    raise OSError(f"could not bind studio on {host}:{port}") from last_err


def start_studio(
    mission: dict,
    *,
    host: str = "127.0.0.1",
    port: int = 8765,
    open_browser: bool = False,
    block: bool = True,
) -> tuple[ThreadingHTTPServer, str]:
    payload = json.dumps(mission, separators=(",", ":")).encode("utf-8")
    StudioHandler.mission_bytes = payload
    StudioHandler.root = web_root()
    httpd = _bind(host, port, StudioHandler)
    bound_host, bound_port = httpd.server_address[:2]
    url = f"http://{bound_host}:{bound_port}/"
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    deadline = time.time() + 2.0
    while time.time() < deadline:
        try:
            with socket.create_connection((bound_host, bound_port), timeout=0.2):
                break
        except OSError:
            time.sleep(0.05)
    if open_browser:
        webbrowser.open(url, new=2)
    if block:
        try:
            thread.join()
        except KeyboardInterrupt:
            httpd.shutdown()
    return httpd, url

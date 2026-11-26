"""RK 统一网页代理：静态前端 + 转发 /api/* 到车机 10.7.5.88:8098。

车机 mapping/nav 后端要求 X-Test-Token，本代理在转发时附带 /key/token 的内容。
/api/stream 为 SSE 长连接逐块转发。
"""
import json
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import Request, urlopen
from urllib.error import HTTPError, URLError

ROOT = Path(__file__).resolve().parent
STATIC = ROOT / "static"
TOKEN_FILE = Path("/key/token")
UPSTREAM = "http://10.7.5.88:8098"
TOKEN = TOKEN_FILE.read_text().strip() if TOKEN_FILE.exists() else ""

CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".svg": "image/svg+xml",
}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def cors_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def proxy(self, method, data=b""):
        headers = {"Content-Type": "application/json"}
        if TOKEN:
            headers["X-Test-Token"] = TOKEN
        request = Request(UPSTREAM + self.path, data=data, headers=headers, method=method)
        try:
            with urlopen(request, timeout=40) as response:
                return response.read(), response.status
        except HTTPError as error:
            return error.read(), error.code
        except (URLError, TimeoutError) as error:
            return json.dumps({"error": str(error)}).encode(), 502

    def serve_static(self, path):
        relative = "index.html" if path in ("/", "/index.html") else path.lstrip("/")
        candidate = (STATIC / relative).resolve()
        if not str(candidate).startswith(str(STATIC.resolve())):
            return None
        if not candidate.is_file():
            return None
        return candidate.read_bytes(), CONTENT_TYPES.get(candidate.suffix, "application/octet-stream")

    def reply_bytes(self, status, body, content_type, cache="no-store"):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.cors_headers()
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", cache)
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/api/stream":
            self.stream_proxy()
            return
        if self.path.startswith("/api/"):
            body, status = self.proxy("GET")
            self.reply_bytes(status, body, "application/json; charset=utf-8")
            return
        static = self.serve_static(self.path)
        if static is None:
            self.send_error(404)
            return
        body, content_type = static
        self.reply_bytes(200, body, content_type)

    def do_POST(self):
        if not self.path.startswith("/api/"):
            self.send_error(404)
            return
        size = int(self.headers.get("Content-Length", "0"))
        data = self.rfile.read(size) if 0 < size <= 65536 else b""
        body, status = self.proxy("POST", data)
        self.reply_bytes(status, body, "application/json; charset=utf-8")

    def do_OPTIONS(self):
        self.send_response(204)
        self.cors_headers()
        self.send_header("Content-Length", "0")
        self.end_headers()

    def stream_proxy(self):
        headers = {"Accept": "text/event-stream"}
        if TOKEN:
            headers["X-Test-Token"] = TOKEN
        request = Request(UPSTREAM + "/api/stream", headers=headers)
        try:
            upstream = urlopen(request, timeout=None)
        except (HTTPError, URLError):
            self.send_error(502)
            return
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.cors_headers()
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        try:
            while True:
                chunk = upstream.read(1024)
                if not chunk:
                    break
                self.wfile.write(chunk)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            upstream.close()


def main():
    server = ThreadingHTTPServer(("127.0.0.1", 8088), Handler)
    print("unified web at 127.0.0.1:8088")
    server.serve_forever()


if __name__ == "__main__":
    main()

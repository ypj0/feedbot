import json
import os
from pathlib import Path
import socket
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.headers.get("Host") != "127.0.0.1:8088":
            self.send_error(403)
            return
        status, content_type = 200, "application/json; charset=utf-8"
        if self.path == "/":
            body = Path(__file__).with_name("index.html").read_bytes()
            content_type = "text/html; charset=utf-8"
        elif self.path == "/app.js":
            body = Path(__file__).with_name("app.js").read_bytes()
            content_type = "text/javascript; charset=utf-8"
        elif self.path == "/api/state":
            request = Request("http://10.7.5.88:8098/api/state", headers={"X-Test-Token": self.server.token})
            try:
                with urlopen(request, timeout=0.7) as response:
                    body = response.read()
                    status = response.status
            except HTTPError as error:
                body, status = error.read(), error.code
            except (URLError, socket.timeout, TimeoutError) as error:
                body, status = json.dumps({"error": str(error)}).encode(), 502
        else:
            self.send_error(404)
            return
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Security-Policy", "default-src 'self'; style-src 'unsafe-inline'; frame-ancestors 'none'; object-src 'none'")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_):
        pass


if __name__ == "__main__":
    server = ThreadingHTTPServer(("127.0.0.1", 8088), Handler)
    server.token = Path(os.environ["FEEDBOT_TEST_TOKEN_FILE"]).read_text().strip()
    try:
        server.serve_forever()
    finally:
        server.server_close()

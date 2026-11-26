import json
import os
from pathlib import Path
import socket
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


STATIC = Path(__file__).with_name("static")
FILES = {"/": ("index.html", "text/html; charset=utf-8"),
         "/app.js": ("app.js", "text/javascript; charset=utf-8"),
         "/style.css": ("style.css", "text/css; charset=utf-8")}


class Handler(BaseHTTPRequestHandler):
    def reply(self, status, body, content_type="application/json; charset=utf-8"):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Security-Policy", "default-src 'self'; frame-ancestors 'none'; object-src 'none'")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def proxy(self, data=None):
        request = Request("http://10.7.5.88:8098" + self.path, data=data,
                          headers={"X-Test-Token": self.server.token, "Content-Type": "application/json"})
        try:
            with urlopen(request, timeout=0.7) as response:
                self.reply(response.status, response.read(65536))
        except HTTPError as error:
            self.reply(error.code, error.read(65536))
        except (URLError, socket.timeout, TimeoutError) as error:
            self.reply(502, json.dumps({"error": "车机连接失败：" + str(error)}).encode())

    def do_GET(self):
        if self.headers.get("Host") != "127.0.0.1:8088":
            return self.reply(403, b'{"error":"invalid host"}')
        if self.path == "/api/state":
            return self.proxy()
        if self.path not in FILES:
            return self.reply(404, b'{"error":"unknown endpoint"}')
        filename, content_type = FILES[self.path]
        self.reply(200, (STATIC / filename).read_bytes(), content_type)

    def do_POST(self):
        if (self.path != "/api/command" or self.headers.get("Host") != "127.0.0.1:8088" or
                self.headers.get("Origin") != "http://127.0.0.1:8088" or
                self.headers.get("X-Test-UI") != "1"):
            return self.reply(403, b'{"error":"same-origin UI required"}')
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 1024:
                raise ValueError("invalid command size")
        except ValueError:
            return self.reply(400, b'{"error":"invalid command size"}')
        self.proxy(self.rfile.read(length))

    def log_message(self, *_):
        pass


def main():
    server = ThreadingHTTPServer(("127.0.0.1", 8088), Handler)
    server.token = Path(os.environ["FEEDBOT_TEST_TOKEN_FILE"]).read_text().strip()
    try:
        server.serve_forever()
    finally:
        server.server_close()


if __name__ == "__main__":
    main()

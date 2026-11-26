"""RK 导航网页代理：静态页面 + 转发 GET/POST /api/* 到车机 10.7.5.88:8098。
/api/stream 是 SSE 长连接，逐块流式转发；其余接口一次性转发。"""
import json
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import Request, urlopen
from urllib.error import HTTPError, URLError

ROOT = Path(__file__).resolve().parent


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def cors_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')

    def proxy(self, method, data=b''):
        req = Request('http://10.7.5.88:8098' + self.path, data=data,
                      headers={'Content-Type': 'application/json'}, method=method)
        try:
            with urlopen(req, timeout=20) as resp:
                return resp.read(), resp.status, resp.headers
        except HTTPError as e:
            return e.read(), e.code, e.headers
        except (URLError, TimeoutError) as e:
            return json.dumps({'error': str(e)}).encode(), 502, {}

    def do_GET(self):
        if self.path == '/api/stream':
            self.stream_proxy()
            return
        if self.path.startswith('/api/'):
            body, status, _ = self.proxy('GET')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.cors_headers()
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path in ('/', '/index.html'):
            body = (ROOT / 'index.html').read_bytes()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.cors_headers()
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404)

    def stream_proxy(self):
        req = Request('http://10.7.5.88:8098/api/stream', headers={'Accept': 'text/event-stream'})
        try:
            upstream = urlopen(req, timeout=None)
        except (HTTPError, URLError) as e:
            self.send_error(502)
            return
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.cors_headers()
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
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

    def do_POST(self):
        if not self.path.startswith('/api/'):
            self.send_error(404); return
        size = int(self.headers.get('Content-Length', '0'))
        data = self.rfile.read(size) if 0 < size <= 4096 else b''
        body, status, _ = self.proxy('POST', data)
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.cors_headers()
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self.cors_headers()
        self.send_header('Content-Length', '0')
        self.end_headers()


def main():
    server = ThreadingHTTPServer(('127.0.0.1', 8088), Handler)
    print('nav web at 127.0.0.1:8088')
    server.serve_forever()


if __name__ == '__main__':
    main()

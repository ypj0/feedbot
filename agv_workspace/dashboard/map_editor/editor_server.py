"""本地地图编辑：读 PGM，接收闭合多边形顶点，重写 PGM。
多边形内：保留 free/occ，unknown 设为 free（用户确认可通行区域已扫）。
多边形外：全部设 occupied（空气墙）。
"""
import base64
import json
import re
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PGM = ROOT / 'map.pgm'
YAML = ROOT / 'map.yaml'
OUT = ROOT / 'map_edited.pgm'


def parse_pgm(data):
    tokens = []
    idx = 0
    while len(tokens) < 4:
        while idx < len(data) and data[idx:idx+1] in b' \t\r\n':
            idx += 1
        if data[idx:idx+1] == b'#':
            while data[idx] != 10:
                idx += 1
            continue
        s = idx
        while idx < len(data) and data[idx:idx+1] not in b' \t\r\n':
            idx += 1
        tokens.append(data[s:idx].decode())
    magic, w, h, maxval = tokens
    w, h, maxval = int(w), int(h), int(maxval)
    idx += 1
    header = data[:idx]
    pixels = bytearray(data[idx:idx + w * h])
    return magic, w, h, maxval, header, pixels


def occupancy(v):
    # 254->free(0), 0->occ(100), else unknown(-1)
    if v >= 250:
        return 0
    if v <= 10:
        return 100
    return -1


def occ_to_pixel(o):
    return 254 if o == 0 else (0 if o == 100 else 205)


def point_in_poly(x, y, poly):
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def reply(self, status, body):
        if isinstance(body, (dict, list)):
            body = json.dumps(body).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == '/':
            body = (ROOT / 'index.html').read_bytes()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == '/map.png':
            # 用 PGM 生成一个 PNG 预览（简单做法：前端直接读原始 PGM 转 canvas）
            self.reply(404, {'error': 'use /map.pgm'})
            return
        if self.path == '/map.pgm':
            body = PGM.read_bytes()
            self.send_response(200)
            self.send_header('Content-Type', 'application/octet-stream')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == '/map.meta':
            magic, w, h, maxval, header, pixels = parse_pgm(PGM.read_bytes())
            self.reply(200, {'width': w, 'height': h, 'maxval': maxval})
            return
        if self.path == '/map_edited.pgm':
            if not OUT.exists():
                self.reply(404, {'error': 'not generated yet'})
                return
            body = OUT.read_bytes()
            self.send_response(200)
            self.send_header('Content-Type', 'application/octet-stream')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.reply(404, {'error': 'unknown'})

    def do_POST(self):
        if self.path != '/edit':
            self.reply(404, {'error': 'unknown'})
            return
        size = int(self.headers.get('Content-Length', '0'))
        if not 0 < size <= 4 * 1024 * 1024:
            self.reply(400, {'error': 'bad size'})
            return
        payload = json.loads(self.rfile.read(size))
        poly = payload.get('polygon')
        if not isinstance(poly, list) or len(poly) < 3:
            self.reply(400, {'error': 'need closed polygon >= 3 vertices'})
            return
        poly = [(float(x), float(y)) for x, y in poly]
        # 自动闭合
        if poly[0] != poly[-1]:
            poly.append(poly[0])

        magic, w, h, maxval, header, pixels = parse_pgm(PGM.read_bytes())
        # 建立 occupancy 数组
        occ = [occupancy(v) for v in pixels]
        # 多边形内：unknown->free；多边形外：全部 occ
        free_count = 0
        occ_count = 0
        for y in range(h):
            for x in range(w):
                inside = point_in_poly(x + 0.5, y + 0.5, poly)
                o = occ[y * w + x]
                if inside:
                    # 保留 free/occ；unknown 设为 free
                    if o == -1:
                        o = 0
                else:
                    o = 100
                pixels[y * w + x] = occ_to_pixel(o)
                if o == 0:
                    free_count += 1
                elif o == 100:
                    occ_count += 1
        OUT.write_bytes(header + bytes(pixels))
        self.reply(200, {'saved': True, 'free': free_count, 'occ': occ_count,
                         'unknown': w * h - free_count - occ_count})


def main():
    if not PGM.exists():
        raise SystemExit('map.pgm missing; put the source PGM next to editor_server.py')
    server = ThreadingHTTPServer(('127.0.0.1', 8080), Handler)
    print('editor at http://127.0.0.1:8080/')
    server.serve_forever()


if __name__ == '__main__':
    main()

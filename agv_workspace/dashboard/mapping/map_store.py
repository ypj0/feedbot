import hashlib
import hmac
import json
import os
from pathlib import Path
import re
import subprocess
import threading
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from uuid import uuid4

import yaml


MAPS = Path('/maps')
DOWNLOAD = re.compile(r'/api/maps/([A-Za-z0-9_-]+)/feedbot\.(yaml|pgm)')


def inspect_map(directory):
    metadata = yaml.safe_load((directory / 'feedbot.yaml').read_text())
    if metadata['image'] != 'feedbot.pgm':
        raise ValueError('map YAML must reference sibling feedbot.pgm')
    with (directory / 'feedbot.pgm').open('rb') as stream:
        if stream.readline().strip() != b'P5':
            raise ValueError('expected P5 map image')
        dimensions = stream.readline()
        while dimensions.startswith(b'#'):
            dimensions = stream.readline()
        width, height = map(int, dimensions.split())
        if int(stream.readline()) != 255 or len(stream.read()) != width * height:
            raise ValueError('map image size or grayscale mismatch')
    return {'id': directory.name, 'width': width, 'height': height,
            'resolution': metadata['resolution'], 'origin': metadata['origin'],
            'saved_at': datetime.now(timezone.utc).isoformat(),
            'files': {name: {'url': '/api/maps/' + directory.name + '/' + name,
                             'sha256': hashlib.sha256((directory / name).read_bytes()).hexdigest()}
                      for name in ('feedbot.yaml', 'feedbot.pgm')}}


class Store:
    def __init__(self, root):
        self.root = root
        self.lock = threading.Lock()
        self.path = root / 'save_status.json'
        self.state = json.loads(self.path.read_text()) if self.path.exists() else {
            'phase': 'idle', 'result': None, 'error': ''}
        if self.state['phase'] == 'saving':
            self.state.update(phase='failed', error='保存服务中断；请核对目录，不自动重发')
            self.persist()

    def persist(self):
        temporary = self.path.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.state, ensure_ascii=False))
        temporary.replace(self.path)

    def snapshot(self):
        with self.lock:
            return dict(self.state)

    def start(self):
        with self.lock:
            if self.state['phase'] == 'saving':
                raise ValueError('正在保存，不重复提交')
            identifier = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_') + uuid4().hex[:8]
            directory = self.root / identifier
            directory.mkdir()
            self.state = {'phase': 'saving', 'id': identifier, 'result': None, 'error': ''}
            self.persist()
            worker = threading.Thread(target=self.save, args=(directory,))
            worker.start()
            return dict(self.state)

    def save(self, directory):
        try:
            with (directory / 'save.log').open('w') as log:
                completed = subprocess.run([
                    'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                    '-f', str(directory / 'feedbot'), '--ros-args', '-p', 'save_map_timeout:=15.0'],
                    stdout=log, stderr=subprocess.STDOUT)
            if completed.returncode != 0:
                raise RuntimeError((directory / 'save.log').read_text()[-4000:])
            result = inspect_map(directory)
            with self.lock:
                self.state.update(phase='saved', result=result)
                self.persist()
        except Exception as error:
            with self.lock:
                self.state.update(phase='failed', error=str(error))
                self.persist()


class Handler(BaseHTTPRequestHandler):
    def reply(self, status, value):
        body = json.dumps(value, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def authenticated(self):
        return hmac.compare_digest(self.headers.get('X-Test-Token', ''), self.server.token)

    def do_GET(self):
        if not self.authenticated():
            return self.reply(403, {'error': 'unauthorized'})
        if self.path == '/api/save':
            return self.reply(200, self.server.store.snapshot())
        match = DOWNLOAD.fullmatch(self.path)
        if match is None:
            return self.reply(404, {'error': 'unknown endpoint'})
        filename = 'feedbot.' + match.group(2)
        try:
            body = (self.server.store.root / match.group(1) / filename).read_bytes()
        except FileNotFoundError:
            return self.reply(404, {'error': 'map file not found'})
        self.send_response(200)
        self.send_header('Content-Type', 'application/octet-stream')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Content-Disposition', 'attachment; filename="' + filename + '"')
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if not self.authenticated():
            return self.reply(403, {'error': 'unauthorized'})
        if self.path != '/api/save':
            return self.reply(404, {'error': 'unknown endpoint'})
        try:
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 < size <= 1024:
                raise ValueError('invalid request size')
            if json.loads(self.rfile.read(size)) != {'confirm_save': True}:
                raise ValueError('请确认停稳并完成扫描')
            self.reply(202, self.server.store.start())
        except (ValueError, TypeError) as error:
            self.reply(409, {'error': str(error)})

    def log_message(self, *_):
        pass


def main():
    server = ThreadingHTTPServer(('10.7.5.88', 8099), Handler)
    server.store = Store(MAPS)
    server.token = Path(os.environ['FEEDBOT_TEST_TOKEN_FILE']).read_text().strip()
    try:
        server.serve_forever()
    finally:
        server.server_close()


if __name__ == '__main__':
    main()

"""HTTP bridge between the 8088 navigation web and a running Cartographer node.

The navigation web backend runs in a different container, so it cannot import
cartographer_ros_msgs directly. This process runs inside the Cartographer
mapping container and exposes:
    GET  /status
    POST /finish
    POST /save?name=map_name
"""

import json
import os
import re
import shlex
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import rclpy
from cartographer_ros_msgs.srv import FinishTrajectory, GetTrajectoryStates, StartTrajectory, WriteState


MAPS_ROOT = Path('/opt/feedbot-maps')
PBSTREAM_TO_MAP = (
    '/opt/cartographer_ws/install/cartographer_ros/lib/cartographer_ros/'
    'cartographer_pbstream_to_ros_map'
)


class CartographerNode:
    def __init__(self):
        self.node = rclpy.create_node('cartographer_web_api')
        self.finish_client = self.node.create_client(FinishTrajectory, '/finish_trajectory')
        self.write_client = self.node.create_client(WriteState, '/write_state')
        self.get_states_client = self.node.create_client(GetTrajectoryStates, '/get_trajectory_states')
        self.start_client = self.node.create_client(StartTrajectory, '/start_trajectory')
        self.finished = False

    def spin(self):
        rclpy.spin(self.node)

    def call(self, client, request, timeout=30.0):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('%s service not available' % client.srv_name)
        future = client.call_async(request)
        deadline = time.time() + timeout
        while time.time() < deadline and not future.done():
            time.sleep(0.02)
        if not future.done():
            raise RuntimeError('%s service timeout' % client.srv_name)
        response = future.result()
        if response.status.code != 0:
            raise RuntimeError(response.status.message or 'code %d' % response.status.code)
        return response

    def mapping_active(self):
        response = self.call(self.get_states_client, GetTrajectoryStates.Request())
        return any(state == 0 for state in response.trajectory_states.trajectory_state)

    def start(self):
        request = StartTrajectory.Request()
        request.configuration_directory = '/opt/carto-deploy/cartographer/config'
        request.configuration_basename = 'feedbot_2d_mapping.lua'
        request.use_initial_pose = False
        request.relative_to_trajectory_id = 0
        response = self.call(self.start_client, request, timeout=60.0)
        self.finished = False
        return {'ok': True, 'trajectory_id': response.trajectory_id}

    def finish(self):
        request = FinishTrajectory.Request()
        request.trajectory_id = 0
        self.call(self.finish_client, request, timeout=60.0)
        self.finished = True

    def restart(self):
        try:
            self.finish()
        except RuntimeError:
            pass
        threading.Timer(0.5, os._exit, args=(0,)).start()
        return {'ok': True}

    def save(self, name):
        name = re.sub(r'[^A-Za-z0-9._-]+', '_', (name or '').strip())
        if not name:
            raise RuntimeError('map name is empty')
        outdir = MAPS_ROOT / name
        outdir.mkdir(parents=True, exist_ok=True)
        pbstream = outdir / 'map.pbstream'
        if not self.finished:
            self.finish()
        request = WriteState.Request()
        request.filename = str(pbstream)
        request.include_unfinished_submaps = True
        self.call(self.write_client, request, timeout=120.0)
        stem = str(outdir / 'feedbot')
        command = (
            'source /opt/cartographer_ws/install/setup.bash && exec '
            + shlex.quote(PBSTREAM_TO_MAP)
            + ' -pbstream_filename ' + shlex.quote(str(pbstream))
            + ' -map_filestem ' + shlex.quote(stem)
            + ' -resolution 0.05'
        )
        result = subprocess.run(
            ['bash', '-lc', command], capture_output=True, text=True, timeout=120.0
        )
        if result.returncode != 0:
            raise RuntimeError((result.stderr or result.stdout)[-500:])
        return {'name': name, 'dir': str(outdir)}


class Handler(BaseHTTPRequestHandler):
    node = None

    def log_message(self, *_):
        pass

    def reply(self, status, body):
        data = json.dumps(body).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == '/status':
            return self.reply(200, {'ok': True, 'mapping': self.node.mapping_active()})
        self.reply(404, {'error': 'unknown'})

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == '/finish':
            self.node.finish()
            return self.reply(200, {'ok': True, 'finished': True})
        if parsed.path == '/restart':
            result = self.node.restart()
            return self.reply(200, result)
        if parsed.path == '/start':
            result = self.node.start()
            return self.reply(200, result)
        if parsed.path == '/save':
            query = parse_qs(parsed.query)
            name = (query.get('name') or [''])[0]
            result = self.node.save(name)
            return self.reply(200, {'ok': True, **result})
        self.reply(404, {'error': 'unknown'})


def main():
    rclpy.init()
    handler_node = CartographerNode()
    Handler.node = handler_node
    server = ThreadingHTTPServer(('0.0.0.0', 8099), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print('cartographer web api at 0.0.0.0:8099', flush=True)
    handler_node.spin()


if __name__ == '__main__':
    main()

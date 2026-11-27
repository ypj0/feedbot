#!/usr/bin/env python3
"""Cartographer 进程级监督器（常驻容器入口）。

容器不再因切图而重建；这里常驻一个 HTTP 服务，按需启停 cartographer 的
ros2 launch 子进程：

    GET  /status              -> {"mode": "mapping"|"localization"|"idle"}
    POST /mapping             -> 切到建图
    POST /localize?name=MAP   -> 切到定位并加载该地图

这样切图只是杀进程 + 起进程，省掉 docker stop/start 的开销。
"""

import json
import os
import signal
import subprocess
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from urllib.parse import parse_qs, urlparse


LAUNCH_DIR = "/opt/carto-deploy/cartographer/launch"
MAPS_ROOT = "/opt/feedbot-maps"
LISTEN_PORT = 8097

_MODE = {"value": "idle"}
_PROC = {"value": None}
_MAP = {"value": None}


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


def stop_process():
    proc = _PROC["value"]
    if proc is not None and proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=5)
            except Exception:
                pass
    _PROC["value"] = None


def start_process(cmd, mode):
    stop_process()
    env = os.environ.copy()
    _PROC["value"] = subprocess.Popen(
        ["bash", "-lc", cmd],
        env=env,
        preexec_fn=os.setsid,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    _MODE["value"] = mode


def start_mapping():
    cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /opt/cartographer_ws/install/setup.bash && "
        "exec ros2 launch %s/cartographer_mapping.launch.py" % LAUNCH_DIR
    )
    start_process(cmd, "mapping")


def start_localization(name):
    name = (name or "").strip()
    if not name:
        raise RuntimeError("map name is required")
    pbstream = MAPS_ROOT + "/" + name + "/map.pbstream"
    cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /opt/cartographer_ws/install/setup.bash && "
        "exec ros2 launch %s/cartographer_localization.launch.py "
        "load_state_filename:=%s" % (LAUNCH_DIR, pbstream)
    )
    start_process(cmd, "localization")
    _MAP["value"] = name


def status():
    return {"mode": _MODE["value"], "map": _MAP["value"]}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def reply(self, body, code=200):
        data = json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/status":
            return self.reply(status())
        self.reply({"error": "unknown"}, 404)

    def do_POST(self):
        parsed = urlparse(self.path)
        try:
            if parsed.path == "/mapping":
                start_mapping()
                return self.reply({"ok": True, **status()})
            if parsed.path == "/localize":
                name = (parse_qs(parsed.query).get("name") or [""])[0]
                start_localization(name)
                return self.reply({"ok": True, **status()})
            return self.reply({"error": "unknown"}, 404)
        except Exception as error:
            return self.reply({"error": str(error)}, 500)


def main():
    server = ThreadingHTTPServer(("127.0.0.1", LISTEN_PORT), Handler)
    print("cartographer supervisor at 127.0.0.1:%d" % LISTEN_PORT, flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()

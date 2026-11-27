#!/usr/bin/env python3
"""Cartographer 模式管理器：在宿主侧常驻，统一管理建图/定位容器切换。

建图和定位都独占 cartographer_node 与 /scan，不能同时运行。这里用 Docker
启停两个容器，并暴露 HTTP 给导航网页：

    GET  /status              -> {"mode": "mapping"|"localization"|"idle", "map": ...}
    POST /mapping             -> 切到建图容器
    POST /localize?name=MAP   -> 切到定位容器并加载该地图
"""

import json
import subprocess
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from urllib.parse import parse_qs, urlparse


DOCKER = "/opt/feedbot-nav2/docker/docker"
SOCK = "unix:///run/feedbot-nav2-docker.sock"
IMAGE = "feedbot-cartographer:jazzy-20261122"
DEPLOY = "/opt/feedbot-cartographer/deploy"
MAPS = "/opt/feedbot-maps"
WORK = "/opt/feedbot-integration/releases/20261124-base-v16-amcl-cadence"
CONTAINER = "feedbot-cartographer"
LISTEN_PORT = 8097
CURRENT_MODE = {"value": "idle"}


ROS_ENV = [
    "-e", "ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT",
    "-e", "RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
    "-e", "RMW_FASTRTPS_PUBLICATION_MODE=SYNCHRONOUS",
    "-e", "FASTRTPS_DEFAULT_PROFILES_FILE=/work/agv_workspace/integration/mapping/config/dds_loopback.xml",
    "-e", "ROS_DOMAIN_ID=89",
    "-e", "ROS_LOCALHOST_ONLY=0",
]


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


def docker(args):
    return subprocess.run(
        [DOCKER, "-H", SOCK] + args,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        universal_newlines=True, timeout=120,
    )


def running(name):
    result = docker(["ps", "-q", "--filter", "name=" + name])
    return bool(result.stdout.strip())


def stop(name):
    if running(name):
        docker(["stop", "-t", "10", name])
    # 清理已停止的 --rm 容器残留，保证同名容器可再次启动
    docker(["rm", "-f", name])


def _run(entrypoint_cmd, mode):
    stop(CONTAINER)
    result = docker(
        ["run", "-d", "--rm", "--name", CONTAINER, "--network", "host", "--ipc", "host"]
        + ROS_ENV
        + [
            "-v", DEPLOY + ":/opt/carto-deploy:ro",
            "-v", MAPS + ":/opt/feedbot-maps:rw",
            "-v", WORK + ":/work:ro",
            IMAGE,
            "bash", "-lc", entrypoint_cmd,
        ]
    )
    if result.returncode != 0:
        raise RuntimeError((result.stderr or result.stdout)[-500:])
    CURRENT_MODE["value"] = mode


def start_mapping():
    cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /opt/cartographer_ws/install/setup.bash && "
        "exec ros2 launch /opt/carto-deploy/cartographer/launch/"
        "cartographer_mapping.launch.py"
    )
    _run(cmd, "mapping")


def start_localization(map_name):
    map_name = (map_name or "").strip()
    if not map_name:
        raise RuntimeError("map name is required")
    pbstream = MAPS + "/" + map_name + "/map.pbstream"
    cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /opt/cartographer_ws/install/setup.bash && "
        "exec ros2 launch /opt/carto-deploy/cartographer/launch/"
        "cartographer_localization.launch.py load_state_filename:=" + pbstream
    )
    _run(cmd, "localization")


def status():
    if running(CONTAINER):
        return {"mode": CURRENT_MODE["value"]}
    return {"mode": "idle"}


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
                return self.reply({"ok": True, **status(), "map": name})
            return self.reply({"error": "unknown"}, 404)
        except Exception as error:
            return self.reply({"error": str(error)}, 500)


def main():
    server = ThreadingHTTPServer(("127.0.0.1", LISTEN_PORT), Handler)
    print("cartographer manager at 127.0.0.1:%d" % LISTEN_PORT, flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import hmac
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "chassis"))
CHASSIS_BRIDGE_PATH = Path(__file__).resolve().parents[1] / "chassis" / "car_bridge.py"
CHASSIS_SPEC = importlib.util.spec_from_file_location("feedbot_chassis_bridge", CHASSIS_BRIDGE_PATH)
CHASSIS_BRIDGE = importlib.util.module_from_spec(CHASSIS_SPEC)
CHASSIS_SPEC.loader.exec_module(CHASSIS_BRIDGE)


def yaw_from_quaternion(quaternion):
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("nonfinite quaternion")
    return math.atan2(2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                      1 - 2 * (quaternion.y ** 2 + quaternion.z ** 2))


def transform_point(x, y, transform):
    yaw = yaw_from_quaternion(transform.rotation)
    return [transform.translation.x + math.cos(yaw) * x - math.sin(yaw) * y,
            transform.translation.y + math.sin(yaw) * x + math.cos(yaw) * y]


def world_to_map_cell(x, y, origin_x, origin_y, origin_yaw, resolution):
    dx = x - origin_x
    dy = y - origin_y
    return [(math.cos(origin_yaw) * dx + math.sin(origin_yaw) * dy) / resolution,
            (-math.sin(origin_yaw) * dx + math.cos(origin_yaw) * dy) / resolution]


def serialize_ranges(message, transform):
    points = []
    for index, value in enumerate(message.ranges):
        if not math.isfinite(value) or value < message.range_min or value > message.range_max:
            continue
        angle = message.angle_min + index * message.angle_increment
        points.append(transform_point(value * math.cos(angle), value * math.sin(angle), transform))
    return points


def store_scan_payload(runtime, value, received):
    with runtime.lock:
        runtime.scan_payload = {"received": received, "value": value}


class Runtime(CHASSIS_BRIDGE.Runtime):
    def __init__(self, publish, log):
        super().__init__(publish, log)
        self.map_payload = None
        self.scan_payload = None
        self.robot_payload = None
        self.transform_errors = {}
        self.save_state = {"phase": "idle"}
        self.save_lock = threading.Lock()

    def mapping_ready(self, now):
        return (self.map_payload is not None and self.scan_payload is not None and self.robot_payload is not None
                and now - self.scan_payload["received"] <= 0.5
                and now - self.robot_payload["received"] <= 0.5)

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            result = super().snapshot()
            result.update({"mapping_ready": self.mapping_ready(now),
                           "transform_errors": dict(self.transform_errors),
                           "acceptance": "代码通过；未执行实车运动、地图保存、重载或定位验收"})
            return result

    def mapping_snapshot(self):
        with self.lock:
            now = time.monotonic()
            def current(payload, maximum_age):
                if payload is None or now - payload["received"] > maximum_age:
                    return None
                result = dict(payload["value"])
                result["age_s"] = now - payload["received"]
                return result
            return {"sampled_at": now, "mapping_ready": self.mapping_ready(now),
                    "map": current(self.map_payload, math.inf), "scan": current(self.scan_payload, 0.5),
                    "robot": current(self.robot_payload, 0.5),
                    "transform_errors": dict(self.transform_errors)}

    def save_status(self):
        with self.save_lock:
            return dict(self.save_state)

    def save_now(self):
        with self.save_lock:
            if self.save_state.get("phase") == "saving":
                return 409, {"error": "already saving"}
            self.save_state = {"phase": "saving"}
        try:
            import subprocess
            import datetime
            output_root = Path("/results/maps")
            output_root.mkdir(parents=True, exist_ok=True)
            map_id = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
            output_dir = output_root / map_id
            output_dir.mkdir(parents=True, exist_ok=False)
            prefix = output_dir / "feedbot"
            command = ("source /opt/ros/jazzy/setup.bash; "
                       "ros2 run nav2_map_server map_saver_cli -f %s "
                       "--ros-args -p save_map_timeout:=15.0" % prefix)
            completed = subprocess.run(["bash", "-lc", command], capture_output=True,
                                       text=True, timeout=40)
            yaml_path = prefix.with_suffix(".yaml")
            pgm_path = prefix.with_suffix(".pgm")
            if completed.returncode != 0:
                raise RuntimeError("map_saver_cli failed: %s" % completed.stderr[-400:])
            if not yaml_path.exists() or not pgm_path.exists():
                raise RuntimeError("map_saver_cli returned but files are missing")
            result = {"id": map_id, "directory": str(output_dir),
                      "yaml": str(yaml_path), "pgm": str(pgm_path),
                      "pgm_size": pgm_path.stat().st_size}
            with self.save_lock:
                self.save_state = {"phase": "saved", "result": result}
            return 200, dict(self.save_state)
        except Exception as error:
            with self.save_lock:
                self.save_state = {"phase": "failed", "error": str(error)}
            return 503, dict(self.save_state)

class Handler(BaseHTTPRequestHandler):
    def reply(self, status, value):
        body = json.dumps(value, allow_nan=False, separators=(",", ":")).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def authenticated(self):
        return hmac.compare_digest(self.headers.get("X-Test-Token", ""), self.server.token)

    def do_GET(self):
        if not self.authenticated():
            return self.reply(403, {"error": "unauthorized"})
        if self.path == "/api/state":
            return self.reply(200, self.server.runtime.snapshot())
        if self.path == "/api/map":
            return self.reply(200, self.server.runtime.mapping_snapshot())
        if self.path == "/api/save":
            return self.reply(200, self.server.runtime.save_status())
        self.reply(404, {"error": "unknown endpoint"})

    def do_POST(self):
        if not self.authenticated():
            return self.reply(403, {"error": "unauthorized"})
        if self.path == "/api/save":
            status, value = self.server.runtime.save_now()
            return self.reply(status, value)
        if self.path != "/api/command":
            return self.reply(403, {"error": "unauthorized"})
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= 1024:
                raise ValueError("invalid command size")
            payload = json.loads(self.rfile.read(size))
            if not isinstance(payload, dict):
                raise ValueError("command must be an object")
            self.reply(200, self.server.runtime.command(payload))
        except (ValueError, TypeError) as error:
            self.reply(409, {"error": str(error)})
        except RuntimeError as error:
            self.reply(503, {"error": str(error)})

    def log_message(self, *_):
        pass


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def main():
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import JointState, LaserScan
    from tf2_ros import Buffer, TransformException, TransformListener

    token = Path(os.environ["FEEDBOT_TEST_TOKEN_FILE"]).read_text().strip()
    log_path = Path(os.environ["FEEDBOT_TEST_LOG"])
    rclpy.init()
    node = rclpy.create_node("mapping_web_manual")
    publisher = node.create_publisher(TwistStamped, "/wheel/cmd_vel", 1)
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    def publish(linear, angular):
        message = TwistStamped()
        message.header.stamp = node.get_clock().now().to_msg()
        message.twist.linear.x = linear
        message.twist.angular.z = angular
        publisher.publish(message)

    with CHASSIS_BRIDGE.EventLog(log_path.open("a")) as log:
        runtime = Runtime(publish, log)
        pending_scan = {"message": None, "received": 0.0}

        def joints(message):
            positions = dict(zip(message.name, message.position))
            if any(not math.isfinite(value) for value in positions.values()):
                raise ValueError("nonfinite joint feedback")
            runtime.update("joints", positions)

        def odometry(message):
            value = {"vx": message.twist.twist.linear.x, "wz": message.twist.twist.angular.z}
            if any(not math.isfinite(number) for number in value.values()):
                raise ValueError("nonfinite odometry")
            runtime.update("odom", value)

        def diagnostics(message):
            for status in message.status:
                if status.name == "feedbot_chassis/MCU":
                    raw_level = status.level
                    level = raw_level[0] if isinstance(raw_level, (bytes, bytearray)) else int(raw_level)
                    runtime.update("diagnostics", {"level": level, "message": status.message,
                                                   "values": {item.key: item.value for item in status.values}})

        def occupancy(message):
            origin = message.info.origin
            data = list(message.data)
            if len(data) != message.info.width * message.info.height:
                raise ValueError("occupancy data size mismatch")
            value = {"frame_id": message.header.frame_id, "stamp_s": stamp_seconds(message.header.stamp),
                     "width": message.info.width, "height": message.info.height,
                     "resolution": message.info.resolution,
                     "origin": {"x": origin.position.x, "y": origin.position.y,
                                "yaw": yaw_from_quaternion(origin.orientation)}, "data": data}
            if not all(math.isfinite(number) for number in (value["resolution"], value["origin"]["x"],
                                                             value["origin"]["y"], value["origin"]["yaw"])):
                raise ValueError("nonfinite map metadata")
            with runtime.lock:
                runtime.map_payload = {"received": time.monotonic(), "value": value}

        def lookup(frame_id, stamp, name):
            try:
                query_time = stamp if isinstance(stamp, rclpy.time.Time) else rclpy.time.Time.from_msg(stamp)
                transform = tf_buffer.lookup_transform("map", frame_id, query_time)
                with runtime.lock:
                    runtime.transform_errors.pop(name, None)
                return transform
            except TransformException as error:
                with runtime.lock:
                    runtime.transform_errors[name] = str(error)
                return None

        def scan(message):
            pending_scan["message"] = message
            pending_scan["received"] = time.monotonic()

        def process_scan():
            message = pending_scan["message"]
            if message is None:
                return
            if time.monotonic() - pending_scan["received"] > 0.5:
                with runtime.lock:
                    runtime.transform_errors["map_to_scan"] = "scan transform unavailable before frame expired"
                pending_scan["message"] = None
                return
            transform = lookup(message.header.frame_id, message.header.stamp, "map_to_scan")
            if transform is None:
                return
            value = {"frame_id": "map", "source_frame_id": message.header.frame_id,
                     "stamp_s": stamp_seconds(message.header.stamp),
                     "points": serialize_ranges(message, transform.transform)}
            store_scan_payload(runtime, value, pending_scan["received"])
            pending_scan["message"] = None

        def robot_pose():
            transform = lookup("base_link", rclpy.time.Time(), "map_to_base_link")
            if transform is None:
                return
            transform_stamp = stamp_seconds(transform.header.stamp)
            source_age = node.get_clock().now().nanoseconds * 1e-9 - transform_stamp
            if source_age < 0 or source_age > 0.5:
                with runtime.lock:
                    runtime.transform_errors["map_to_base_link"] = "transform timestamp is stale"
                return
            value = {"frame_id": "map", "stamp_s": transform_stamp,
                     "x": transform.transform.translation.x, "y": transform.transform.translation.y,
                     "yaw": yaw_from_quaternion(transform.transform.rotation)}
            with runtime.lock:
                runtime.robot_payload = {"received": time.monotonic() - source_age, "value": value}

        subscriptions = [node.create_subscription(JointState, "/joint_states", joints, 10),
                         node.create_subscription(Odometry, "/wheel/odom", odometry, 10),
                         node.create_subscription(DiagnosticArray, "/diagnostics", diagnostics, 10),
                         node.create_subscription(OccupancyGrid, "/map", occupancy, 1),
                         node.create_subscription(LaserScan, "/scan", scan, rclpy.qos.qos_profile_sensor_data)]
        timers = [node.create_timer(0.05, runtime.tick), node.create_timer(0.02, process_scan),
                  node.create_timer(0.1, robot_pose)]
        server = ThreadingHTTPServer(("10.7.5.88", 8098), Handler)
        server.token = token
        server.runtime = runtime
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        stopping = threading.Event()
        signal.signal(signal.SIGTERM, lambda *_: stopping.set())
        signal.signal(signal.SIGINT, lambda *_: stopping.set())
        try:
            while rclpy.ok() and not stopping.is_set():
                rclpy.spin_once(node, timeout_sec=0.05)
        finally:
            server.shutdown()
            server.server_close()
            if rclpy.ok():
                publish(0.0, 0.0)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()

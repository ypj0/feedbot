import hmac
import json
import math
import os
from pathlib import Path
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from control import PulseControl
from event_log import EventLog


class Runtime:
    def __init__(self, publish, log):
        self.lock = threading.RLock()
        self.control = PulseControl()
        self.publish = publish
        self.log = log
        self.samples = {}
        self.last_log = 0.0

    def event(self, kind, value):
        self.log.submit(json.dumps({"event": kind, "monotonic": time.monotonic(),
                                   "system_time": time.time(), "value": value}, allow_nan=False) + "\n")

    def update(self, name, value):
        with self.lock:
            self.samples[name] = {"received": time.monotonic(), "value": value}

    def ready(self, now):
        for name, maximum_age in (("joints", 0.3), ("odom", 0.3), ("diagnostics", 0.6)):
            sample = self.samples.get(name)
            if sample is None or now - sample["received"] > maximum_age:
                return False
        diagnostics = self.samples["diagnostics"]["value"]
        values = diagnostics["values"]
        return diagnostics["level"] == 0 and values.get("initialized") == "1" and all(
            values.get(name) == "0" for name in ("emergency", "charge", "left_alarm", "right_alarm", "shutdown"))

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            return {"sampled_at": now, "ready": self.ready(now), "control": self.control.snapshot(now),
                    "samples": {name: {"age_s": now - sample["received"], "value": sample["value"]}
                                for name, sample in self.samples.items()},
                    "covariance": "以消息中的方差为准；协方差待地面标定验证",
                    "acceptance": "实车运动通过（基础运动）；精度未量化，IMU/EKF 与导航另行验收"}

    def command(self, payload):
        with self.lock:
            if payload.get("action") != "stop":
                self.log.check()
            now = time.monotonic()
            self.control.accept(payload, now, self.ready(now))
            velocity = self.control.velocity(now, self.ready(now))
            self.publish(*velocity)
            self.event("command", {"request": payload, "body_velocity": velocity})
            return self.snapshot()

    def tick(self):
        with self.lock:
            self.log.check()
            now = time.monotonic()
            previous = self.control.action
            velocity = self.control.velocity(now, self.ready(now))
            self.publish(*velocity)
            if previous != self.control.action:
                self.event("pulse_end", {"reason": self.control.error or "duration_elapsed"})
            if now - self.last_log >= 0.1:
                self.event("telemetry", self.snapshot())
                self.last_log = now


class Handler(BaseHTTPRequestHandler):
    def reply(self, status, value):
        body = json.dumps(value, allow_nan=False).encode()
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
        if self.path != "/api/state":
            return self.reply(404, {"error": "unknown endpoint"})
        self.reply(200, self.server.runtime.snapshot())

    def do_POST(self):
        if not self.authenticated():
            return self.reply(403, {"error": "unauthorized"})
        if self.path != "/api/command":
            return self.reply(404, {"error": "unknown endpoint"})
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


def main():
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState

    token = Path(os.environ["FEEDBOT_TEST_TOKEN_FILE"]).read_text().strip()
    log_path = Path(os.environ["FEEDBOT_TEST_LOG"])
    rclpy.init()
    node = rclpy.create_node("chassis_web_test")
    publisher = node.create_publisher(TwistStamped, "/wheel/cmd_vel", 1)

    def publish(linear, angular):
        message = TwistStamped()
        message.header.stamp = node.get_clock().now().to_msg()
        message.twist.linear.x = linear
        message.twist.angular.z = angular
        publisher.publish(message)

    with EventLog(log_path.open("a")) as log:
        runtime = Runtime(publish, log)

        def joints(message):
            positions = dict(zip(message.name, message.position))
            if any(not math.isfinite(value) for value in positions.values()):
                raise ValueError("nonfinite joint feedback")
            runtime.update("joints", positions)

        def odometry(message):
            pose = message.pose.pose
            orientation = pose.orientation
            value = {"x": pose.position.x, "y": pose.position.y,
                     "yaw": math.atan2(2 * (orientation.w * orientation.z + orientation.x * orientation.y),
                                       1 - 2 * (orientation.y ** 2 + orientation.z ** 2)),
                     "vx": message.twist.twist.linear.x, "wz": message.twist.twist.angular.z}
            if any(not math.isfinite(number) for number in value.values()):
                raise ValueError("nonfinite odometry")
            value["twist_covariance"] = list(message.twist.covariance)
            runtime.update("odom", value)

        def diagnostics(message):
            for status in message.status:
                if status.name == "feedbot_chassis/MCU":
                    runtime.update("diagnostics", {"level": status.level[0], "message": status.message,
                                                   "values": {value.key: value.value for value in status.values}})

        subscriptions = [node.create_subscription(JointState, "/joint_states", joints, 10),
                         node.create_subscription(Odometry, "/wheel/odom", odometry, 10),
                         node.create_subscription(DiagnosticArray, "/diagnostics", diagnostics, 10)]
        timer = node.create_timer(0.05, runtime.tick)
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

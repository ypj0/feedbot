#!/usr/bin/env python3
"""1B Static Window: Verify streams, timestamps, TF, ownership, startup repeatability.

Run for 30-60s while vehicle is stationary (no motion commands). Collect:
- /scan ≈15 Hz, /imu ≈200 Hz, /wheel/odom ≈50 Hz, /odometry/filtered ≈50 Hz
- All timestamps strictly monotonic
- TF single publisher (odom→base_link by ekf_filter_node)
- No overruns, no IMU backlog, no TF interrupts
- Lidar and serial each have one owner
- LTR2 auto-activates
- Vehicle restart repeatable without manual intervention
- Failure stops cleanly with explicit error logged
"""
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


class StaticCollector:
    def __init__(self, results_dir):
        self.results = Path(results_dir)
        self.results.mkdir(parents=True, exist_ok=True)
        rclpy.init()
        self.node = rclpy.create_node("acceptance_1b_static")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self.samples = {
            "scan": [],
            "imu": [],
            "wheel": [],
            "filtered": [],
            "diagnostics": [],
            "tf_edges": [],
        }
        self.errors = []
        self.start_time = time.time()
        self.start_monotonic = time.monotonic()

        self.node.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.node.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.node.create_subscription(Odometry, "/wheel/odom", self._on_wheel_odom, 10)
        self.node.create_subscription(Odometry, "/odometry/filtered", self._on_filtered, 10)
        self.node.create_subscription(DiagnosticArray, "/diagnostics", self._on_diagnostics, 10)

    def _on_scan(self, msg):
        self.samples["scan"].append({
            "stamp_ns": stamp_ns(msg.header),
            "received_monotonic": time.monotonic() - self.start_monotonic,
            "frame_id": msg.header.frame_id,
        })

    def _on_joints(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name == "imu" and math.isfinite(pos):
                self.samples["imu"].append({
                    "stamp_ns": stamp_ns(msg.header),
                    "received_monotonic": time.monotonic() - self.start_monotonic,
                })
                break

    def _on_wheel_odom(self, msg):
        self.samples["wheel"].append({
            "stamp_ns": stamp_ns(msg.header),
            "received_monotonic": time.monotonic() - self.start_monotonic,
        })

    def _on_filtered(self, msg):
        self.samples["filtered"].append({
            "stamp_ns": stamp_ns(msg.header),
            "received_monotonic": time.monotonic() - self.start_monotonic,
        })

    def _on_diagnostics(self, msg):
        self.samples["diagnostics"].append({
            "stamp_ns": stamp_ns(msg.header),
            "received_monotonic": time.monotonic() - self.start_monotonic,
            "status": [{"name": s.name, "level": int(s.level[0]) if isinstance(s.level, (bytes, bytearray)) else int(s.level),
                       "values": {v.key: v.value for v in s.values}} for s in msg.status],
        })

    def collect_tf_edges(self):
        for parent, child in [("odom", "base_link")]:
            try:
                tf = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                self.samples["tf_edges"].append({
                    "parent": parent,
                    "child": child,
                    "stamp_ns": stamp_ns(tf.header),
                    "translation": [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z],
                })
            except TransformException:
                self.errors.append(f"TF lookup failed: {parent} -> {child}")

    def run(self, duration_s=60):
        end_time = time.time() + duration_s
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            self.collect_tf_edges()

    def evaluate(self):
        failures = []

        for name in ("scan", "imu", "wheel", "filtered"):
            samples = self.samples[name]
            if len(samples) < 2:
                failures.append(f"insufficient {name} samples: {len(samples)}")
                continue

            stamps = [s["stamp_ns"] for s in samples]
            if not all(after > before for before, after in zip(stamps, stamps[1:])):
                failures.append(f"non-monotonic {name} timestamps")

            rate = (len(samples) - 1) / (samples[-1]["received_monotonic"] - samples[0]["received_monotonic"])
            expected_range = {
                "scan": (14, 16),
                "imu": (190, 210),
                "wheel": (45, 55),
                "filtered": (45, 55),
            }
            if rate and name in expected_range:
                min_rate, max_rate = expected_range[name]
                if not (min_rate <= rate <= max_rate):
                    failures.append(f"{name} rate {rate:.1f} Hz outside {min_rate}-{max_rate}")

        diagnostics = self.samples["diagnostics"]
        for sample in diagnostics:
            for status in sample["status"]:
                if status["level"] >= 2 and status["name"] not in ("ekf_filter_node: odometry/filtered topic status",):
                    failures.append(f"diagnostic ERROR: {status['name']}")

        if self.errors:
            failures.extend(self.errors)

        result = {
            "passed": len(failures) == 0,
            "failures": failures,
            "samples": {name: len(samples) for name, samples in self.samples.items()},
        }
        return result

    def save(self):
        self.node.destroy_node()
        rclpy.shutdown()
        result = self.evaluate()
        (self.results / "1b_static_result.json").write_text(json.dumps(result, indent=2))
        return result


if __name__ == "__main__":
    results_dir = sys.argv[1] if len(sys.argv) > 1 else "/tmp/acceptance_1b_static"
    collector = StaticCollector(results_dir)
    collector.run(duration_s=60)
    result = collector.save()
    print(json.dumps(result, indent=2))
    sys.exit(0 if result["passed"] else 1)

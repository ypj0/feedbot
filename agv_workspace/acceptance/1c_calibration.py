#!/usr/bin/env python3
"""1C Motion Calibration: Passive wheel, ground-truth distance, angle measurement.

After 1B ground window: if distance/angle errors < 2%, optionally collect additional
samples for statistical confidence. Otherwise, manually measure passive wheel diameter
and update if needed.
"""
import json
import math
import sys
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from feedbot_motion_interfaces.msg import MotionRequest
from feedbot_motion_interfaces.srv import AcquireLease


class CalibrationCollector:
    def __init__(self, results_dir):
        self.results = Path(results_dir)
        self.results.mkdir(parents=True, exist_ok=True)
        rclpy.init()
        self.node = rclpy.create_node("acceptance_1c_calibration")

        self.lease_id = None
        self.odom_runs = []
        self.errors = []

        # Acquire lease
        lease_client = self.node.create_client(AcquireLease, "/motion/acquire_lease")
        while rclpy.ok() and not lease_client.wait_for_service(timeout_sec=0.5):
            pass
        req = AcquireLease.Request()
        req.source = 1
        future = lease_client.call_async(req)
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.01)
        result = future.result()
        if result.granted:
            self.lease_id = result.lease_id
        else:
            self.errors.append(f"Failed to acquire lease: {result.reason}")

        self.motion_pub = self.node.create_publisher(MotionRequest, "/motion/request", 1)
        self.node.create_subscription(Odometry, "/wheel/odom", self._on_odom, 10)

    def _on_odom(self, msg):
        if self.odom_runs:
            self.odom_runs[-1].append({
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "yaw": self._yaw_from_quat(msg.pose.pose.orientation),
            })

    @staticmethod
    def _yaw_from_quat(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))

    def run_trial(self, distance_m, speed_mps):
        import time
        self.odom_runs.append([])
        duration = distance_m / speed_mps
        for _ in range(int(duration * 50)):
            if self.lease_id is None:
                break
            msg = MotionRequest()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.source = 1
            msg.lease_id = self.lease_id
            msg.seq = 0
            msg.twist.linear.x = speed_mps
            msg.twist.angular.z = 0.0
            self.motion_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.02)
        time.sleep(0.5)
        # Stop
        msg = MotionRequest()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.source = 1
        msg.lease_id = self.lease_id
        msg.seq = 0
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.motion_pub.publish(msg)

    def run(self):
        import time
        print("=" * 60)
        print("1C Motion Calibration - Multi-run validation")
        print("=" * 60)
        print("\nRunning 3× trials of 5m straight at 0.1 m/s")

        for trial in range(3):
            print(f"\nTrial {trial + 1}: Mark start, press ENTER")
            input("   Begin motion...")
            self.run_trial(5.0, 0.1)
            time.sleep(0.5)
            input("   Mark end, press ENTER")

        results = []
        for i, run in enumerate(self.odom_runs):
            if len(run) > 1:
                dist = math.sqrt(
                    (run[-1]["x"] - run[0]["x"]) ** 2 +
                    (run[-1]["y"] - run[0]["y"]) ** 2
                )
                error_pct = abs(dist - 5.0) / 5.0 * 100
                results.append({"trial": i + 1, "distance_m": dist, "error_pct": error_pct})
                print(f"Trial {i + 1}: {dist:.3f}m (error: {error_pct:.1f}%)")

        return results

    def save(self, results):
        self.node.destroy_node()
        rclpy.shutdown()
        mean_error = sum(r["error_pct"] for r in results) / len(results) if results else 100
        result = {
            "passed": mean_error < 1.0,
            "trials": results,
            "mean_error_pct": mean_error,
            "errors": self.errors,
            "note": "If mean error < 1%, calibration complete. Otherwise, check wheel_radius in URDF/config.",
        }
        (self.results / "1c_calibration_result.json").write_text(json.dumps(result, indent=2))
        return result


if __name__ == "__main__":
    results_dir = sys.argv[1] if len(sys.argv) > 1 else "/tmp/acceptance_1c_calibration"
    collector = CalibrationCollector(results_dir)
    try:
        trials = collector.run()
    except KeyboardInterrupt:
        trials = []
    result = collector.save(trials)
    print(json.dumps(result, indent=2))
    sys.exit(0 if result["passed"] else 1)

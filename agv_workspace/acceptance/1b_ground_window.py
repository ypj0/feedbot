#!/usr/bin/env python3
"""1B Ground Window: Known distance/angle, record odometry error.

Measure ground-truth distance (5m straight) and angle (90° turn), command low speed,
record /wheel/odom position/heading. Calculate error. If error < 2%, update
controllers.yaml, chassis.urdf, session.hpp wheel_radius and wheel_separation.
"""
import json
import math
import sys
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from feedbot_motion_interfaces.msg import MotionRequest
from feedbot_motion_interfaces.srv import AcquireLease


class GroundCollector:
    def __init__(self, results_dir):
        self.results = Path(results_dir)
        self.results.mkdir(parents=True, exist_ok=True)
        rclpy.init()
        self.node = rclpy.create_node("acceptance_1b_ground")

        self.lease_id = None
        self.odom_samples = []
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
        self.odom_samples.append({
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "yaw": self._yaw_from_quat(msg.pose.pose.orientation),
        })

    @staticmethod
    def _yaw_from_quat(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))

    def send_motion(self, linear, angular, duration_s):
        import time
        for _ in range(int(duration_s * 50)):  # 50 Hz
            if self.lease_id is None:
                break
            msg = MotionRequest()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.source = 1
            msg.lease_id = self.lease_id
            msg.seq = 0
            msg.twist.linear.x = linear
            msg.twist.angular.z = angular
            self.motion_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.send_motion(0.0, 0.0, 0.0)

    def run(self):
        import time
        print("=" * 60)
        print("1B Ground Window - Odometry Calibration")
        print("=" * 60)
        print("\n1. STRAIGHT: Command 5m at 0.1 m/s")
        input("   Mark start line. Press ENTER to begin, then mark end when vehicle stops.")
        self.odom_samples.clear()
        self.send_motion(0.1, 0.0, 50)  # 5m at 0.1 m/s
        time.sleep(0.5)
        self.send_motion(0.0, 0.0, 1)

        if len(self.odom_samples) > 1:
            dist_odometry = math.sqrt(
                (self.odom_samples[-1]["x"] - self.odom_samples[0]["x"]) ** 2 +
                (self.odom_samples[-1]["y"] - self.odom_samples[0]["y"]) ** 2
            )
            ground_truth_dist = 5.0
            dist_error_pct = abs(dist_odometry - ground_truth_dist) / ground_truth_dist * 100
            print(f"   Measured: {dist_odometry:.3f}m (error: {dist_error_pct:.1f}%)")
        else:
            self.errors.append("No odometry samples during straight motion")
            dist_error_pct = 100

        print("\n2. TURN: Command 90° at 0.2 rad/s")
        input("   Mark start heading. Press ENTER to begin, then mark end when done.")
        self.odom_samples.clear()
        angle_command = 90 * math.pi / 180
        duration = angle_command / 0.2
        self.send_motion(0.0, 0.2, duration)
        time.sleep(0.5)
        self.send_motion(0.0, 0.0, 1)

        if len(self.odom_samples) > 1:
            angle_odometry = abs(self.odom_samples[-1]["yaw"] - self.odom_samples[0]["yaw"])
            angle_odometry = min(angle_odometry, 2 * math.pi - angle_odometry)
            ground_truth_angle = math.pi / 2
            angle_error_pct = abs(angle_odometry - ground_truth_angle) / ground_truth_angle * 100
            print(f"   Measured: {math.degrees(angle_odometry):.1f}° (error: {angle_error_pct:.1f}%)")
        else:
            self.errors.append("No odometry samples during turn")
            angle_error_pct = 100

        return dist_error_pct, angle_error_pct

    def save(self, dist_error, angle_error):
        self.node.destroy_node()
        rclpy.shutdown()
        result = {
            "passed": dist_error < 2.0 and angle_error < 2.0,
            "distance_error_pct": dist_error,
            "angle_error_pct": angle_error,
            "errors": self.errors,
            "note": "If passed, update wheel_radius/separation in controllers.yaml, chassis.urdf, session.hpp",
        }
        (self.results / "1b_ground_result.json").write_text(json.dumps(result, indent=2))
        return result


if __name__ == "__main__":
    results_dir = sys.argv[1] if len(sys.argv) > 1 else "/tmp/acceptance_1b_ground"
    collector = GroundCollector(results_dir)
    try:
        dist_err, angle_err = collector.run()
    except KeyboardInterrupt:
        dist_err, angle_err = 100, 100
    result = collector.save(dist_err, angle_err)
    print(json.dumps(result, indent=2))
    sys.exit(0 if result["passed"] else 1)

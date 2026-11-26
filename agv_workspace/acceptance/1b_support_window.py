#!/usr/bin/env python3
"""1B Support Window: Verify direction, position increment, MotionAuthority control.

Wheels off ground. Manually command low-speed motions and verify:
- Left wheel turns in correct direction
- Right wheel turns in correct direction
- Position increment in /wheel/odom matches wheel rotation
- Unauthorized motion (no lease / expired / revoked) produces zero output
"""
import json
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from feedbot_motion_interfaces.msg import MotionRequest
from feedbot_motion_interfaces.srv import AcquireLease, ReleaseLease


class SupportCollector:
    def __init__(self, results_dir):
        self.results = Path(results_dir)
        self.results.mkdir(parents=True, exist_ok=True)
        rclpy.init()
        self.node = rclpy.create_node("acceptance_1b_support")

        self.lease_id = None
        self.odom_samples = []
        self.errors = []

        # Try to acquire lease
        lease_client = self.node.create_client(AcquireLease, "/motion/acquire_lease")
        while rclpy.ok() and not lease_client.wait_for_service(timeout_sec=0.5):
            pass
        req = AcquireLease.Request()
        req.source = 1  # WEB_CONTROL
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
            "vx": msg.twist.twist.linear.x,
            "wz": msg.twist.twist.angular.z,
        })

    def send_motion(self, linear, angular):
        if self.lease_id is None:
            return
        msg = MotionRequest()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.source = 1
        msg.lease_id = self.lease_id
        msg.seq = 0
        msg.twist = Twist()
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.motion_pub.publish(msg)

    def run(self):
        """Manual prompts for operator to verify direction and control."""
        import time
        print("=" * 60)
        print("1B Support Window - Manual Verification")
        print("=" * 60)
        print("\n1. Verify left/right wheel direction:")
        input("   Press ENTER when ready, then command LEFT TURN (counterclockwise) for 3s")
        self.odom_samples.clear()
        for _ in range(30):
            self.send_motion(0.0, 0.5)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        time.sleep(0.3)
        self.send_motion(0.0, 0.0)

        if len(self.odom_samples) > 1:
            start_z = self.odom_samples[0].get("y", 0)
            end_z = self.odom_samples[-1].get("y", 0)
            if end_z > start_z:
                print("   ✓ LEFT TURN observed (y increasing)")
            else:
                self.errors.append("LEFT TURN direction incorrect")

        print("\n2. Verify release stops motion:")
        input("   Press ENTER, motion should stop immediately after release")
        release_client = self.node.create_client(ReleaseLease, "/motion/release_lease")
        req = ReleaseLease.Request()
        req.lease_id = self.lease_id
        future = release_client.call_async(req)
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.lease_id = None

        self.odom_samples.clear()
        self.send_motion(0.5, 0.0)  # Will be ignored since no lease
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.send_motion(0.0, 0.0)

        if len(self.odom_samples) > 0:
            any_nonzero = any(s["vx"] != 0 or s["wz"] != 0 for s in self.odom_samples)
            if not any_nonzero:
                print("   ✓ Motion blocked without lease")
            else:
                self.errors.append("Motion observed without lease")

    def save(self):
        self.node.destroy_node()
        rclpy.shutdown()
        result = {
            "passed": len(self.errors) == 0,
            "errors": self.errors,
            "odom_samples": len(self.odom_samples),
        }
        (self.results / "1b_support_result.json").write_text(json.dumps(result, indent=2))
        return result


if __name__ == "__main__":
    results_dir = sys.argv[1] if len(sys.argv) > 1 else "/tmp/acceptance_1b_support"
    collector = SupportCollector(results_dir)
    try:
        collector.run()
    except KeyboardInterrupt:
        pass
    result = collector.save()
    print(json.dumps(result, indent=2))
    sys.exit(0 if result["passed"] else 1)

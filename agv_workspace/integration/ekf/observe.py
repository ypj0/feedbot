import json
import math
from pathlib import Path
import statistics
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage


def stamp(message):
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


def odometry(message):
    position = message.pose.pose.position
    rotation = message.pose.pose.orientation
    yaw = math.atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y),
                     1 - 2 * (rotation.y**2 + rotation.z**2))
    return {"stamp_ns": stamp(message), "x": position.x, "y": position.y, "yaw": yaw,
            "vx": message.twist.twist.linear.x, "wz": message.twist.twist.angular.z,
            "vx_variance": message.twist.covariance[0], "wz_variance": message.twist.covariance[35]}


def main():
    path = Path(sys.argv[1])
    duration = float(sys.argv[2])
    rclpy.init()
    node = rclpy.create_node("feedbot_readonly_observer")
    data = {"imu": [], "wheel": [], "filtered": [], "diagnostics": [], "tf": [], "tf_static": []}
    with path.open("x") as output:
        def record(kind, value):
            value["monotonic"] = time.monotonic()
            data[kind].append(value)
            output.write(json.dumps({"kind": kind, **value}) + "\n")

        def imu(message):
            record("imu", {"stamp_ns": stamp(message), "frame": message.header.frame_id,
                           "wx": message.angular_velocity.x, "wy": message.angular_velocity.y,
                           "wz": message.angular_velocity.z, "wz_variance": message.angular_velocity_covariance[8],
                           "orientation_available": bool(message.orientation_covariance[0] != -1),
                           "acceleration_available": bool(message.linear_acceleration_covariance[0] != -1)})

        def diagnostics(message):
            record("diagnostics", {"stamp_ns": stamp(message), "status": [
                {"name": status.name, "level": status.level[0], "message": status.message,
                 "values": {value.key: value.value for value in status.values}} for status in message.status]})

        def transforms(kind, message):
            for transform in message.transforms:
                record(kind, {"stamp_ns": stamp(transform), "parent": transform.header.frame_id,
                              "child": transform.child_frame_id})

        subscriptions = [
            node.create_subscription(Imu, "/imu", imu, qos_profile_sensor_data),
            node.create_subscription(Odometry, "/wheel/odom", lambda message: record("wheel", odometry(message)), 100),
            node.create_subscription(Odometry, "/odometry/filtered", lambda message: record("filtered", odometry(message)), 100),
            node.create_subscription(DiagnosticArray, "/diagnostics", diagnostics, 100),
            node.create_subscription(TFMessage, "/tf", lambda message: transforms("tf", message), 100),
            node.create_subscription(TFMessage, "/tf_static", lambda message: transforms("tf_static", message),
                                     QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        ]
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        endpoints = {topic: [{"gid": bytes(info.endpoint_gid).hex(), "node": info.node_name,
                             "namespace": info.node_namespace} for info in node.get_publishers_info_by_topic(topic)]
                     for topic in ("/tf", "/tf_static", "/imu", "/wheel/odom", "/odometry/filtered")}
    result = {"duration_requested_s": duration, "endpoints": endpoints, "counts": {key: len(value) for key, value in data.items()}}
    for name in ("imu", "wheel", "filtered"):
        samples = data[name]
        if len(samples) < 2:
            continue
        stamps = [value["stamp_ns"] for value in samples]
        intervals = [(after - before) / 1e9 for before, after in zip(stamps, stamps[1:])]
        result[name] = {"rate_hz": (len(samples) - 1) / ((stamps[-1] - stamps[0]) / 1e9),
                        "max_stamp_gap_s": max(intervals), "strictly_monotonic": min(intervals) > 0,
                        "wz_mean": statistics.mean(value["wz"] for value in samples),
                        "wz_stddev": statistics.stdev(value["wz"] for value in samples),
                        "wz_min": min(value["wz"] for value in samples), "wz_max": max(value["wz"] for value in samples),
                        "wz_variances": sorted({value["wz_variance"] for value in samples})}
        if name != "imu":
            result[name]["max_displacement_from_first_m"] = max(math.hypot(
                value["x"] - samples[0]["x"], value["y"] - samples[0]["y"]) for value in samples)
            result[name]["max_yaw_change_from_first_rad"] = max(abs(math.remainder(
                value["yaw"] - samples[0]["yaw"], 2 * math.pi)) for value in samples)
    result["edge_provenance"] = "Use companion tf_edge_probe C++ capture; Python MessageInfo has no GID in this image"
    path.with_suffix(".summary.json").write_text(json.dumps(result, indent=2) + "\n")
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()

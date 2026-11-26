#!/usr/bin/env python3
import json
import os
import select
import socket

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry


SOCKET_PATH = "/navcfg/bridge/pose.sock"


def publish_pose(data, publisher):
    message = PoseStamped()
    message.header.stamp.sec = int(data["sec"])
    message.header.stamp.nanosec = int(data["nsec"])
    message.header.frame_id = data.get("frame_id", "map").lstrip("/")
    message.pose.position.x = float(data["x"])
    message.pose.position.y = float(data["y"])
    message.pose.position.z = float(data["z"])
    message.pose.orientation.x = float(data["qx"])
    message.pose.orientation.y = float(data["qy"])
    message.pose.orientation.z = float(data["qz"])
    message.pose.orientation.w = float(data["qw"])
    publisher.publish(message)


def publish_scan(data, publisher):
    message = LaserScan()
    message.header.stamp.sec = int(data["sec"])
    message.header.stamp.nanosec = int(data["nsec"])
    message.header.frame_id = data.get("frame_id", "scan").lstrip("/")
    message.angle_min = float(data["angle_min"])
    message.angle_max = float(data["angle_max"])
    message.angle_increment = float(data["angle_increment"])
    message.time_increment = float(data["time_increment"])
    message.scan_time = float(data["scan_time"])
    message.range_min = float(data["range_min"])
    message.range_max = float(data["range_max"])
    message.ranges = [float(value) for value in data["ranges"]]
    publisher.publish(message)


def publish_tf(data, publisher):
    message = TFMessage()
    for transform in data["transforms"]:
        item = TransformStamped()
        item.header.stamp.sec = int(transform["sec"])
        item.header.stamp.nanosec = int(transform["nsec"])
        item.header.frame_id = transform["frame_id"].lstrip("/")
        item.child_frame_id = transform["child_frame_id"].lstrip("/")
        item.transform.translation.x = float(transform["tx"])
        item.transform.translation.y = float(transform["ty"])
        item.transform.translation.z = float(transform["tz"])
        item.transform.rotation.x = float(transform["rx"])
        item.transform.rotation.y = float(transform["ry"])
        item.transform.rotation.z = float(transform["rz"])
        item.transform.rotation.w = float(transform["rw"])
        message.transforms.append(item)
    publisher.publish(message)


def publish_odom(data, publisher):
    message = Odometry()
    message.header.stamp.sec = int(data["sec"])
    message.header.stamp.nanosec = int(data["nsec"])
    message.header.frame_id = data.get("frame_id", "odom").lstrip("/")
    message.child_frame_id = data.get("child_frame_id", "base_footprint").lstrip("/")
    message.pose.pose.position.x = float(data["x"])
    message.pose.pose.position.y = float(data["y"])
    message.pose.pose.position.z = float(data["z"])
    message.pose.pose.orientation.x = float(data["qx"])
    message.pose.pose.orientation.y = float(data["qy"])
    message.pose.pose.orientation.z = float(data["qz"])
    message.pose.pose.orientation.w = float(data["qw"])
    message.twist.twist.linear.x = float(data["vx"])
    message.twist.twist.linear.y = float(data["vy"])
    message.twist.twist.linear.z = float(data["vz"])
    message.twist.twist.angular.x = float(data["wx"])
    message.twist.twist.angular.y = float(data["wy"])
    message.twist.twist.angular.z = float(data["wz"])
    publisher.publish(message)


def main():
    rclpy.init()
    node = rclpy.create_node("original_pose_relay")
    pose_publisher = node.create_publisher(PoseStamped, "/original/tracked_pose", 10)
    scan_publisher = node.create_publisher(LaserScan, "/original/scan", 10)
    tf_publisher = node.create_publisher(TFMessage, "/original/tf", 10)
    odom_publisher = node.create_publisher(Odometry, "/original/odom", 10)
    live_scan_publisher = node.create_publisher(LaserScan, "/scan", 10)
    live_tf_publisher = node.create_publisher(TFMessage, "/tf", 10)
    live_odom_publisher = node.create_publisher(Odometry, "/odom", 10)

    connection = None
    pending = b""
    last_retry = 0.0

    def send_command(message):
        if connection is None:
            return
        payload = json.dumps({
            "topic": "/cmd_vel_smoothed",
            "linear_x": message.linear.x,
            "linear_y": message.linear.y,
            "linear_z": message.linear.z,
            "angular_x": message.angular.x,
            "angular_y": message.angular.y,
            "angular_z": message.angular.z,
        }, separators=(",", ":")) + "\n"
        try:
            connection.sendall(payload.encode("utf-8"))
        except OSError:
            pass

    node.create_subscription(Twist, "/cmd_vel_smoothed", send_command, 10)

    def handle_line(line):
        try:
            data = json.loads(line.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return
        topic = data.get("topic")
        if topic == "/tracked_pose":
            publish_pose(data, pose_publisher)
        elif topic == "/scan":
            publish_scan(data, scan_publisher)
            publish_scan(data, live_scan_publisher)
        elif topic == "/tf":
            publish_tf(data, tf_publisher)
            publish_tf(data, live_tf_publisher)
        elif topic == "/wheel_odom":
            publish_odom(data, odom_publisher)
            publish_odom(data, live_odom_publisher)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            now = node.get_clock().now().nanoseconds * 1e-9
            if connection is None:
                if os.path.exists(SOCKET_PATH) and now - last_retry >= 0.5:
                    last_retry = now
                    try:
                        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                        sock.settimeout(0.5)
                        sock.connect(SOCKET_PATH)
                        sock.setblocking(False)
                        connection = sock
                        pending = b""
                        node.get_logger().info("original pose socket connected")
                    except OSError:
                        connection = None
                continue
            readable, _, _ = select.select([connection], [], [], 0)
            if not readable:
                continue
            try:
                chunk = connection.recv(65536)
            except OSError:
                chunk = b""
            if not chunk:
                connection.close()
                connection = None
                pending = b""
                node.get_logger().warn("original pose socket disconnected")
                continue
            pending += chunk
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                if line:
                    handle_line(line)
    finally:
        if connection is not None:
            try:
                connection.close()
            except OSError:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

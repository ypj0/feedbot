#!/usr/bin/env python2
import json
import os
import socket
import sys
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry


SOCKET_PATH = "/opt/feedbot-navigation/bridge/pose.sock"


def make_server(path):
    try:
        os.unlink(path)
    except Exception:
        pass
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(path)
    server.listen(1)
    return server


def main():
    rospy.init_node("original_pose_relay", anonymous=False)
    server = make_server(SOCKET_PATH)
    state = {"client": None}
    command_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    command_lock = threading.Lock()
    command_queue = []

    def reader_loop(connection):
        print("reader start")
        sys.stdout.flush()
        pending = b""
        while not rospy.is_shutdown():
            try:
                chunk = connection.recv(4096)
            except Exception:
                break
            if not chunk:
                break
            print("reader recv", repr(chunk[:120]))
            sys.stdout.flush()
            pending += chunk
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                if not line:
                    continue
                try:
                    data = json.loads(line.decode("utf-8"))
                except (ValueError, UnicodeDecodeError):
                    continue
                if data.get("topic") == "/cmd_vel_smoothed":
                    print("reader got cmd_vel_smoothed", data.get("linear_x"), data.get("angular_z"))
                    sys.stdout.flush()
                    with command_lock:
                        command_queue.append((float(data.get("linear_x", 0.0)),
                                              float(data.get("angular_z", 0.0))))

    def accept_loop():
        while not rospy.is_shutdown():
            connection, _ = server.accept()
            previous = state.get("client")
            if previous is not None:
                try:
                    previous.close()
                except Exception:
                    pass
            state["client"] = connection
            print("accepted client")
            sys.stdout.flush()
            threading.Thread(target=reader_loop, args=(connection,)).start()

    thread = threading.Thread(target=accept_loop)
    thread.daemon = True
    thread.start()

    def send(message):
        connection = state.get("client")
        if connection is None:
            return
        try:
            connection.sendall(json.dumps(message, separators=(",", ":")) + "\n")
        except Exception:
            try:
                connection.close()
            except Exception:
                pass
            state["client"] = None

    def drain_commands(_):
        with open("/tmp/relay_debug.log", "a") as handle:
            handle.write("drain_tick queue=%d\n" % len(command_queue))
        while command_queue:
            with command_lock:
                if not command_queue:
                    break
                linear_x, angular_z = command_queue.pop(0)
            with open("/tmp/relay_debug.log", "a") as handle:
                handle.write("publish_cmd %s %s\n" % (linear_x, angular_z))
            print("drain cmd_vel", linear_x, angular_z)
            sys.stdout.flush()
            message = Twist()
            message.linear.x = linear_x
            message.linear.y = 0.0
            message.linear.z = 0.0
            message.angular.x = 0.0
            message.angular.y = 0.0
            message.angular.z = angular_z
            command_publisher.publish(message)

    command_timer = rospy.Timer(rospy.Duration(0.02), drain_commands)

    def pose_callback(message):
        stamp = message.header.stamp
        send({
            "topic": "/tracked_pose",
            "sec": stamp.secs,
            "nsec": stamp.nsecs,
            "frame_id": message.header.frame_id,
            "x": message.pose.position.x,
            "y": message.pose.position.y,
            "z": message.pose.position.z,
            "qx": message.pose.orientation.x,
            "qy": message.pose.orientation.y,
            "qz": message.pose.orientation.z,
            "qw": message.pose.orientation.w,
        })

    def scan_callback(message):
        send({
            "topic": "/scan",
            "sec": message.header.stamp.secs,
            "nsec": message.header.stamp.nsecs,
            "frame_id": message.header.frame_id,
            "angle_min": message.angle_min,
            "angle_max": message.angle_max,
            "angle_increment": message.angle_increment,
            "time_increment": message.time_increment,
            "scan_time": message.scan_time,
            "range_min": message.range_min,
            "range_max": message.range_max,
            "ranges": list(message.ranges),
        })

    def tf_callback(message):
        transforms = []
        for transform in message.transforms:
            transforms.append({
                "sec": transform.header.stamp.secs,
                "nsec": transform.header.stamp.nsecs,
                "frame_id": transform.header.frame_id,
                "child_frame_id": transform.child_frame_id,
                "tx": transform.transform.translation.x,
                "ty": transform.transform.translation.y,
                "tz": transform.transform.translation.z,
                "rx": transform.transform.rotation.x,
                "ry": transform.transform.rotation.y,
                "rz": transform.transform.rotation.z,
                "rw": transform.transform.rotation.w,
            })
        send({"topic": "/tf", "transforms": transforms})

    def tf_static_callback(message):
        transforms = []
        for transform in message.transforms:
            transforms.append({
                "sec": transform.header.stamp.secs,
                "nsec": transform.header.stamp.nsecs,
                "frame_id": transform.header.frame_id,
                "child_frame_id": transform.child_frame_id,
                "tx": transform.transform.translation.x,
                "ty": transform.transform.translation.y,
                "tz": transform.transform.translation.z,
                "rx": transform.transform.rotation.x,
                "ry": transform.transform.rotation.y,
                "rz": transform.transform.rotation.z,
                "rw": transform.transform.rotation.w,
            })
        send({"topic": "/tf_static", "transforms": transforms})

    def odom_callback(message):
        send({
            "topic": "/wheel_odom",
            "sec": message.header.stamp.secs,
            "nsec": message.header.stamp.nsecs,
            "frame_id": message.header.frame_id,
            "child_frame_id": message.child_frame_id,
            "x": message.pose.pose.position.x,
            "y": message.pose.pose.position.y,
            "z": message.pose.pose.position.z,
            "qx": message.pose.pose.orientation.x,
            "qy": message.pose.pose.orientation.y,
            "qz": message.pose.pose.orientation.z,
            "qw": message.pose.pose.orientation.w,
            "vx": message.twist.twist.linear.x,
            "vy": message.twist.twist.linear.y,
            "vz": message.twist.twist.linear.z,
            "wx": message.twist.twist.angular.x,
            "wy": message.twist.twist.angular.y,
            "wz": message.twist.twist.angular.z,
        })

    rospy.Subscriber("/tracked_pose", PoseStamped, pose_callback, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, scan_callback, queue_size=1)
    rospy.Subscriber("/tf", TFMessage, tf_callback, queue_size=10)
    rospy.Subscriber("/tf_static", TFMessage, tf_static_callback, queue_size=10)
    rospy.Subscriber("/wheel_odom", Odometry, odom_callback, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    main()

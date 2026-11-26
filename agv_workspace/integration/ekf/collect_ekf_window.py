#!/usr/bin/env python3
import json
import os
from pathlib import Path
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


PROCESS_ARGV0_NAMES = ("imu_node", "ekf_node", "ros2_control_node")


def message_stamp(message):
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


def read_text(path):
    try:
        return Path(path).read_text()
    except (OSError, UnicodeDecodeError):
        return None


def cpu_times():
    raw_stat = read_text("/proc/stat")
    if raw_stat is None:
        return None
    values = [int(value) for value in raw_stat.splitlines()[0].split()[1:]]
    return {"total": sum(values[:8]), "idle": values[3] + values[4]}


def thread_record(process_id, thread_id):
    stat = read_text("/proc/{}/task/{}/stat".format(process_id, thread_id))
    if stat is None:
        return None
    fields = stat[stat.rfind(")") + 2:].split()
    name = read_text("/proc/{}/task/{}/comm".format(process_id, thread_id))
    if name is None:
        return None
    try:
        affinity = sorted(os.sched_getaffinity(thread_id))
        scheduler = os.sched_getscheduler(thread_id)
        scheduler_priority = os.sched_getparam(thread_id).sched_priority
    except (OSError, ProcessLookupError):
        return None
    return {
        "tid": thread_id,
        "name": name.strip(),
        "state": fields[0],
        "priority": int(fields[15]),
        "nice": int(fields[16]),
        "startticks": int(fields[19]),
        "processor": int(fields[36]),
        "rt_priority": int(fields[37]),
        "policy": int(fields[38]),
        "scheduler": scheduler,
        "scheduler_priority": scheduler_priority,
        "cpu_affinity": affinity,
    }


def process_records():
    records = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        raw_command = read_text(entry / "cmdline")
        if raw_command is None:
            continue
        arguments = raw_command.split("\0")
        if not arguments or not arguments[0]:
            continue
        argv0_name = Path(arguments[0]).name
        if argv0_name not in PROCESS_ARGV0_NAMES:
            continue
        stat = read_text(entry / "stat")
        if stat is None:
            continue
        fields = stat[stat.rfind(")") + 2:].split()
        try:
            thread_ids = sorted(int(item.name) for item in (entry / "task").iterdir())
        except OSError:
            continue
        threads = []
        for thread_id in thread_ids:
            record = thread_record(int(entry.name), thread_id)
            if record is not None:
                threads.append(record)
        records.append({
            "argv0_name": argv0_name,
            "pid": int(entry.name),
            "startticks": int(fields[19]),
            "argv": arguments[:-1] if arguments[-1] == "" else arguments,
            "thread_count": len(threads),
            "threads": threads,
        })
    return records


def phase_value(path):
    value = read_text(path)
    return value.strip() if value is not None else "unknown"


def signal_ready(path):
    descriptor = os.open(str(path), os.O_WRONLY | os.O_NONBLOCK)
    try:
        os.write(descriptor, b"ready\n")
    finally:
        os.close(descriptor)


def odometry_record(message):
    return {
        "stamp_ns": message_stamp(message),
        "frame": message.header.frame_id,
        "child_frame": message.child_frame_id,
        "pose": {
            "position": [message.pose.pose.position.x, message.pose.pose.position.y,
                         message.pose.pose.position.z],
            "orientation": [message.pose.pose.orientation.x, message.pose.pose.orientation.y,
                            message.pose.pose.orientation.z, message.pose.pose.orientation.w],
            "covariance": list(message.pose.covariance),
        },
        "twist": {
            "linear": [message.twist.twist.linear.x, message.twist.twist.linear.y,
                       message.twist.twist.linear.z],
            "angular": [message.twist.twist.angular.x, message.twist.twist.angular.y,
                        message.twist.twist.angular.z],
            "covariance": list(message.twist.covariance),
        },
    }


def main():
    if len(sys.argv) != 5:
        raise SystemExit("usage: collect_ekf_window.py OUTPUT DURATION PHASE_FILE READY_FIFO")
    output_path = Path(sys.argv[1])
    duration = float(sys.argv[2])
    phase_path = Path(sys.argv[3])
    ready_fifo = Path(sys.argv[4])
    rclpy.init()
    node = rclpy.create_node("feedbot_ekf_window_observer")
    counts = {"imu": 0, "diagnostics": 0, "wheel": 0, "filtered": 0, "load": 0}
    previous_cpu = None
    ready = False

    with output_path.open("x") as output:
        def write(kind, value):
            counts[kind] += 1
            output.write(json.dumps({"kind": kind, "monotonic": time.monotonic(),
                                     "phase": phase_value(phase_path), **value}) + "\n")
            output.flush()

        def imu(message):
            write("imu", {
                "stamp_ns": message_stamp(message),
                "frame": message.header.frame_id,
                "orientation": [message.orientation.x, message.orientation.y,
                                message.orientation.z, message.orientation.w],
                "orientation_covariance": list(message.orientation_covariance),
                "angular_velocity": [message.angular_velocity.x, message.angular_velocity.y,
                                     message.angular_velocity.z],
                "angular_velocity_covariance": list(message.angular_velocity_covariance),
                "linear_acceleration": [message.linear_acceleration.x,
                                        message.linear_acceleration.y,
                                        message.linear_acceleration.z],
                "linear_acceleration_covariance": list(message.linear_acceleration_covariance),
            })

        def diagnostics(message):
            write("diagnostics", {"stamp_ns": message_stamp(message), "status": [
                {"name": status.name, "level": int(status.level[0]), "message": status.message,
                 "hardware_id": status.hardware_id,
                 "values": {item.key: item.value for item in status.values}}
                for status in message.status]})

        subscriptions = [
            node.create_subscription(Imu, "/imu", imu, qos_profile_sensor_data),
            node.create_subscription(DiagnosticArray, "/diagnostics", diagnostics, 100),
            node.create_subscription(Odometry, "/wheel/odom",
                                     lambda message: write("wheel", odometry_record(message)), 100),
            node.create_subscription(Odometry, "/odometry/filtered",
                                     lambda message: write("filtered", odometry_record(message)), 100),
        ]
        del subscriptions
        start = time.monotonic()
        deadline = start + duration
        next_load = start
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if now >= next_load:
                current_cpu = cpu_times()
                cpu_percent = None
                if previous_cpu is not None and current_cpu is not None:
                    total_delta = current_cpu["total"] - previous_cpu["total"]
                    idle_delta = current_cpu["idle"] - previous_cpu["idle"]
                    if total_delta > 0:
                        cpu_percent = 100.0 * (total_delta - idle_delta) / total_delta
                load_average = [float(value) for value in read_text("/proc/loadavg").split()[:3]]
                write("load", {"elapsed_s": now - start, "cpu": current_cpu,
                               "cpu_percent_since_previous": cpu_percent,
                               "load_average": load_average, "processes": process_records()})
                previous_cpu = current_cpu
                next_load += 1.0
            if (not ready and counts["imu"] > 0 and counts["diagnostics"] > 0
                    and counts["load"] > 0):
                signal_ready(ready_fifo)
                ready = True

    summary = {"duration_requested_s": duration, "counts": counts,
               "ready_signalled": ready, "finished_monotonic": time.monotonic()}
    output_path.with_suffix(".summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()

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


PROCESS_NAMES = ("imu_node", "ekf_node", "ros2_control_node")


def message_stamp(message):
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


def read_text(path):
    try:
        return Path(path).read_text()
    except (OSError, UnicodeDecodeError):
        return None


def cpu_times():
    fields = read_text("/proc/stat").splitlines()[0].split()[1:]
    values = [int(value) for value in fields]
    return {"total": sum(values[:8]), "idle": values[3] + values[4]}


def thread_record(process_id, thread_id):
    stat = read_text("/proc/{}/task/{}/stat".format(process_id, thread_id))
    if stat is None:
        return None
    fields = stat[stat.rfind(")") + 2:].split()
    try:
        affinity = sorted(os.sched_getaffinity(thread_id))
        scheduler = os.sched_getscheduler(thread_id)
        scheduler_priority = os.sched_getparam(thread_id).sched_priority
    except ProcessLookupError:
        return None
    name = read_text("/proc/{}/task/{}/comm".format(process_id, thread_id))
    if name is None:
        return None
    return {
        "tid": thread_id,
        "name": name.strip(),
        "state": fields[0],
        "priority": int(fields[15]),
        "nice": int(fields[16]),
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
        process_name = Path(arguments[0]).name
        if process_name not in PROCESS_NAMES:
            continue
        command = " ".join(arguments).strip()
        threads = []
        try:
            thread_ids = sorted(int(item.name) for item in (entry / "task").iterdir())
        except OSError:
            continue
        for thread_id in thread_ids:
            record = thread_record(int(entry.name), thread_id)
            if record is not None:
                threads.append(record)
        records.append({"marker": process_name, "pid": int(entry.name), "command": command,
                        "thread_count": len(threads), "threads": threads})
    return records


def main():
    output_path = Path(sys.argv[1])
    duration = float(sys.argv[2])
    rclpy.init()
    node = rclpy.create_node("feedbot_imu_baseline_observer")
    counts = {"imu": 0, "diagnostics": 0, "wheel": 0, "filtered": 0, "load": 0}
    previous_cpu = None

    with output_path.open("x") as output:
        def write(kind, value):
            counts[kind] += 1
            output.write(json.dumps({"kind": kind, "monotonic": time.monotonic(), **value}) + "\n")
            output.flush()

        def imu(message):
            write("imu", {"stamp_ns": message_stamp(message), "frame": message.header.frame_id,
                          "wx": message.angular_velocity.x, "wy": message.angular_velocity.y,
                          "wz": message.angular_velocity.z})

        def diagnostics(message):
            write("diagnostics", {"stamp_ns": message_stamp(message), "status": [
                {"name": status.name, "level": int(status.level[0]), "message": status.message,
                 "values": {item.key: item.value for item in status.values}}
                for status in message.status]})

        def odometry(kind, message):
            write(kind, {"stamp_ns": message_stamp(message),
                         "vx": message.twist.twist.linear.x, "wz": message.twist.twist.angular.z})

        subscriptions = [
            node.create_subscription(Imu, "/imu", imu, qos_profile_sensor_data),
            node.create_subscription(DiagnosticArray, "/diagnostics", diagnostics, 100),
            node.create_subscription(Odometry, "/wheel/odom", lambda value: odometry("wheel", value), 100),
            node.create_subscription(Odometry, "/odometry/filtered",
                                     lambda value: odometry("filtered", value), 100),
        ]
        start = time.monotonic()
        deadline = start + duration
        next_load = start
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if now < next_load:
                continue
            current_cpu = cpu_times()
            cpu_percent = None
            if previous_cpu is not None:
                total_delta = current_cpu["total"] - previous_cpu["total"]
                idle_delta = current_cpu["idle"] - previous_cpu["idle"]
                if total_delta > 0:
                    cpu_percent = 100.0 * (total_delta - idle_delta) / total_delta
            load_average = [float(value) for value in read_text("/proc/loadavg").split()[:3]]
            write("load", {"elapsed_s": now - start, "cpu": current_cpu,
                           "cpu_percent_since_previous": cpu_percent, "load_average": load_average,
                           "processes": process_records()})
            previous_cpu = current_cpu
            next_load += 1.0

    summary = {"duration_requested_s": duration, "counts": counts,
               "finished_monotonic": time.monotonic()}
    output_path.with_suffix(".summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()

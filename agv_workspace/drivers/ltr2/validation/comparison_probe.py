from __future__ import print_function

import ctypes
import json
import math
import os
import re
import subprocess
import sys
import threading
import time

import ros_probe


class Timespec(ctypes.Structure):
    _fields_ = [("seconds", ctypes.c_long), ("nanoseconds", ctypes.c_long)]


LIBC = ctypes.CDLL(None, use_errno=True)


def monotonic():
    value = Timespec()
    if LIBC.clock_gettime(1, ctypes.byref(value)) != 0:
        raise OSError(ctypes.get_errno(), "clock_gettime")
    return value.seconds + value.nanoseconds * 1e-9


def geometry(message, managed):
    count = len(message.ranges)
    error = message.angle_max - (message.angle_min + (count - 1) * message.angle_increment)
    if managed and abs(error) > 1e-6:
        raise ValueError("managed angular endpoint mismatch: %.12g rad" % error)
    increment = 2 * math.pi / 2048
    return {
        "count": count,
        "intensity_count": len(message.intensities),
        "nominal_min": message.angle_min,
        "nominal_max": message.angle_max,
        "increment": message.angle_increment,
        "endpoint_error_rad": error,
        "discrete_first_angle_from_frozen_grid": math.ceil(-1.65 / increment) * increment,
        "discrete_last_angle_from_frozen_grid": math.floor(1.65 / increment) * increment,
        "physical_angle_calibration_performed": False,
    }


class Observer(object):
    def __init__(self, managed, output):
        self.managed = managed
        self.output = output
        self.condition = threading.Condition()
        self.frames = []
        self.error = None

    def receive(self, message):
        with self.condition:
            try:
                received = monotonic()
                metadata = ros_probe.validate_scan(message, time.time(), self.managed)
                if self.frames and metadata["stamp"] <= self.frames[-1]["stamp"]:
                    raise ValueError("scan timestamps did not strictly increase")
                metadata["received_monotonic"] = received
                metadata["geometry"] = geometry(message, self.managed)
                self.output.write(json.dumps(ros_probe.full_scan(message, metadata), allow_nan=False) + "\n")
                self.output.flush()
                self.frames.append(metadata)
            except Exception as error:
                self.error = str(error)
            self.condition.notify_all()

    def collect(self, after, count=30):
        deadline = monotonic() + 12
        with self.condition:
            while True:
                if self.error:
                    raise RuntimeError(self.error)
                frames = [frame for frame in self.frames if frame["received_monotonic"] >= after]
                if len(frames) >= count:
                    return frames[:count]
                remaining = deadline - monotonic()
                if remaining <= 0:
                    raise RuntimeError("only %d scans after transition" % len(frames))
                self.condition.wait(remaining)

    def stopped(self, returned):
        deadline = monotonic() + 5
        with self.condition:
            while True:
                if self.error:
                    raise RuntimeError(self.error)
                now = monotonic()
                last = self.frames[-1]["received_monotonic"] if self.frames else returned
                quiet_since = max(returned, last)
                if now - quiet_since >= 1:
                    return {"last_scan_minus_service_return_s": last - returned,
                            "no_scan_observed_for_s": now - quiet_since,
                            "silence_confirmed_minus_service_return_s": now - returned}
                if now >= deadline:
                    raise RuntimeError("scan stream did not become silent")
                self.condition.wait(min(deadline - now, 1 - (now - quiet_since)))


def call_service(name, managed):
    import rospy
    from std_srvs.srv import Empty, Trigger
    rospy.wait_for_service(name, timeout=10)
    requested = monotonic()
    response = rospy.ServiceProxy(name, Trigger if managed else Empty)()
    returned = monotonic()
    if managed and not response.success:
        raise RuntimeError("%s: %s" % (name, response.message))
    return {"service": name, "request_monotonic": requested, "return_monotonic": returned,
            "service_duration_s": returned - requested, "service_returned_success": True}


def comparison(mode, root):
    import rospy
    from sensor_msgs.msg import LaserScan
    managed = mode == "managed"
    report = {"mode": mode, "passed": False, "absolute_time_alignment_verified": False,
              "stop_measurement": "last subscriber receipt and one-second observed silence; not physical stop instant"}
    capture = None
    capture_log = None
    with open(os.path.join(root, mode + "_scans.jsonl"), "w") as output:
        observer = Observer(managed, output)
        ros_probe.init_node("feedbot_comparison_" + mode)
        subscription = rospy.Subscriber("/scan", LaserScan, observer.receive, queue_size=100)
        try:
            if managed:
                capture_log = open(os.path.join(root, "managed_capture.json"), "w")
                capture = subprocess.Popen(["/usr/bin/python3", os.path.join(os.path.dirname(__file__), "capture_lidar_passive.py"),
                                            "--duration", "20", "--output", os.path.join(root, "managed_packets.bin")],
                                           stdout=capture_log, stderr=capture_log)
                time.sleep(0.3)
                report["start"] = call_service("/ltr2_driver/activate", True)
            else:
                requested = monotonic()
                subprocess.check_call([
                    "/usr/bin/systemd-run", "--unit=feedbot-ltr2-comparison-official.service", "--collect",
                    "--property=Restart=no", "--property=TimeoutStopSec=15",
                    "--property=Environment=ROS_MASTER_URI=http://127.0.0.1:11372",
                    "--property=Environment=ROS_IP=127.0.0.1", "/bin/bash", "-c",
                    "source /opt/ros/melodic/setup.bash; unset ROS_HOSTNAME; exec "
                    "/var/tmp/feedbot-ltme-reference-f6733085/devel/lib/ltme_node/ltme_node "
                    "__name:=official_reference _device_model:=LT-R2 _device_address:=192.168.10.160 "
                    "_angle_min:=-1.65 _angle_max:=1.65 _average_factor:=1"])
                report["start"] = {"request_monotonic": requested, "return_monotonic": monotonic(),
                                   "method": "process launch; official node has no initial activation service"}
            first = observer.collect(report["start"]["request_monotonic"])
            sockets = subprocess.check_output(["/bin/ss", "-H", "-tanp"]).decode() + subprocess.check_output(["/bin/ss", "-H", "-uanp"]).decode()
            owners = [line for line in sockets.splitlines() if "users:((" in line and
                      "2105" in [field.rsplit(":", 1)[-1] for field in line.split()[3:5]]]
            owner_pids = set(int(value) for line in owners for value in re.findall(r"pid=(\d+)", line))
            expected = "/opt/feedbot-ltr2/releases/20261122-managed-v3/build/ltr2_managed_ros1" if managed else "/var/tmp/feedbot-ltme-reference-f6733085/devel/lib/ltme_node/ltme_node"
            report["port_owners"] = owners
            if len(owner_pids) != 1 or len(owners) < 2 or any(os.readlink("/proc/%d/exe" % pid) != expected for pid in owner_pids):
                raise RuntimeError("unexpected exclusive lidar socket ownership")
            report["start"]["first_scan_minus_return_s"] = first[0]["received_monotonic"] - report["start"]["return_monotonic"]
            stop_service = "/ltr2_driver/deactivate" if managed else "/official_reference/request_hibernation"
            wake_service = "/ltr2_driver/activate" if managed else "/official_reference/request_wake_up"
            report["stop"] = call_service(stop_service, managed)
            report["stop"].update(observer.stopped(report["stop"]["return_monotonic"]))
            report["wake"] = call_service(wake_service, managed)
            second = observer.collect(report["wake"]["request_monotonic"])
            report["wake"]["first_scan_minus_return_s"] = second[0]["received_monotonic"] - report["wake"]["return_monotonic"]
            if managed:
                report["final_stop"] = call_service(stop_service, True)
                report["final_stop"].update(observer.stopped(report["final_stop"]["return_monotonic"]))
            else:
                report["quit"] = call_service("/official_reference/quit_driver", False)
                report["quit"].update(observer.stopped(report["quit"]["return_monotonic"]))
            report["first_phase"] = first
            report["second_phase"] = second
            report["passed"] = True
        except Exception as error:
            report["error"] = "%s: %s" % (type(error).__name__, error)
        finally:
            subscription.unregister()
            if capture is not None:
                capture_return = capture.wait()
                capture_log.close()
                report["capture_exit_code"] = capture_return
                if capture_return != 0:
                    report["passed"] = False
            if observer.error:
                report["passed"] = False
                report["error"] = observer.error
            report["scans_received"] = len(observer.frames)
            with open(os.path.join(root, mode + ".json"), "w") as destination:
                json.dump(report, destination, indent=2, allow_nan=False)
    if not report["passed"]:
        raise RuntimeError(json.dumps(report))
    print(json.dumps({"passed": True, "mode": mode, "scans_received": report["scans_received"]}))


def factory_rates(root):
    import rospy
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from map_msgs.msg import DriverEntry
    ros_probe.init_node("feedbot_comparison_factory_rates")
    records = {"scan": [], "wheel_odom": [], "driver_info": []}
    errors = []

    def receive(kind, message):
        received = monotonic()
        if kind != "driver_info":
            age = time.time() - ros_probe.stamp_seconds(message.header.stamp)
            if not -0.5 <= age <= 2:
                errors.append(kind + " timestamp not fresh")
        if kind == "driver_info" and (message.Speedx != 0 or message.Speedth != 0):
            errors.append("nonzero chassis speed")
        records[kind].append(received)

    subscriptions = [rospy.Subscriber(topic, message_type, lambda message, kind=kind: receive(kind, message), queue_size=100)
                     for topic, message_type, kind in (("/scan", LaserScan, "scan"),
                        ("/wheel_odom", Odometry, "wheel_odom"), ("/driver_info", DriverEntry, "driver_info"))]
    time.sleep(6)
    for subscription in subscriptions:
        subscription.unregister()
    report = {"passed": False, "errors": errors, "streams": {}}
    for kind, stamps in records.items():
        if len(stamps) < 30:
            errors.append(kind + " fewer than 30 samples")
            continue
        gaps = [later - earlier for earlier, later in zip(stamps, stamps[1:])]
        report["streams"][kind] = {"samples": len(stamps), "hz": (len(stamps) - 1) / (stamps[-1] - stamps[0]),
                                    "maximum_gap_s": max(gaps)}
        if max(gaps) > 0.5:
            errors.append(kind + " gap exceeds 0.5 seconds")
    report["passed"] = not errors
    with open(os.path.join(root, "factory_rates.json"), "w") as destination:
        json.dump(report, destination, indent=2)
    if errors:
        raise RuntimeError(json.dumps(report))
    print(json.dumps(report))


if __name__ == "__main__":
    if sys.argv[1] == "factory":
        factory_rates(sys.argv[2])
    elif sys.argv[1] in ("official", "managed"):
        comparison(sys.argv[1], sys.argv[2])
    else:
        raise ValueError("unknown comparison mode")

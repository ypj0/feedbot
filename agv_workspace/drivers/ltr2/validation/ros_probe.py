#!/usr/bin/env python2
"""ROS1 evidence and lidar-lifecycle helper for the bounded real LT-R2-B trial."""

from __future__ import print_function

import argparse
import hashlib
import json
import math
import struct
import sys
import time


SCAN_TOPIC = "/candidate/ltr2/managed_scan"
DIAGNOSTIC_TOPIC = "/candidate/ltr2/managed_diagnostics"


def finite(value):
    return not math.isnan(float(value)) and not math.isinf(float(value))


def json_float(value):
    value = float(value)
    if math.isinf(value) and value > 0:
        return "inf"
    return value


def stamp_seconds(stamp):
    return float(stamp.secs) + float(stamp.nsecs) * 1e-9


def scan_digest(message):
    digest = hashlib.sha256()
    digest.update(struct.pack(
        "<7f",
        message.angle_min,
        message.angle_max,
        message.angle_increment,
        message.time_increment,
        message.scan_time,
        message.range_min,
        message.range_max,
    ))
    digest.update(struct.pack("<I", len(message.ranges)))
    for value in message.ranges:
        digest.update(struct.pack("<f", value))
    digest.update(struct.pack("<I", len(message.intensities)))
    for value in message.intensities:
        digest.update(struct.pack("<f", value))
    return digest.hexdigest()


def validate_scan(message, received_wall, strict_geometry=True):
    if len(message.ranges) != 1075 or len(message.intensities) != 1075:
        raise ValueError("scan does not contain 1075 ranges and intensities")
    geometry = [
        message.angle_min,
        message.angle_max,
        message.angle_increment,
        message.time_increment,
        message.scan_time,
        message.range_min,
        message.range_max,
    ]
    if not all(finite(value) for value in geometry):
        raise ValueError("scan geometry contains a non-finite value")
    if message.angle_increment <= 0 or message.range_min <= 0 or message.range_max <= message.range_min:
        raise ValueError("scan geometry bounds are invalid")
    implied_count = (message.angle_max - message.angle_min) / message.angle_increment + 1.0
    geometry_count_matches = abs(implied_count - 1075.0) <= 0.05
    if strict_geometry and not geometry_count_matches:
        raise ValueError("scan angular geometry does not match 1075 samples")
    finite_ranges = []
    finite_in_range = []
    finite_out_of_range = 0
    positive_infinity = 0
    for value in message.ranges:
        if math.isnan(value) or (math.isinf(value) and value < 0):
            raise ValueError("range contains NaN or negative infinity")
        if math.isinf(value):
            positive_infinity += 1
        else:
            finite_ranges.append(float(value))
            if message.range_min <= value <= message.range_max:
                finite_in_range.append(float(value))
            else:
                finite_out_of_range += 1
    if not finite_in_range:
        raise ValueError("scan has no finite indoor return within declared bounds")
    if not all(finite(value) for value in message.intensities):
        raise ValueError("intensity contains a non-finite value")
    stamp = stamp_seconds(message.header.stamp)
    age = received_wall - stamp
    if not finite(stamp) or age < -0.5 or age > 2.0:
        raise ValueError("scan header timestamp is not fresh")
    return {
        "age_s": age,
        "digest_sha256_excluding_header": scan_digest(message),
        "finite_count": len(finite_ranges),
        "finite_in_declared_range_count": len(finite_in_range),
        "finite_max_m": max(finite_ranges),
        "finite_min_m": min(finite_ranges),
        "finite_out_of_declared_range_count": finite_out_of_range,
        "geometry_implied_count": implied_count,
        "geometry_matches_1075": geometry_count_matches,
        "positive_infinity_count": positive_infinity,
        "received_wall": received_wall,
        "stamp": stamp,
    }


def full_scan(message, metadata):
    return {
        "frame_id": message.header.frame_id,
        "header_seq": int(message.header.seq),
        "header_stamp": stamp_seconds(message.header.stamp),
        "geometry": {
            "angle_increment": float(message.angle_increment),
            "angle_max": float(message.angle_max),
            "angle_min": float(message.angle_min),
            "range_max": float(message.range_max),
            "range_min": float(message.range_min),
            "scan_time": float(message.scan_time),
            "time_increment": float(message.time_increment),
        },
        "intensities": [float(value) for value in message.intensities],
        "metadata": metadata,
        "ranges": [json_float(value) for value in message.ranges],
    }


class ScanCollector(object):
    def __init__(self, count, require_diagnostics, strict_geometry):
        self.count = count
        self.require_diagnostics = require_diagnostics
        self.strict_geometry = strict_geometry
        self.frames = []
        self.first = None
        self.last = None
        self.error = None
        self.diagnostic = None

    def scan(self, message):
        if self.error is not None or len(self.frames) >= self.count:
            return
        try:
            received = time.time()
            metadata = validate_scan(message, received, self.strict_geometry)
            if self.frames and metadata["stamp"] <= self.frames[-1]["stamp"]:
                raise ValueError("scan timestamps are not strictly increasing")
            self.frames.append(metadata)
            if self.first is None:
                self.first = full_scan(message, metadata)
            self.last = full_scan(message, metadata)
        except Exception as error:
            self.error = str(error)

    def diagnostics(self, message):
        for entry in message.status:
            if entry.name != "ltr2_managed_data":
                continue
            values = dict((item.key, item.value) for item in entry.values)
            self.diagnostic = {
                "hardware_id": entry.hardware_id,
                "level": int(entry.level),
                "message": entry.message,
                "values": values,
            }

    def complete(self):
        return self.error is not None or (
            len(self.frames) >= self.count and
            (not self.require_diagnostics or self.diagnostic is not None)
        )

    def report(self, mode, topic):
        if self.error is not None:
            raise RuntimeError(self.error)
        if len(self.frames) < self.count:
            raise RuntimeError("received %d of %d required scans" % (len(self.frames), self.count))
        if self.require_diagnostics:
            if self.diagnostic is None:
                raise RuntimeError("managed diagnostic was not received")
            values = self.diagnostic["values"]
            if self.diagnostic["hardware_id"] != "LT-R2-B":
                raise RuntimeError("diagnostic hardware identity mismatch")
            if values.get("ready") != "true" or values.get("healthy") != "true":
                raise RuntimeError("managed diagnostic is not ready and healthy")
            if values.get("navigation_accepted") != "false":
                raise RuntimeError("managed diagnostic claimed navigation acceptance")
        elapsed = self.frames[-1]["received_wall"] - self.frames[0]["received_wall"]
        frequency = float(len(self.frames) - 1) / elapsed if elapsed > 0 and len(self.frames) > 1 else None
        return {
            "data_origin": (
                "real LT-R2-B through fixed managed ROS1 driver; no simulator or replay"
                if self.require_diagnostics else
                "restored factory /ltme_node on production ROS master"
            ),
            "diagnostic": self.diagnostic,
            "first_scan": self.first,
            "frames": self.frames,
            "last_scan": self.last,
            "mode": mode,
            "navigation_accepted": False,
            "passed": True,
            "real_hardware": True,
            "receive_frequency_hz": frequency,
            "scan_count": len(self.frames),
            "strict_corrected_geometry_required": self.strict_geometry,
            "topic": topic,
        }


def init_node(name):
    import rospy
    rospy.init_node(name, anonymous=True, disable_signals=True, log_level=rospy.ERROR)


def collect_scans(count, timeout, managed):
    import rospy
    from diagnostic_msgs.msg import DiagnosticArray
    from sensor_msgs.msg import LaserScan
    init_node("feedbot_ltr2_real_scan_probe")
    collector = ScanCollector(count, managed, managed)
    topic = SCAN_TOPIC if managed else "/scan"
    subscriptions = [rospy.Subscriber(topic, LaserScan, collector.scan, queue_size=20)]
    if managed:
        subscriptions.append(rospy.Subscriber(DIAGNOSTIC_TOPIC, DiagnosticArray, collector.diagnostics, queue_size=10))
    deadline = time.time() + timeout
    while not rospy.is_shutdown() and time.time() < deadline and not collector.complete():
        time.sleep(0.02)
    for subscription in subscriptions:
        subscription.unregister()
    return collector.report("collect" if managed else "factory", topic)


def transition(name):
    import rospy
    from std_srvs.srv import Trigger
    init_node("feedbot_ltr2_real_transition")
    service = "/ltr2_driver/" + name
    rospy.wait_for_service(service, timeout=6.0)
    response = rospy.ServiceProxy(service, Trigger, persistent=False)()
    if not response.success:
        raise RuntimeError("%s failed: %s" % (service, response.message))
    return {
        "message": response.message,
        "mode": "transition",
        "name": name,
        "navigation_accepted": False,
        "passed": True,
        "real_hardware": True,
        "service": service,
    }


def silence(duration):
    import rospy
    from sensor_msgs.msg import LaserScan
    init_node("feedbot_ltr2_real_silence_probe")
    received = []
    subscription = rospy.Subscriber(SCAN_TOPIC, LaserScan, lambda message: received.append(time.time()), queue_size=20)
    connection_deadline = time.time() + 3.0
    while not rospy.is_shutdown() and time.time() < connection_deadline and subscription.get_num_connections() == 0:
        time.sleep(0.02)
    if subscription.get_num_connections() == 0:
        subscription.unregister()
        raise RuntimeError("managed scan publisher did not connect for silence observation")
    deadline = time.time() + duration
    while not rospy.is_shutdown() and time.time() < deadline:
        time.sleep(0.02)
    subscription.unregister()
    if received:
        raise RuntimeError("managed scan published %d messages while stopped" % len(received))
    return {
        "duration_s": duration,
        "mode": "silence",
        "navigation_accepted": False,
        "passed": True,
        "real_hardware": True,
        "received": 0,
        "topic": SCAN_TOPIC,
    }


def preflight(timeout):
    import rospy
    from map_msgs.msg import DriverEntry
    from nav_msgs.msg import Odometry
    init_node("feedbot_ltr2_real_stationary_preflight")
    state = {
        "driver": [],
        "odom": [],
    }

    def driver(message):
        state["driver"].append({
            "received_wall": time.time(),
            "Speedth": int(message.Speedth),
            "Speedx": int(message.Speedx),
        })

    def odom(message):
        received = time.time()
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        speeds = [linear.x, linear.y, linear.z, angular.x, angular.y, angular.z]
        if not all(finite(value) for value in speeds):
            return
        state["odom"].append({
            "age_s": received - stamp_seconds(message.header.stamp),
            "angular_rad_s": [float(angular.x), float(angular.y), float(angular.z)],
            "linear_m_s": [float(linear.x), float(linear.y), float(linear.z)],
            "received_wall": received,
            "stamp": stamp_seconds(message.header.stamp),
        })

    subscriptions = [
        rospy.Subscriber("/driver_info", DriverEntry, driver, queue_size=10),
        rospy.Subscriber("/wheel_odom", Odometry, odom, queue_size=20),
    ]
    deadline = time.time() + timeout
    while not rospy.is_shutdown() and time.time() < deadline:
        fresh = [sample for sample in state["odom"] if -0.5 <= sample["age_s"] <= 2.0]
        if len(state["driver"]) >= 3 and len(fresh) >= 3:
            break
        time.sleep(0.02)
    for subscription in subscriptions:
        subscription.unregister()
    fresh = [sample for sample in state["odom"] if -0.5 <= sample["age_s"] <= 2.0]
    if len(state["driver"]) < 3:
        raise RuntimeError("fewer than three fresh /driver_info messages received")
    if len(fresh) < 3:
        raise RuntimeError("fewer than three fresh /wheel_odom messages received")
    for earlier, later in zip(fresh, fresh[1:]):
        if later["stamp"] <= earlier["stamp"]:
            raise RuntimeError("wheel odometry timestamps are not increasing")
    maxima = [max(abs(sample[axis][index]) for sample in fresh) for axis, index in (
        ("linear_m_s", 0), ("linear_m_s", 1), ("linear_m_s", 2),
        ("angular_rad_s", 0), ("angular_rad_s", 1), ("angular_rad_s", 2),
    )]
    if any(value > 0.01 for value in maxima):
        raise RuntimeError("wheel odometry is not stationary")
    if any(sample["Speedx"] != 0 or sample["Speedth"] != 0 for sample in state["driver"]):
        raise RuntimeError("driver feedback speed is not zero")
    return {
        "driver_info_samples": state["driver"],
        "driver_info_type": "map_msgs/DriverEntry",
        "max_abs_angular_rad_s": maxima[3:],
        "max_abs_linear_m_s": maxima[:3],
        "mode": "preflight",
        "motion_guard": "fresh wheel odometry velocity only; no motor disable, E-stop, or speed publication",
        "navigation_accepted": False,
        "passed": True,
        "real_hardware": True,
        "wheel_odom_samples": fresh,
    }


def factory(timeout, count):
    import rosgraph
    import rosnode
    deadline = time.time() + timeout
    publishers = []
    while time.time() < deadline:
        try:
            nodes = rosnode.get_node_names()
            published, unused_subscribed, unused_services = rosgraph.Master("/feedbot_ltr2_factory_probe").getSystemState()
            publishers = dict(published).get("/scan", [])
            if "/ltme_node" in nodes and "/ltme_node" in publishers:
                break
        except Exception:
            pass
        time.sleep(0.1)
    if "/ltme_node" not in publishers:
        raise RuntimeError("restored /ltme_node is absent")
    remaining = deadline - time.time()
    if remaining < 1.0:
        raise RuntimeError("restored /ltme_node did not leave time for a fresh scan")
    report = collect_scans(count, remaining, False)
    report["ltme_node_present"] = True
    report["scan_publishers"] = publishers
    return report


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="mode")
    preflight_parser = subparsers.add_parser("preflight")
    preflight_parser.add_argument("--timeout", type=float, default=12.0)
    transition_parser = subparsers.add_parser("transition")
    transition_parser.add_argument("name", choices=("configure", "activate", "deactivate", "cleanup"))
    collect_parser = subparsers.add_parser("collect")
    collect_parser.add_argument("--count", type=int, default=10)
    collect_parser.add_argument("--timeout", type=float, default=15.0)
    silence_parser = subparsers.add_parser("silence")
    silence_parser.add_argument("--duration", type=float, default=1.0)
    factory_parser = subparsers.add_parser("factory")
    factory_parser.add_argument("--count", type=int, default=3)
    factory_parser.add_argument("--timeout", type=float, default=40.0)
    arguments = parser.parse_args(argv)
    if arguments.mode is None:
        parser.error("a mode is required")
    if hasattr(arguments, "timeout") and not 1.0 <= arguments.timeout <= 60.0:
        parser.error("timeout must be between 1 and 60 seconds")
    if hasattr(arguments, "count") and not 1 <= arguments.count <= 30:
        parser.error("count must be between 1 and 30")
    if hasattr(arguments, "duration") and not 1.0 <= arguments.duration <= 3.0:
        parser.error("duration must be between 1 and 3 seconds")
    return arguments


def main(argv=None):
    arguments = parse_arguments(argv)
    try:
        if arguments.mode == "preflight":
            report = preflight(arguments.timeout)
        elif arguments.mode == "transition":
            report = transition(arguments.name)
        elif arguments.mode == "collect":
            report = collect_scans(arguments.count, arguments.timeout, True)
        elif arguments.mode == "silence":
            report = silence(arguments.duration)
        else:
            report = factory(arguments.timeout, arguments.count)
    except BaseException as error:
        report = {
            "error": "%s: %s" % (error.__class__.__name__, error),
            "mode": arguments.mode,
            "navigation_accepted": False,
            "passed": False,
            "real_hardware": True,
        }
    print(json.dumps(report, sort_keys=True, allow_nan=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())

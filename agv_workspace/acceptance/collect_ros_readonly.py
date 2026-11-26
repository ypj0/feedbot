#!/usr/bin/env python
from __future__ import print_function

import argparse
import ctypes
import json
import math
import os
import signal
import sys
import threading
import tempfile
import time


DRIVER_FIELDS = (
    "MotorInitStatus", "EmergencyStatus", "Power", "ChargeStatus", "Mode",
    "LeftAlarm", "RightAlarm", "MoveAlarm", "StopAlarm",
    "LeftEncoder", "RightEncoder", "Speedx", "Speedth", "Version",
    "Shutdown", "MotorEnabled",
)

INTEGER_TYPES = (int, long) if sys.version_info[0] == 2 else (int,)


class Timespec(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_nsec", ctypes.c_long)]


LIBRT = None if hasattr(time, "monotonic") else ctypes.CDLL("librt.so.1", use_errno=True)


def write_json_exclusive(path, report):
    payload = json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
    directory = os.path.dirname(os.path.abspath(path))
    descriptor, temporary = tempfile.mkstemp(prefix=".readonly-report-", dir=directory)
    try:
        with os.fdopen(descriptor, "w") as output:
            output.write(payload)
        os.link(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def monotonic_time():
    if hasattr(time, "monotonic"):
        return time.monotonic()
    value = Timespec()
    if LIBRT.clock_gettime(1, ctypes.byref(value)) != 0:
        error_number = ctypes.get_errno()
        raise OSError(error_number, os.strerror(error_number))
    return value.tv_sec + value.tv_nsec / 1000000000.0


def finite_number(value):
    try:
        return math.isfinite(float(value)) if hasattr(math, "isfinite") else not (math.isnan(float(value)) or math.isinf(float(value)))
    except (TypeError, ValueError, OverflowError):
        return False


def nested_value(value, path):
    for name in path.split("."):
        if not hasattr(value, name):
            return None
        value = getattr(value, name)
    return value


def stamp_seconds(message):
    stamp = nested_value(message, "header.stamp")
    if stamp is None:
        return None
    if hasattr(stamp, "to_sec"):
        return float(stamp.to_sec())
    seconds = getattr(stamp, "secs", None)
    nanoseconds = getattr(stamp, "nsecs", None)
    if seconds is None or nanoseconds is None:
        return None
    return float(seconds) + float(nanoseconds) / 1000000000.0


def add_issue(issues, name, count=1):
    issues[name] = issues.get(name, 0) + count


def bounded_count(counts, name):
    if name not in counts and len(counts) >= 16:
        name = "<additional_values>"
    counts[name] = counts.get(name, 0) + 1


def update_range(summary, name, value):
    if not finite_number(value):
        return
    number = float(value)
    entry = summary.setdefault(name, {"min": number, "max": number})
    entry["min"] = min(entry["min"], number)
    entry["max"] = max(entry["max"], number)


def vector_values(value, prefix):
    return [(prefix + axis, getattr(value, axis, None)) for axis in ("x", "y", "z")]


def validate_covariance(values, expected_length, prefix, issues, summary):
    try:
        length = len(values)
    except TypeError:
        add_issue(issues, prefix + "_covariance_missing")
        return
    if length != expected_length:
        add_issue(issues, prefix + "_covariance_length")
        return
    if expected_length == 9 and values[0] == -1:
        add_issue(summary.setdefault("observations", {}), prefix + "_covariance_unavailable")
        return
    if any(not finite_number(value) for value in values):
        add_issue(issues, prefix + "_covariance_nonfinite")


def validate_quaternion(quaternion, prefix, issues, summary):
    values = [getattr(quaternion, axis, None) for axis in ("x", "y", "z", "w")]
    if any(not finite_number(value) for value in values):
        add_issue(issues, prefix + "_quaternion_nonfinite")
        return
    norm = math.sqrt(sum(float(value) * float(value) for value in values))
    update_range(summary, prefix + "_quaternion_norm", norm)
    if abs(norm - 1.0) > 0.001:
        add_issue(issues, prefix + "_quaternion_not_unit")


def validate_scan(message, issues, summary):
    ranges = getattr(message, "ranges", ())
    try:
        range_count = len(ranges)
    except TypeError:
        add_issue(issues, "ranges_missing")
        return
    for name in ("angle_min", "angle_max", "angle_increment", "range_min", "range_max"):
        value = getattr(message, name, None)
        update_range(summary, name, value)
        if not finite_number(value):
            add_issue(issues, name + "_nonfinite")
    angle_min = getattr(message, "angle_min", None)
    angle_max = getattr(message, "angle_max", None)
    increment = getattr(message, "angle_increment", None)
    range_min = getattr(message, "range_min", None)
    range_max = getattr(message, "range_max", None)
    if finite_number(increment) and float(increment) <= 0:
        add_issue(issues, "angle_increment_not_positive")
    if finite_number(angle_min) and finite_number(angle_max) and float(angle_max) < float(angle_min):
        add_issue(issues, "angle_bounds_reversed")
    if all(finite_number(value) for value in (angle_min, angle_max, increment)) and float(increment) > 0:
        expected = int(round((float(angle_max) - float(angle_min)) / float(increment))) + 1
        if range_count != expected:
            add_issue(issues, "ranges_geometry_length")
    finite_count = 0
    in_range_count = 0
    nan_count = 0
    for value in ranges:
        if finite_number(value):
            finite_count += 1
            if finite_number(range_min) and finite_number(range_max) and float(range_min) <= float(value) <= float(range_max):
                in_range_count += 1
        elif isinstance(value, (int, float)) and math.isnan(float(value)):
            nan_count += 1
    update_range(summary, "range_count", range_count)
    update_range(summary, "finite_ratio", float(finite_count) / range_count if range_count else 0.0)
    update_range(summary, "finite_in_declared_range_ratio", float(in_range_count) / finite_count if finite_count else 0.0)
    if nan_count:
        add_issue(issues, "range_nan", nan_count)
    if finite_count - in_range_count:
        add_issue(issues, "finite_range_outside_declared_bounds", finite_count - in_range_count)
    intensities = getattr(message, "intensities", ())
    if intensities is not None and len(intensities) not in (0, range_count):
        add_issue(issues, "intensities_length")


def validate_imu(message, issues, summary):
    orientation_covariance = getattr(message, "orientation_covariance", ())
    if not orientation_covariance or orientation_covariance[0] != -1:
        validate_quaternion(getattr(message, "orientation", object()), "orientation", issues, summary)
    for name in ("angular_velocity", "linear_acceleration"):
        value = getattr(message, name, object())
        for field, number in vector_values(value, name + "."):
            update_range(summary, field, number)
            if not finite_number(number):
                add_issue(issues, field + "_nonfinite")
    validate_covariance(getattr(message, "orientation_covariance", None), 9, "orientation", issues, summary)
    validate_covariance(getattr(message, "angular_velocity_covariance", None), 9, "angular_velocity", issues, summary)
    validate_covariance(getattr(message, "linear_acceleration_covariance", None), 9, "linear_acceleration", issues, summary)


def validate_odom(message, issues, summary):
    position = nested_value(message, "pose.pose.position") or object()
    orientation = nested_value(message, "pose.pose.orientation") or object()
    linear = nested_value(message, "twist.twist.linear") or object()
    angular = nested_value(message, "twist.twist.angular") or object()
    for value, prefix in ((position, "position."), (linear, "linear."), (angular, "angular.")):
        for field, number in vector_values(value, prefix):
            update_range(summary, field, number)
            if not finite_number(number):
                add_issue(issues, field + "_nonfinite")
    validate_quaternion(orientation, "pose", issues, summary)
    validate_covariance(nested_value(message, "pose.covariance"), 36, "pose", issues, summary)
    validate_covariance(nested_value(message, "twist.covariance"), 36, "twist", issues, summary)


ENCODING_BYTES = {
    "mono8": 1, "8UC1": 1, "mono16": 2, "16UC1": 2, "16SC1": 2,
    "rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "32FC1": 4,
}


def validate_image(message, issues, summary):
    width = getattr(message, "width", None)
    height = getattr(message, "height", None)
    step = getattr(message, "step", None)
    encoding = getattr(message, "encoding", "")
    data = getattr(message, "data", None)
    for name, value in (("width", width), ("height", height), ("step", step)):
        update_range(summary, name, value)
        if not isinstance(value, INTEGER_TYPES) or value <= 0:
            add_issue(issues, name + "_invalid")
    bounded_count(summary.setdefault("encodings", {}), encoding or "<empty>")
    if not encoding:
        add_issue(issues, "encoding_empty")
    elif encoding not in ENCODING_BYTES:
        add_issue(issues, "encoding_not_validated")
    try:
        data_length = len(data)
    except TypeError:
        data_length = -1
        add_issue(issues, "data_missing")
    update_range(summary, "data_length", data_length)
    if isinstance(width, INTEGER_TYPES) and isinstance(step, INTEGER_TYPES) and encoding in ENCODING_BYTES:
        if step < width * ENCODING_BYTES[encoding]:
            add_issue(issues, "step_too_small_for_encoding")
    if isinstance(height, INTEGER_TYPES) and isinstance(step, INTEGER_TYPES) and data_length >= 0 and data_length < height * step:
        add_issue(issues, "data_shorter_than_step_times_height")
    if isinstance(height, INTEGER_TYPES) and isinstance(step, INTEGER_TYPES) and data_length > height * step:
        add_issue(issues, "data_longer_than_step_times_height")


def validate_camera_info(message, issues, summary):
    width = getattr(message, "width", None)
    height = getattr(message, "height", None)
    update_range(summary, "width", width)
    update_range(summary, "height", height)
    if not isinstance(width, INTEGER_TYPES) or width <= 0 or not isinstance(height, INTEGER_TYPES) or height <= 0:
        add_issue(issues, "dimensions_invalid")
    matrix_k = getattr(message, "K", ())
    matrix_p = getattr(message, "P", ())
    if len(matrix_k) != 9 or any(not finite_number(value) for value in matrix_k):
        add_issue(issues, "K_invalid")
    if len(matrix_p) != 12 or any(not finite_number(value) for value in matrix_p):
        add_issue(issues, "P_invalid")
    calibrated = len(matrix_k) == 9 and finite_number(matrix_k[0]) and float(matrix_k[0]) != 0.0
    summary.setdefault("observations", {})["calibrated_samples"] = summary.setdefault("observations", {}).get("calibrated_samples", 0) + int(calibrated)
    if calibrated:
        update_range(summary, "K_fx", matrix_k[0])
        if not finite_number(matrix_k[4]) or matrix_k[0] <= 0 or matrix_k[4] <= 0:
            add_issue(issues, "K_focal_length_not_positive")
        if len(matrix_p) == 12:
            update_range(summary, "P_fx", matrix_p[0])
    else:
        add_issue(issues, "uncalibrated")


def validate_driver(message, issues, summary):
    present = 0
    for field in DRIVER_FIELDS:
        if hasattr(message, field):
            value = getattr(message, field)
            if finite_number(value):
                update_range(summary, "field." + field, value)
                present += 1
            else:
                add_issue(issues, "field_non_numeric")
        else:
            add_issue(issues, "missing_field." + field)
    if not present:
        add_issue(issues, "no_whitelisted_numeric_fields")


VALIDATORS = {
    "scan": validate_scan,
    "imu": validate_imu,
    "odom": validate_odom,
    "image": validate_image,
    "camera_info": validate_camera_info,
    "driver": validate_driver,
}


class TopicObservation(object):
    def __init__(self, topic, validator_name):
        self.topic = topic
        self.validator_name = validator_name
        self.message_type = None
        self.samples = 0
        self.first_monotonic = None
        self.last_monotonic = None
        self.first_wall = None
        self.last_wall = None
        self.max_gap = None
        self.max_gap_ending_offset = None
        self.post_warmup_max_gap = None
        self.post_warmup_gap_count = 0
        self.header_first = None
        self.header_last = None
        self.previous_header = None
        self.header_age_min = None
        self.header_age_max = None
        self.header_fallbacks = 0
        self.header_duplicates = 0
        self.header_reversals = 0
        self.header_future = 0
        self.receive_reversals = 0
        self.frame_ids = {}
        self.issues = {}
        self.summary = {}
        self.errors = []

    def observe(self, message, received_monotonic, received_wall):
        self.samples += 1
        if self.first_monotonic is None:
            self.first_monotonic = received_monotonic
            self.first_wall = received_wall
        if self.last_monotonic is not None:
            gap = received_monotonic - self.last_monotonic
            if gap < 0:
                self.receive_reversals += 1
            else:
                if self.max_gap is None or gap > self.max_gap:
                    self.max_gap = gap
                    self.max_gap_ending_offset = received_monotonic - self.first_monotonic
                if self.last_monotonic >= self.first_monotonic + 2.0:
                    self.post_warmup_gap_count += 1
                    self.post_warmup_max_gap = gap if self.post_warmup_max_gap is None else max(self.post_warmup_max_gap, gap)
        self.last_monotonic = received_monotonic
        self.last_wall = received_wall
        frame_id = nested_value(message, "header.frame_id")
        if frame_id:
            bounded_count(self.frame_ids, str(frame_id))
        stamp = stamp_seconds(message)
        if stamp is None or stamp <= 0 or not finite_number(stamp):
            self.header_fallbacks += 1
        else:
            if self.header_first is None:
                self.header_first = stamp
            self.header_last = stamp
            if self.previous_header is not None:
                if stamp == self.previous_header:
                    self.header_duplicates += 1
                elif stamp < self.previous_header:
                    self.header_reversals += 1
            self.previous_header = stamp
            age = received_wall - stamp
            self.header_age_min = age if self.header_age_min is None else min(self.header_age_min, age)
            self.header_age_max = age if self.header_age_max is None else max(self.header_age_max, age)
            if age < 0:
                self.header_future += 1
        VALIDATORS[self.validator_name](message, self.issues, self.summary)

    def report(self):
        status = "error" if self.errors else ("no_data" if not self.samples else ("invalid" if self.issues else "observation_ok"))
        span = None if self.samples < 2 else self.last_monotonic - self.first_monotonic
        frequency = None if not span or span <= 0 else (self.samples - 1) / span
        return {
            "topic": self.topic,
            "validator": self.validator_name,
            "message_type": self.message_type,
            "status": status,
            "sample_count": self.samples,
            "frequency_hz": frequency,
            "max_receive_gap_s": self.max_gap,
            "max_gap_ending_offset_from_first_sample_s": self.max_gap_ending_offset,
            "post_warmup_receive_gaps": {
                "warmup_after_first_sample_s": 2.0,
                "definition": "Both bounding samples must arrive at least two seconds after the first; boundary-crossing gaps excluded only here, not from whole-run statistics",
                "gap_count": self.post_warmup_gap_count,
                "max_gap_s": self.post_warmup_max_gap,
                "acceptance_threshold": None,
            },
            "receive_wall_epoch_range": [self.first_wall, self.last_wall],
            "header_stamp_range": [self.header_first, self.header_last],
            "header_age_s_range": [self.header_age_min, self.header_age_max],
            "header_clock_observations": {
                "fallback_count": self.header_fallbacks,
                "duplicate_count": self.header_duplicates,
                "reversal_count": self.header_reversals,
                "future_count": self.header_future,
                "receive_monotonic_reversal_count": self.receive_reversals,
            },
            "frame_ids": self.frame_ids,
            "issues": self.issues,
            "summary": self.summary,
            "errors": self.errors,
        }


class CollectorCore(object):
    def __init__(self, topics):
        self.lock = threading.RLock()
        self.observations = dict((name, TopicObservation(topic, validator)) for name, topic, validator in topics)

    def observe(self, name, message, received_monotonic=None, received_wall=None):
        received_monotonic = monotonic_time() if received_monotonic is None else received_monotonic
        received_wall = time.time() if received_wall is None else received_wall
        with self.lock:
            try:
                self.observations[name].observe(message, received_monotonic, received_wall)
            except Exception as error:
                self.record_error(name, "validation failed: %s" % error)

    def record_error(self, name, error):
        with self.lock:
            if len(self.observations[name].errors) < 10:
                self.observations[name].errors.append(error)

    def report(self, duration_requested=None, elapsed_monotonic=None):
        with self.lock:
            topics = json.loads(json.dumps(dict((name, observation.report()) for name, observation in sorted(self.observations.items())), allow_nan=False))
        return {
            "schema_version": 1,
            "collection": {
                "duration_requested_s": duration_requested,
                "elapsed_monotonic_s": elapsed_monotonic,
                "receive_clock": "local CLOCK_MONOTONIC",
                "header_age_assumption": "ROS header stamps use Unix epoch and the collector host clock is synchronized; ages are observations, not acceptance limits",
                "bounded_payload": "image bytes and unrecognized message payloads are not retained",
            },
            "topics": topics,
            "all_hardware_accepted": False,
            "acceptance_statement": "Read-only observations cannot establish complete hardware acceptance",
        }


def dynamic_driver_callback(core, name, any_message):
    try:
        import roslib.message
        type_name = getattr(any_message, "_connection_header", {}).get("type")
        message_class = roslib.message.get_message_class(type_name) if type_name else None
        if message_class is None:
            raise ValueError("message class unavailable for %r" % (type_name,))
        decoded = message_class()
        decoded.deserialize(any_message._buff)
        core.observations[name].message_type = type_name
        core.observe(name, decoded)
    except Exception as error:
        core.record_error(name, "dynamic decode failed: %s" % error)


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description="Bounded, read-only ROS1 hardware observation collector")
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--odom-topic", default="/wheel_odom")
    parser.add_argument("--depth-topic", default="/ascamera_hp60c/depth0/image_raw")
    parser.add_argument("--rgb-topic", default="/ascamera_hp60c/rgb0/image")
    parser.add_argument("--depth-info-topic", default="/ascamera_hp60c/depth0/camera_info")
    parser.add_argument("--rgb-info-topic", default="/ascamera_hp60c/rgb0/camera_info")
    parser.add_argument("--driver-topic", default="/driver_info")
    arguments = parser.parse_args(argv)
    if os.path.lexists(arguments.output):
        parser.error("--output already exists; choose a new run path")
    if not 1.0 <= arguments.duration <= 120.0:
        parser.error("--duration must be between 1 and 120 seconds")
    return arguments


def run_ros(arguments):
    import rospy
    import rostopic
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan

    topics = [
        ("scan", arguments.scan_topic, "scan"),
        ("imu", arguments.imu_topic, "imu"),
        ("odom", arguments.odom_topic, "odom"),
        ("depth_image", arguments.depth_topic, "image"),
        ("rgb_image", arguments.rgb_topic, "image"),
        ("depth_camera_info", arguments.depth_info_topic, "camera_info"),
        ("rgb_camera_info", arguments.rgb_info_topic, "camera_info"),
        ("driver", arguments.driver_topic, "driver"),
    ]
    core = CollectorCore(topics)
    rospy.init_node("feedbot_readonly_observer", anonymous=True, disable_signals=True)
    subscribers = []
    message_classes = {
        "scan": LaserScan, "imu": Imu, "odom": Odometry,
        "depth_image": Image, "rgb_image": Image,
        "depth_camera_info": CameraInfo, "rgb_camera_info": CameraInfo,
    }
    for name, topic, unused_validator in topics:
        if name == "driver":
            try:
                message_class, real_topic, unused_eval = rostopic.get_topic_class(topic, blocking=False)
            except Exception as error:
                message_class, real_topic = None, None
                core.observations[name].errors.append("topic type lookup failed; using AnyMsg: %s" % error)
            if message_class is None:
                core.observations[name].message_type = "rospy.AnyMsg"
                subscribers.append(rospy.Subscriber(topic, rospy.AnyMsg, lambda message, n=name: dynamic_driver_callback(core, n, message), queue_size=10))
            else:
                core.observations[name].message_type = getattr(message_class, "_type", str(message_class))
                subscribers.append(rospy.Subscriber(real_topic or topic, message_class, lambda message, n=name: core.observe(n, message), queue_size=10))
        else:
            message_class = message_classes[name]
            core.observations[name].message_type = message_class._type
            subscribers.append(rospy.Subscriber(topic, message_class, lambda message, n=name: core.observe(n, message), queue_size=10))
    start = monotonic_time()
    deadline = start + arguments.duration
    while not rospy.is_shutdown():
        remaining = deadline - monotonic_time()
        if remaining <= 0:
            break
        time.sleep(min(0.05, remaining))
    for subscriber in subscribers:
        subscriber.unregister()
    return core.report(arguments.duration, monotonic_time() - start)


def main(argv=None):
    arguments = parse_arguments(argv)
    if hasattr(signal, "alarm"):
        signal.alarm(int(math.ceil(arguments.duration)) + 15)
    try:
        report = run_ros(arguments)
    except Exception as error:
        report = {
            "schema_version": 1,
            "collector_error": "%s: %s" % (error.__class__.__name__, error),
            "all_hardware_accepted": False,
            "acceptance_statement": "Collection failure is not hardware acceptance",
        }
    output_directory = os.path.dirname(os.path.abspath(arguments.output))
    if not os.path.isdir(output_directory):
        raise IOError("output directory does not exist: %s" % output_directory)
    write_json_exclusive(arguments.output, report)
    return 0 if "collector_error" not in report else 2


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python
from __future__ import print_function

import argparse
import hashlib
import json
import math
import os
import signal
import struct
import sys
import threading
import time

from collect_ros_readonly import monotonic_time, write_json_exclusive


INTEGER_TYPES = (int, long) if sys.version_info[0] == 2 else (int,)
MAX_IMAGE_BYTES = 32 * 1024 * 1024
MAX_SCAN_VALUES = 100000
DEPTH_BANDS = (
    (1, 499),
    (500, 999),
    (1000, 1999),
    (2000, 3999),
    (4000, 7999),
    (8000, 65535),
)


def stamp_seconds(message):
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    try:
        if hasattr(stamp, "to_sec"):
            value = float(stamp.to_sec())
        else:
            seconds = getattr(stamp, "secs", None)
            nanoseconds = getattr(stamp, "nsecs", None)
            if seconds is None or nanoseconds is None:
                return None
            value = float(seconds) + float(nanoseconds) / 1000000000.0
        return value if not (math.isnan(value) or math.isinf(value)) else None
    except (TypeError, ValueError, OverflowError):
        return None


def frame_id(message):
    value = getattr(getattr(message, "header", None), "frame_id", "")
    return str(value).strip() if value else None


def byte_view(data):
    if data is None:
        raise ValueError("payload is missing")
    if isinstance(data, bytearray):
        return data
    if isinstance(data, bytes):
        return bytearray(data)
    if sys.version_info[0] == 2 and isinstance(data, str):
        return bytearray(data)
    try:
        return bytearray(data)
    except (TypeError, ValueError):
        raise ValueError("payload is not byte-compatible")


def image_layout(message, bytes_per_pixel, accepted_encodings):
    encoding = str(getattr(message, "encoding", ""))
    width = getattr(message, "width", None)
    height = getattr(message, "height", None)
    step = getattr(message, "step", None)
    if encoding not in accepted_encodings:
        raise ValueError("unsupported encoding: %s" % (encoding or "<empty>"))
    if not isinstance(width, INTEGER_TYPES) or width <= 0:
        raise ValueError("width must be a positive integer")
    if not isinstance(height, INTEGER_TYPES) or height <= 0:
        raise ValueError("height must be a positive integer")
    if not isinstance(step, INTEGER_TYPES) or step < width * bytes_per_pixel:
        raise ValueError("step is smaller than the encoded row")
    expected_length = height * step
    if expected_length > MAX_IMAGE_BYTES:
        raise ValueError("image exceeds the bounded analysis limit")
    raw_payload = getattr(message, "data", None)
    if raw_payload is not None and len(raw_payload) > MAX_IMAGE_BYTES:
        raise ValueError("payload exceeds the bounded analysis limit")
    payload = byte_view(raw_payload)
    if len(payload) < expected_length:
        raise ValueError("payload is truncated: %d < %d" % (len(payload), expected_length))
    if len(payload) > expected_length:
        raise ValueError("payload has trailing bytes: %d > %d" % (len(payload), expected_length))
    return encoding, width, height, step, payload


def analyze_depth_image(message):
    encoding, width, height, step, payload = image_layout(message, 2, ("16UC1",))
    byte_order = ">" if bool(getattr(message, "is_bigendian", False)) else "<"
    zero_count = 0
    nonzero_count = 0
    nonzero_min = None
    nonzero_max = None
    nonzero_sum = 0
    band_counts = [0] * len(DEPTH_BANDS)
    for row in range(height):
        row_offset = row * step
        for column in range(width):
            value = struct.unpack_from(byte_order + "H", payload, row_offset + column * 2)[0]
            if value == 0:
                zero_count += 1
                continue
            nonzero_count += 1
            nonzero_sum += value
            nonzero_min = value if nonzero_min is None else min(nonzero_min, value)
            nonzero_max = value if nonzero_max is None else max(nonzero_max, value)
            for index, bounds in enumerate(DEPTH_BANDS):
                if bounds[0] <= value <= bounds[1]:
                    band_counts[index] += 1
                    break
    pixel_count = width * height
    distribution = []
    for bounds, count in zip(DEPTH_BANDS, band_counts):
        distribution.append({
            "raw_min_inclusive": bounds[0],
            "raw_max_inclusive": bounds[1],
            "count": count,
            "nonzero_ratio": float(count) / nonzero_count if nonzero_count else 0.0,
        })
    return {
        "status": "observation_ok",
        "encoding": encoding,
        "width": width,
        "height": height,
        "step": step,
        "row_padding_bytes": step - width * 2,
        "is_bigendian": bool(getattr(message, "is_bigendian", False)),
        "pixel_count": pixel_count,
        "zero_count": zero_count,
        "nonzero_count": nonzero_count,
        "zero_ratio": float(zero_count) / pixel_count,
        "nonzero_ratio": float(nonzero_count) / pixel_count,
        "nonzero_raw_min": nonzero_min,
        "nonzero_raw_max": nonzero_max,
        "nonzero_raw_mean": float(nonzero_sum) / nonzero_count if nonzero_count else None,
        "nonzero_raw_distribution": distribution,
        "interpretation": "Raw 16UC1 content distribution only; units, accuracy, calibration and freshness are not established",
    }


def analyze_rgb_image(message):
    channels_by_encoding = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}
    encoding = str(getattr(message, "encoding", ""))
    channels = channels_by_encoding.get(encoding)
    if channels is None:
        raise ValueError("unsupported encoding: %s" % (encoding or "<empty>"))
    encoding, width, height, step, payload = image_layout(message, channels, tuple(channels_by_encoding))
    byte_counts = [0] * 256
    channel_values = [set() for unused_index in range(channels)]
    for row in range(height):
        row_offset = row * step
        for byte_index in range(width * channels):
            value = payload[row_offset + byte_index]
            byte_counts[value] += 1
            channel_values[byte_index % channels].add(value)
    active_bytes = width * height * channels
    distinct_values = sum(1 for count in byte_counts if count)
    entropy = 0.0
    for count in byte_counts:
        if count:
            probability = float(count) / active_bytes
            entropy -= probability * (math.log(probability) / math.log(2.0))
    return {
        "status": "observation_ok",
        "encoding": encoding,
        "width": width,
        "height": height,
        "step": step,
        "row_padding_bytes": step - width * channels,
        "pixel_count": width * height,
        "analyzed_byte_count": active_bytes,
        "distinct_byte_values": distinct_values,
        "byte_min": next((index for index, count in enumerate(byte_counts) if count), None),
        "byte_max": next((index for index in range(255, -1, -1) if byte_counts[index]), None),
        "byte_entropy_bits": entropy,
        "distinct_values_by_channel": [len(values) for values in channel_values],
        "interpretation": "Encoded byte diversity only; scene quality, color accuracy, calibration and freshness are not established",
    }


def scan_payload_fingerprint(message):
    ranges = getattr(message, "ranges", None)
    intensities = getattr(message, "intensities", None)
    if ranges is None or intensities is None:
        raise ValueError("scan ranges or intensities are missing")
    if len(ranges) + len(intensities) > MAX_SCAN_VALUES:
        raise ValueError("scan exceeds the bounded analysis limit")
    digest = hashlib.sha256()
    for name in ("angle_min", "angle_max", "angle_increment", "time_increment", "scan_time", "range_min", "range_max"):
        value = getattr(message, name, None)
        if value is None:
            raise ValueError("scan field is missing: %s" % name)
        digest.update(struct.pack("!d", float(value)))
    for values in (ranges, intensities):
        digest.update(struct.pack("!I", len(values)))
        for value in values:
            digest.update(struct.pack("!d", float(value)))
    return digest.hexdigest()


class ScanContentObservation(object):
    def __init__(self):
        self.sample_count = 0
        self.error_count = 0
        self.errors = []
        self.first_fingerprint = None
        self.latest_fingerprint = None
        self.unique_fingerprints = set()
        self.unique_fingerprint_overflow = False
        self.content_change_count = 0
        self.identical_consecutive_count = 0
        self.identical_with_advancing_header_count = 0
        self.previous_header_stamp = None
        self.latest_header_stamp = None
        self.latest_frame_id = None

    def observe(self, message):
        try:
            fingerprint = scan_payload_fingerprint(message)
        except Exception as error:
            self.error_count += 1
            if len(self.errors) < 8:
                self.errors.append("%s: %s" % (error.__class__.__name__, error))
            return
        stamp = stamp_seconds(message)
        current_frame = frame_id(message)
        if self.latest_fingerprint is not None:
            if fingerprint == self.latest_fingerprint:
                self.identical_consecutive_count += 1
                if stamp is not None and self.previous_header_stamp is not None and stamp > self.previous_header_stamp:
                    self.identical_with_advancing_header_count += 1
            else:
                self.content_change_count += 1
        if len(self.unique_fingerprints) < 64:
            self.unique_fingerprints.add(fingerprint)
        elif fingerprint not in self.unique_fingerprints:
            self.unique_fingerprint_overflow = True
        self.sample_count += 1
        self.first_fingerprint = fingerprint if self.first_fingerprint is None else self.first_fingerprint
        self.latest_fingerprint = fingerprint
        self.previous_header_stamp = stamp
        self.latest_header_stamp = stamp
        self.latest_frame_id = current_frame or self.latest_frame_id

    def report(self):
        status = "error" if self.error_count and not self.sample_count else ("no_data" if not self.sample_count else ("invalid" if self.error_count else "observation_ok"))
        return {
            "status": status,
            "sample_count": self.sample_count,
            "error_count": self.error_count,
            "errors": list(self.errors),
            "latest_header_stamp": self.latest_header_stamp,
            "latest_frame_id": self.latest_frame_id,
            "first_payload_sha256": self.first_fingerprint,
            "latest_payload_sha256": self.latest_fingerprint,
            "unique_payload_fingerprints_lower_bound": len(self.unique_fingerprints),
            "unique_payload_fingerprint_limit_reached": self.unique_fingerprint_overflow,
            "content_change_count": self.content_change_count,
            "identical_consecutive_count": self.identical_consecutive_count,
            "identical_content_with_advancing_header_count": self.identical_with_advancing_header_count,
            "all_successful_payloads_identical": self.sample_count > 1 and self.content_change_count == 0,
            "interpretation": "Payload identity and header progression are separate observations; neither establishes sensor freshness or physical response",
        }


class SensorContentProbe(object):
    def __init__(self, topics):
        self.lock = threading.RLock()
        self.topics = dict(topics)
        self.scan = ScanContentObservation()
        self.images = {"depth": None, "rgb": None}
        self.image_errors = {"depth": [], "rgb": []}
        self.frames = {"scan": None, "imu": None, "depth": None, "rgb": None}
        self.latest_header_stamps = {"scan": None, "imu": None, "depth": None, "rgb": None}
        self.received_counts = {"scan": 0, "imu": 0, "depth": 0, "rgb": 0}

    def observe_scan(self, message):
        with self.lock:
            self.received_counts["scan"] += 1
            self.frames["scan"] = frame_id(message) or self.frames["scan"]
            self.latest_header_stamps["scan"] = stamp_seconds(message)
            self.scan.observe(message)

    def observe_imu(self, message):
        with self.lock:
            self.received_counts["imu"] += 1
            self.frames["imu"] = frame_id(message) or self.frames["imu"]
            self.latest_header_stamps["imu"] = stamp_seconds(message)

    def observe_image(self, name, message):
        with self.lock:
            self.received_counts[name] += 1
            self.frames[name] = frame_id(message) or self.frames[name]
            self.latest_header_stamps[name] = stamp_seconds(message)
            if self.images[name] is not None or self.image_errors[name]:
                return
            try:
                self.images[name] = analyze_depth_image(message) if name == "depth" else analyze_rgb_image(message)
                self.images[name]["analyzed_header_stamp"] = stamp_seconds(message)
                self.images[name]["analyzed_frame_id"] = frame_id(message)
            except Exception as error:
                self.image_errors[name].append("%s: %s" % (error.__class__.__name__, error))

    def report(self, duration_requested=None, elapsed=None, transforms=None):
        with self.lock:
            image_reports = {}
            for name in ("depth", "rgb"):
                if self.images[name] is not None:
                    image_reports[name] = dict(self.images[name])
                elif self.image_errors[name]:
                    image_reports[name] = {"status": "error", "errors": list(self.image_errors[name])}
                else:
                    image_reports[name] = {"status": "no_data", "errors": []}
                image_reports[name]["received_count"] = self.received_counts[name]
                image_reports[name]["latest_header_stamp"] = self.latest_header_stamps[name]
                image_reports[name]["latest_frame_id"] = self.frames[name]
            scan_report = self.scan.report()
            scan_report["received_count"] = self.received_counts["scan"]
            imu_report = {
                "status": "observation_ok" if self.received_counts["imu"] else "no_data",
                "received_count": self.received_counts["imu"],
                "latest_header_stamp": self.latest_header_stamps["imu"],
                "latest_frame_id": self.frames["imu"],
                "interpretation": "Header and frame identity observation only; IMU accuracy, calibration and freshness are not established",
            }
            return {
                "schema_version": 1,
                "collection": {
                    "duration_requested_s": duration_requested,
                    "elapsed_monotonic_s": elapsed,
                    "mode": "ROS1 subscriber-only bounded sensor-content observation",
                    "payload_retention": "Only bounded hashes, counts and one-shot summaries are retained",
                },
                "topics": dict(self.topics),
                "observations": {"scan": scan_report, "imu": imu_report, "depth": image_reports["depth"], "rgb": image_reports["rgb"]},
                "transforms": transforms if transforms is not None else {},
                "all_hardware_accepted": False,
                "acceptance_statement": "Content and TF observations do not establish calibration, freshness, installation correctness or complete hardware acceptance",
            }


def lookup_transforms(listener, rospy_module, frames, base_frame="base_footprint"):
    transforms = {}
    for name in ("scan", "imu", "depth", "rgb"):
        sensor_frame = frames.get(name)
        if not sensor_frame:
            transforms[name] = {"status": "no_data", "target_frame": base_frame, "source_frame": None, "error": "sensor frame was not observed"}
            continue
        try:
            latest_time = listener.getLatestCommonTime(base_frame, sensor_frame)
            translation, rotation = listener.lookupTransform(base_frame, sensor_frame, latest_time)
            values = [float(value) for value in translation] + [float(value) for value in rotation] + [float(latest_time.to_sec())]
            if len(translation) != 3 or len(rotation) != 4 or any(math.isnan(value) or math.isinf(value) for value in values):
                raise ValueError("invalid or nonfinite transform")
            transforms[name] = {
                "status": "observation_ok",
                "target_frame": base_frame,
                "source_frame": sensor_frame,
                "latest_common_timestamp": float(latest_time.to_sec()),
                "translation": [float(value) for value in translation],
                "rotation_xyzw": [float(value) for value in rotation],
                "interpretation": "Latest available TF value only; actual calibration and physical mounting are not established",
            }
        except Exception as error:
            transforms[name] = {
                "status": "error",
                "target_frame": base_frame,
                "source_frame": sensor_frame,
                "error": "%s: %s" % (error.__class__.__name__, error),
            }
    return transforms


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description="Bounded ROS1 subscriber-only sensor content probe")
    parser.add_argument("--duration", type=float, default=7.0)
    parser.add_argument("--output", required=True)
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--depth-topic", default="/ascamera_hp60c/depth0/image_raw")
    parser.add_argument("--rgb-topic", default="/ascamera_hp60c/rgb0/image")
    arguments = parser.parse_args(argv)
    if os.path.lexists(arguments.output):
        parser.error("--output already exists; choose a new run path")
    if not 5.0 <= arguments.duration <= 10.0:
        parser.error("--duration must be between 5 and 10 seconds")
    return arguments


def run_ros(arguments):
    import rospy
    import tf
    from sensor_msgs.msg import Image, Imu, LaserScan

    topics = {
        "scan": arguments.scan_topic,
        "imu": arguments.imu_topic,
        "depth": arguments.depth_topic,
        "rgb": arguments.rgb_topic,
    }
    core = SensorContentProbe(topics)
    rospy.init_node("feedbot_sensor_content_probe", anonymous=True, disable_signals=True)
    listener = tf.TransformListener()
    subscribers = [
        rospy.Subscriber(arguments.scan_topic, LaserScan, core.observe_scan, queue_size=2),
        rospy.Subscriber(arguments.imu_topic, Imu, core.observe_imu, queue_size=2),
        rospy.Subscriber(arguments.depth_topic, Image, lambda message: core.observe_image("depth", message), queue_size=1),
        rospy.Subscriber(arguments.rgb_topic, Image, lambda message: core.observe_image("rgb", message), queue_size=1),
    ]
    started = monotonic_time()
    deadline = started + arguments.duration
    while not rospy.is_shutdown() and monotonic_time() < deadline:
        time.sleep(min(0.05, max(0.0, deadline - monotonic_time())))
    for subscriber in subscribers:
        subscriber.unregister()
    with core.lock:
        frames = dict(core.frames)
    transforms = lookup_transforms(listener, rospy, frames, arguments.base_frame)
    return core.report(arguments.duration, monotonic_time() - started, transforms)


def main(argv=None):
    arguments = parse_arguments(argv)
    if hasattr(signal, "alarm"):
        signal.alarm(int(math.ceil(arguments.duration)) + 10)
    try:
        report = run_ros(arguments)
        json.dumps(report, allow_nan=False)
    except Exception as error:
        report = {
            "schema_version": 1,
            "collector_error": "%s: %s" % (error.__class__.__name__, error),
            "all_hardware_accepted": False,
            "acceptance_statement": "Probe failure is not hardware acceptance",
        }
    finally:
        if hasattr(signal, "alarm"):
            signal.alarm(0)
    output_directory = os.path.dirname(os.path.abspath(arguments.output))
    if not os.path.isdir(output_directory):
        raise IOError("output directory does not exist: %s" % output_directory)
    write_json_exclusive(arguments.output, report)
    return 0 if "collector_error" not in report else 2


if __name__ == "__main__":
    sys.exit(main())

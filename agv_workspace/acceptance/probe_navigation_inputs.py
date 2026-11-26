#!/usr/bin/env python
from __future__ import print_function

import argparse
import json
import math
import os
import signal
import sys
import threading
import time

from collect_ros_readonly import bounded_count, finite_number, monotonic_time, write_json_exclusive


INTEGER_TYPES = (int, long) if sys.version_info[0] == 2 else (int,)
MAX_MAP_DIMENSION = 100000
MAX_MAP_CELLS_ANALYZED = 5000000
TF_QUERY_INTERVAL_S = 0.5


def stamp_seconds(message):
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    try:
        value = float(stamp.to_sec()) if hasattr(stamp, "to_sec") else float(stamp.secs) + float(stamp.nsecs) / 1e9
        return value if finite_number(value) and value > 0 else None
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def quaternion_yaw(quaternion):
    values = [getattr(quaternion, name, None) for name in ("x", "y", "z", "w")]
    if any(not finite_number(value) for value in values):
        raise ValueError("quaternion has missing or nonfinite components")
    x_value, y_value, z_value, w_value = [float(value) for value in values]
    norm = math.sqrt(sum(value * value for value in (x_value, y_value, z_value, w_value)))
    if abs(norm - 1.0) > 0.001:
        raise ValueError("quaternion norm is not within 0.001 of one")
    return math.atan2(2.0 * (w_value * z_value + x_value * y_value),
                      1.0 - 2.0 * (y_value * y_value + z_value * z_value))


def angle_delta(current, first):
    return math.atan2(math.sin(current - first), math.cos(current - first))


def add_count(counts, name):
    counts[name] = counts.get(name, 0) + 1


class StreamStats(object):
    def __init__(self):
        self.samples = 0
        self.first_monotonic = None
        self.last_monotonic = None
        self.max_gap = None
        self.receive_reversals = 0
        self.header_first = None
        self.header_last = None
        self.previous_header = None
        self.header_missing = 0
        self.header_duplicates = 0
        self.header_reversals = 0
        self.header_age_min = None
        self.header_age_max = None

    def observe(self, message, received_monotonic, received_wall):
        self.samples += 1
        if self.first_monotonic is None:
            self.first_monotonic = received_monotonic
        if self.last_monotonic is not None:
            gap = received_monotonic - self.last_monotonic
            if gap < 0:
                self.receive_reversals += 1
            else:
                self.max_gap = gap if self.max_gap is None else max(self.max_gap, gap)
        self.last_monotonic = received_monotonic
        stamp = stamp_seconds(message)
        if stamp is None:
            self.header_missing += 1
            return
        if self.header_first is None:
            self.header_first = stamp
        self.header_last = stamp
        if self.previous_header is not None:
            if stamp == self.previous_header:
                self.header_duplicates += 1
            elif stamp < self.previous_header:
                self.header_reversals += 1
        self.previous_header = stamp
        if finite_number(received_wall):
            age = float(received_wall) - stamp
            self.header_age_min = age if self.header_age_min is None else min(self.header_age_min, age)
            self.header_age_max = age if self.header_age_max is None else max(self.header_age_max, age)

    def report(self):
        span = None if self.samples < 2 else self.last_monotonic - self.first_monotonic
        frequency = None if span is None or span <= 0 else (self.samples - 1) / span
        return {
            "sample_count": self.samples, "receive_frequency_hz": frequency,
            "max_receive_gap_s": self.max_gap,
            "header_stamp_range": [self.header_first, self.header_last],
            "header_age_s_range": [self.header_age_min, self.header_age_max],
            "header_missing_or_invalid_count": self.header_missing,
            "header_duplicate_count": self.header_duplicates,
            "header_reversal_count": self.header_reversals,
            "receive_monotonic_reversal_count": self.receive_reversals,
        }


class CovarianceStats(object):
    def __init__(self):
        self.sample_count = 0
        self.malformed_count = 0
        self.nonfinite_count = 0
        self.zero_diagonal_sample_count = 0
        self.negative_diagonal_sample_count = 0

    def observe(self, values):
        self.sample_count += 1
        try:
            if len(values) != 36:
                raise ValueError("covariance length is not 36")
            diagonal = [values[index] for index in (0, 7, 14, 21, 28, 35)]
        except (TypeError, ValueError, IndexError):
            self.malformed_count += 1
            return False
        if any(not finite_number(value) for value in values):
            self.nonfinite_count += 1
            return False
        self.zero_diagonal_sample_count += int(any(float(value) == 0.0 for value in diagonal))
        negative = any(float(value) < 0.0 for value in diagonal)
        self.negative_diagonal_sample_count += int(negative)
        return not negative

    def report(self):
        return {
            "sample_count": self.sample_count, "malformed_count": self.malformed_count,
            "nonfinite_count": self.nonfinite_count,
            "zero_diagonal_sample_count": self.zero_diagonal_sample_count,
            "negative_diagonal_sample_count": self.negative_diagonal_sample_count,
        }


class PoseObservation(object):
    def __init__(self, topic, odometry=False):
        self.topic = topic
        self.odometry = odometry
        self.stream = StreamStats()
        self.invalid = {}
        self.errors = []
        self.first_position = None
        self.last_displacement = None
        self.max_displacement = None
        self.first_yaw = None
        self.last_yaw_drift = None
        self.max_abs_yaw_drift = None
        self.valid_quaternion_count = 0
        self.first_frame_id = None
        self.first_child_frame_id = None
        self.frame_ids = {}
        self.child_frame_ids = {}
        self.pose_covariance = CovarianceStats() if odometry else None
        self.twist_covariance = CovarianceStats() if odometry else None

    def observe(self, message, received_monotonic, received_wall):
        self.stream.observe(message, received_monotonic, received_wall)
        frame_id = getattr(getattr(message, "header", None), "frame_id", None)
        frame_id = str(frame_id).strip() if frame_id else None
        bounded_count(self.frame_ids, frame_id or "<missing>")
        frame_compatible = bool(frame_id)
        if not frame_id:
            add_count(self.invalid, "frame_id_missing")
        elif self.first_frame_id is None:
            self.first_frame_id = frame_id
        elif frame_id != self.first_frame_id:
            frame_compatible = False
            add_count(self.invalid, "frame_id_changed")
        if self.odometry:
            child_frame_id = getattr(message, "child_frame_id", None)
            child_frame_id = str(child_frame_id).strip() if child_frame_id else None
            bounded_count(self.child_frame_ids, child_frame_id or "<missing>")
            if not child_frame_id:
                add_count(self.invalid, "child_frame_id_missing")
            elif self.first_child_frame_id is None:
                self.first_child_frame_id = child_frame_id
            elif child_frame_id != self.first_child_frame_id:
                add_count(self.invalid, "child_frame_id_changed")
        pose = getattr(message, "pose", None)
        if self.odometry:
            pose = getattr(pose, "pose", None)
        position = getattr(pose, "position", None)
        coordinates = [getattr(position, name, None) for name in ("x", "y", "z")]
        if any(not finite_number(value) for value in coordinates):
            add_count(self.invalid, "position_missing_or_nonfinite")
        elif frame_compatible:
            point = tuple(float(value) for value in coordinates)
            if self.first_position is None:
                self.first_position = point
            displacement = math.sqrt(sum((point[index] - self.first_position[index]) ** 2 for index in range(3)))
            self.last_displacement = displacement
            self.max_displacement = displacement if self.max_displacement is None else max(self.max_displacement, displacement)
        try:
            yaw = quaternion_yaw(getattr(pose, "orientation", None))
            self.valid_quaternion_count += 1
            if frame_compatible:
                if self.first_yaw is None:
                    self.first_yaw = yaw
                drift = angle_delta(yaw, self.first_yaw)
                self.last_yaw_drift = drift
                absolute_drift = abs(drift)
                self.max_abs_yaw_drift = absolute_drift if self.max_abs_yaw_drift is None else max(self.max_abs_yaw_drift, absolute_drift)
        except ValueError:
            add_count(self.invalid, "quaternion_invalid")
        if self.odometry:
            if not self.pose_covariance.observe(getattr(getattr(message, "pose", None), "covariance", None)):
                add_count(self.invalid, "pose_covariance_invalid")
            if not self.twist_covariance.observe(getattr(getattr(message, "twist", None), "covariance", None)):
                add_count(self.invalid, "twist_covariance_invalid")

    def record_error(self, error):
        if len(self.errors) < 8:
            self.errors.append("%s: %s" % (error.__class__.__name__, error))

    def report(self):
        status = "missing" if not self.stream.samples else ("invalid" if self.invalid or self.errors else "observed")
        report = self.stream.report()
        report.update({
            "status": status, "topic": self.topic, "invalid_observations": dict(self.invalid),
            "errors": list(self.errors), "valid_quaternion_count": self.valid_quaternion_count,
            "frame_ids": dict(self.frame_ids), "child_frame_ids": dict(self.child_frame_ids),
            "position_drift_from_first_m": self.last_displacement,
            "max_position_displacement_from_first_m": self.max_displacement,
            "yaw_drift_from_first_rad": self.last_yaw_drift,
            "max_abs_yaw_displacement_from_first_rad": self.max_abs_yaw_drift,
        })
        if self.odometry:
            report["pose_covariance"] = self.pose_covariance.report()
            report["twist_covariance"] = self.twist_covariance.report()
        else:
            report["covariance"] = {"status": "not_provided_by_PoseStamped"}
        return report


class MapObservation(object):
    def __init__(self, topic):
        self.topic = topic
        self.stream = StreamStats()
        self.summary = None
        self.invalid = {}
        self.errors = []
        self.frame_ids = {}
        self.first_frame_id = None
        self.map_load_time_first = None
        self.map_load_time_last = None
        self.map_load_time_change_count = 0
        self.header_seq_first = None
        self.header_seq_last = None

    def observe(self, message, received_monotonic, received_wall):
        self.stream.observe(message, received_monotonic, received_wall)
        header = getattr(message, "header", None)
        frame_id = getattr(header, "frame_id", None)
        frame_id = str(frame_id).strip() if frame_id else None
        bounded_count(self.frame_ids, frame_id or "<missing>")
        if not frame_id:
            add_count(self.invalid, "frame_id_missing")
        elif self.first_frame_id is None:
            self.first_frame_id = frame_id
        elif frame_id != self.first_frame_id:
            add_count(self.invalid, "frame_id_changed")
        sequence = getattr(header, "seq", None)
        if isinstance(sequence, INTEGER_TYPES):
            self.header_seq_first = sequence if self.header_seq_first is None else self.header_seq_first
            self.header_seq_last = sequence
        info = getattr(message, "info", None)
        map_load_stamp = getattr(info, "map_load_time", None)
        try:
            map_load_time = float(map_load_stamp.to_sec())
            if not finite_number(map_load_time) or map_load_time < 0:
                raise ValueError("invalid map_load_time")
            if self.map_load_time_first is None:
                self.map_load_time_first = map_load_time
            elif map_load_time != self.map_load_time_last:
                self.map_load_time_change_count += 1
                add_count(self.invalid, "map_load_time_changed")
            self.map_load_time_last = map_load_time
        except (AttributeError, TypeError, ValueError, OverflowError):
            add_count(self.invalid, "map_load_time_missing_or_invalid")
        if self.summary is not None or self.errors:
            return
        width, height = getattr(info, "width", None), getattr(info, "height", None)
        resolution = getattr(info, "resolution", None)
        valid_dimensions = (isinstance(width, INTEGER_TYPES) and isinstance(height, INTEGER_TYPES) and
                            0 < width <= MAX_MAP_DIMENSION and 0 < height <= MAX_MAP_DIMENSION)
        if not valid_dimensions:
            add_count(self.invalid, "dimensions_invalid_or_over_limit")
        if not finite_number(resolution) or float(resolution) <= 0:
            add_count(self.invalid, "resolution_invalid")
        cell_count = width * height if valid_dimensions else None
        if cell_count is not None and cell_count > MAX_MAP_CELLS_ANALYZED:
            add_count(self.invalid, "cell_count_over_analysis_limit")
        origin = getattr(info, "origin", None)
        origin_position = getattr(origin, "position", None)
        if any(not finite_number(getattr(origin_position, name, None)) for name in ("x", "y", "z")):
            add_count(self.invalid, "origin_position_invalid")
        try:
            quaternion_yaw(getattr(origin, "orientation", None))
        except ValueError:
            add_count(self.invalid, "origin_quaternion_invalid")
        data = getattr(message, "data", None)
        try:
            payload_length = len(data)
        except TypeError:
            payload_length = None
            add_count(self.invalid, "payload_missing")
        if cell_count is not None and payload_length != cell_count:
            add_count(self.invalid, "payload_length_mismatch")
        counts = None
        if cell_count is not None and cell_count <= MAX_MAP_CELLS_ANALYZED and payload_length == cell_count:
            counts = {"unknown": 0, "free": 0, "occupied": 0, "intermediate": 0, "invalid": 0}
            for value in data:
                if value == -1:
                    counts["unknown"] += 1
                elif value == 0:
                    counts["free"] += 1
                elif value == 100:
                    counts["occupied"] += 1
                elif isinstance(value, INTEGER_TYPES) and 0 < value < 100:
                    counts["intermediate"] += 1
                else:
                    counts["invalid"] += 1
            if counts["invalid"]:
                add_count(self.invalid, "payload_value_out_of_range")
        self.summary = {
            "width": width if isinstance(width, INTEGER_TYPES) else None,
            "height": height if isinstance(height, INTEGER_TYPES) else None,
            "resolution_m_per_cell": float(resolution) if finite_number(resolution) else None,
            "declared_cell_count": cell_count, "payload_length": payload_length,
            "bounded_occupancy_counts": counts, "payload_retained": False,
            "analysis_cell_limit": MAX_MAP_CELLS_ANALYZED,
        }

    def record_error(self, error):
        if len(self.errors) < 8:
            self.errors.append("%s: %s" % (error.__class__.__name__, error))

    def report(self):
        status = "missing" if not self.stream.samples else ("invalid" if self.invalid or self.errors else "observed")
        report = self.stream.report()
        report.update({"status": status, "topic": self.topic, "one_shot_analysis": True,
                       "summary": self.summary, "invalid_observations": dict(self.invalid), "errors": list(self.errors),
                       "frame_ids": dict(self.frame_ids),
                       "map_load_time_range": [self.map_load_time_first, self.map_load_time_last],
                       "map_load_time_change_count": self.map_load_time_change_count,
                       "header_seq_range": [self.header_seq_first, self.header_seq_last]})
        return report


class TransformObservation(object):
    def __init__(self, target, source):
        self.target, self.source = target, source
        self.success_count = 0
        self.error_count = 0
        self.max_header_age = None
        self.latest_success = None
        self.errors = []

    def lookup(self, listener, stamp, stamp_value, received_wall):
        if finite_number(received_wall):
            age = float(received_wall) - stamp_value
            self.max_header_age = age if self.max_header_age is None else max(self.max_header_age, age)
        try:
            translation, rotation = listener.lookupTransform(self.target, self.source, stamp)
            values = list(translation) + list(rotation)
            if len(translation) != 3 or len(rotation) != 4 or any(not finite_number(value) for value in values):
                raise ValueError("transform has invalid dimensions or nonfinite values")
            quaternion_yaw(type("Quaternion", (), dict(zip(("x", "y", "z", "w"), rotation)))())
            self.success_count += 1
            self.latest_success = {"scan_header_stamp": stamp_value,
                                   "translation_xyz": [float(value) for value in translation],
                                   "rotation_xyzw": [float(value) for value in rotation]}
        except Exception as error:
            self.error_count += 1
            if len(self.errors) < 8:
                self.errors.append("%s: %s" % (error.__class__.__name__, error))

    def report(self):
        status = "missing" if not self.success_count and not self.error_count else ("invalid" if self.error_count else "observed")
        return {"status": status, "target_frame": self.target, "source_frame": self.source,
                "success_count": self.success_count, "error_count": self.error_count,
                "max_scan_header_age_s": self.max_header_age, "latest_success": self.latest_success,
                "errors": list(self.errors),
                "interpretation": "Exact received scan timestamps were queried; this is availability evidence, not transform accuracy"}


class NavigationProbe(object):
    def __init__(self, topics, listener=None, base_frame="base_footprint"):
        self.lock = threading.RLock()
        self.topics = dict(topics)
        self.listener = listener
        self.map = MapObservation(topics["map"])
        self.poses = {
            "tracked_pose": PoseObservation(topics.get("tracked_pose"), False),
            "wheel_odom": PoseObservation(topics["wheel_odom"], True),
            "raw_odom": PoseObservation(topics["raw_odom"], True),
        }
        self.scan = StreamStats()
        self.last_tf_query_monotonic = None
        self.tf_stamp_invalid_count = 0
        self.transforms = dict((target, TransformObservation(target, base_frame)) for target in ("map", "odom"))

    def observe(self, name, message, received_monotonic=None, received_wall=None):
        received_monotonic = monotonic_time() if received_monotonic is None else received_monotonic
        received_wall = time.time() if received_wall is None else received_wall
        observation = self.map if name == "map" else self.poses[name]
        with self.lock:
            try:
                observation.observe(message, received_monotonic, received_wall)
            except Exception as error:
                observation.record_error(error)

    def observe_scan(self, message, received_monotonic=None, received_wall=None):
        received_monotonic = monotonic_time() if received_monotonic is None else received_monotonic
        received_wall = time.time() if received_wall is None else received_wall
        with self.lock:
            self.scan.observe(message, received_monotonic, received_wall)
            stamp_value = stamp_seconds(message)
            stamp = getattr(getattr(message, "header", None), "stamp", None)
            if stamp_value is None:
                self.tf_stamp_invalid_count += 1
                return
            if (self.last_tf_query_monotonic is not None and
                    received_monotonic - self.last_tf_query_monotonic < TF_QUERY_INTERVAL_S):
                return
            self.last_tf_query_monotonic = received_monotonic
            if self.listener is None:
                for observation in self.transforms.values():
                    observation.error_count += 1
                    if len(observation.errors) < 8:
                        observation.errors.append("transform listener is unavailable")
                return
            for observation in self.transforms.values():
                observation.lookup(self.listener, stamp, stamp_value, received_wall)

    def report(self, duration=None, start_wall=None, elapsed=None):
        with self.lock:
            topic_reports = {"map": self.map.report()}
            topic_reports.update(dict((name, observation.report()) for name, observation in self.poses.items()))
            scan_report = self.scan.report()
            scan_status = "missing" if not self.scan.samples else ("invalid" if self.tf_stamp_invalid_count else "observed")
            scan_report.update({"status": scan_status,
                                "topic": self.topics["scan"], "tf_stamp_invalid_count": self.tf_stamp_invalid_count})
            topic_reports["scan"] = scan_report
            if not self.topics.get("tracked_pose"):
                topic_reports["tracked_pose"]["status"] = "not_configured"
            return json.loads(json.dumps({
                "schema_version": 1,
                "provenance": {"start_wall_epoch_s": start_wall, "elapsed_monotonic_s": elapsed,
                               "duration_requested_s": duration, "assumed_stationary": False,
                               "operator_state": "not_inferred"},
                "mode": "ROS1 subscriber-only bounded navigation input observation",
                "topics": topic_reports,
                "transforms_at_received_scan_stamp": dict((name, value.report()) for name, value in self.transforms.items()),
                "all_hardware_accepted": False,
                "acceptance_statement": "Observed metadata, drift and TF availability do not establish localization or navigation accuracy",
            }, allow_nan=False))


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description="Bounded read-only ROS1 navigation input probe")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--output", required=True)
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--tracked-pose-topic", default="/tracked_pose")
    parser.add_argument("--wheel-odom-topic", default="/wheel_odom")
    parser.add_argument("--raw-odom-topic", default="/raw_odom")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--base-frame", default="base_footprint")
    arguments = parser.parse_args(argv)
    if os.path.lexists(arguments.output):
        parser.error("--output already exists; choose a new run path")
    if not 5.0 <= arguments.duration <= 60.0:
        parser.error("--duration must be between 5 and 60 seconds")
    return arguments


def run_ros(arguments):
    import rospy
    import tf
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import LaserScan

    topics = {"map": arguments.map_topic, "tracked_pose": arguments.tracked_pose_topic,
              "wheel_odom": arguments.wheel_odom_topic, "raw_odom": arguments.raw_odom_topic,
              "scan": arguments.scan_topic}
    rospy.init_node("feedbot_navigation_input_probe", anonymous=True, disable_signals=True)
    listener = tf.TransformListener(cache_time=rospy.Duration(10.0))
    core = NavigationProbe(topics, listener, arguments.base_frame)
    subscribers = [
        rospy.Subscriber(arguments.map_topic, OccupancyGrid, lambda message: core.observe("map", message), queue_size=1),
        rospy.Subscriber(arguments.wheel_odom_topic, Odometry, lambda message: core.observe("wheel_odom", message), queue_size=5),
        rospy.Subscriber(arguments.raw_odom_topic, Odometry, lambda message: core.observe("raw_odom", message), queue_size=5),
        rospy.Subscriber(arguments.scan_topic, LaserScan, core.observe_scan, queue_size=2),
    ]
    if arguments.tracked_pose_topic:
        subscribers.append(rospy.Subscriber(arguments.tracked_pose_topic, PoseStamped,
                                            lambda message: core.observe("tracked_pose", message), queue_size=5))
    start_wall, started = time.time(), monotonic_time()
    deadline = started + arguments.duration
    while not rospy.is_shutdown():
        remaining = deadline - monotonic_time()
        if remaining <= 0:
            break
        time.sleep(min(0.05, remaining))
    for subscriber in subscribers:
        subscriber.unregister()
    return core.report(arguments.duration, start_wall, monotonic_time() - started)


def main(argv=None):
    arguments = parse_arguments(argv)
    if hasattr(signal, "alarm"):
        signal.alarm(int(math.ceil(arguments.duration)) + 10)
    try:
        report = run_ros(arguments)
        json.dumps(report, allow_nan=False)
    except Exception as error:
        report = {"schema_version": 1, "collector_error": "%s: %s" % (error.__class__.__name__, error),
                  "all_hardware_accepted": False,
                  "acceptance_statement": "Probe failure is not hardware acceptance"}
    finally:
        if hasattr(signal, "alarm"):
            signal.alarm(0)
    directory = os.path.dirname(os.path.abspath(arguments.output))
    if not os.path.isdir(directory):
        raise IOError("output directory does not exist: %s" % directory)
    write_json_exclusive(arguments.output, report)
    return 0 if "collector_error" not in report else 2


if __name__ == "__main__":
    sys.exit(main())

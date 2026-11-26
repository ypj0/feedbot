from __future__ import print_function

import math
import json
import os
import sys
import unittest
import tempfile


sys.path.insert(0, os.path.dirname(__file__))
from collect_ros_readonly import CollectorCore, write_json_exclusive


class Value(object):
    def __init__(self, **values):
        self.__dict__.update(values)


def header(seconds=100.0, frame_id="sensor"):
    whole = int(seconds)
    return Value(stamp=Value(secs=whole, nsecs=int((seconds - whole) * 1000000000)), frame_id=frame_id)


def quaternion():
    return Value(x=0.0, y=0.0, z=0.0, w=1.0)


def scan_message(ranges=None, seconds=100.0):
    return Value(
        header=header(seconds), angle_min=0.0, angle_max=0.2,
        angle_increment=0.1, range_min=0.1, range_max=10.0,
        ranges=[1.0, 2.0, 3.0] if ranges is None else ranges,
        intensities=[],
    )


def imu_message():
    covariance = [0.0] * 9
    return Value(
        header=header(), orientation=quaternion(),
        angular_velocity=Value(x=0.1, y=0.2, z=0.3),
        linear_acceleration=Value(x=0.0, y=0.0, z=9.8),
        orientation_covariance=covariance,
        angular_velocity_covariance=covariance,
        linear_acceleration_covariance=covariance,
    )


class CollectorTests(unittest.TestCase):
    def test_warmup_does_not_hide_whole_run_gap(self):
        core = CollectorCore([("scan", "/scan", "scan")])
        for received in (0.0, 0.75, 2.0, 2.1, 2.2):
            core.observe("scan", scan_message(), received, 100.0 + received)
        topic = core.report()["topics"]["scan"]
        self.assertAlmostEqual(1.25, topic["max_receive_gap_s"])
        self.assertEqual(2.0, topic["max_gap_ending_offset_from_first_sample_s"])
        self.assertEqual(2, topic["post_warmup_receive_gaps"]["gap_count"])
        self.assertAlmostEqual(0.1, topic["post_warmup_receive_gaps"]["max_gap_s"])

    def test_warmup_without_valid_pairs_is_unavailable(self):
        core = CollectorCore([("scan", "/scan", "scan")])
        core.observe("scan", scan_message(), 0.0, 100.0)
        core.observe("scan", scan_message(), 3.0, 103.0)
        core.observe("scan", scan_message(), 2.5, 102.5)
        topic = core.report()["topics"]["scan"]
        self.assertIsNone(topic["post_warmup_receive_gaps"]["max_gap_s"])
        self.assertEqual(1, topic["header_clock_observations"]["receive_monotonic_reversal_count"])

    def test_report_commit_never_overwrites_evidence(self):
        directory = tempfile.mkdtemp()
        path = os.path.join(directory, "report.json")
        try:
            write_json_exclusive(path, {"original": True})
            with self.assertRaises(OSError):
                write_json_exclusive(path, {"replacement": True})
            with open(path) as handle:
                self.assertEqual({"original": True}, json.load(handle))
            self.assertEqual(["report.json"], os.listdir(directory))
        finally:
            os.unlink(path)
            os.rmdir(directory)

    def test_nonfinite_report_never_creates_partial_evidence(self):
        directory = tempfile.mkdtemp()
        path = os.path.join(directory, "report.json")
        try:
            with self.assertRaises(ValueError):
                write_json_exclusive(path, {"invalid": float("nan")})
            self.assertFalse(os.path.exists(path))
            self.assertEqual([], os.listdir(directory))
        finally:
            os.rmdir(directory)

    def test_normal_scan_reports_observation_only(self):
        core = CollectorCore([("scan", "/scan", "scan")])
        core.observe("scan", scan_message(), 10.0, 100.1)
        core.observe("scan", scan_message(seconds=100.1), 10.1, 100.2)
        report = core.report(1.0, 1.0)
        self.assertEqual("observation_ok", report["topics"]["scan"]["status"])
        self.assertEqual(2, report["topics"]["scan"]["sample_count"])
        self.assertFalse(report["all_hardware_accepted"])

    def test_bad_scan_length_and_nan_are_invalid(self):
        core = CollectorCore([("scan", "/scan", "scan")])
        core.observe("scan", scan_message([1.0, float("nan")]), 1.0, 100.0)
        topic = core.report()["topics"]["scan"]
        self.assertEqual("invalid", topic["status"])
        self.assertEqual(1, topic["issues"]["ranges_geometry_length"])
        self.assertEqual(1, topic["issues"]["range_nan"])

    def test_current_factory_scan_geometry_is_rejected(self):
        message = scan_message([1.0] * 1075)
        message.angle_min = -1.649999976158142
        message.angle_max = 1.649999976158142
        message.angle_increment = 0.003067961661145091
        core = CollectorCore([("scan", "/scan", "scan")])
        core.observe("scan", message, 1.0, 100.0)
        self.assertEqual(1, core.report()["topics"]["scan"]["issues"]["ranges_geometry_length"])
        message.angle_max = message.angle_min + 1074 * message.angle_increment
        corrected = CollectorCore([("scan", "/scan", "scan")])
        corrected.observe("scan", message, 1.0, 100.0)
        self.assertEqual("observation_ok", corrected.report()["topics"]["scan"]["status"])

    def test_header_and_receive_time_reversal_are_counted(self):
        core = CollectorCore([("scan", "/scan", "scan")])
        core.observe("scan", scan_message(seconds=101.0), 2.0, 100.0)
        core.observe("scan", scan_message(seconds=100.0), 1.0, 100.1)
        clocks = core.report()["topics"]["scan"]["header_clock_observations"]
        self.assertEqual(1, clocks["reversal_count"])
        self.assertEqual(1, clocks["receive_monotonic_reversal_count"])
        self.assertEqual(1, clocks["future_count"])

    def test_no_data_is_not_pass(self):
        core = CollectorCore([("imu", "/imu/data", "imu")])
        topic = core.report()["topics"]["imu"]
        self.assertEqual("no_data", topic["status"])
        self.assertFalse(core.report()["all_hardware_accepted"])

    def test_nonfinite_imu_is_invalid(self):
        message = imu_message()
        message.angular_velocity.x = float("nan")
        core = CollectorCore([("imu", "/imu/data", "imu")])
        core.observe("imu", message, 1.0, 100.0)
        topic = core.report()["topics"]["imu"]
        self.assertEqual("invalid", topic["status"])
        self.assertIn("angular_velocity.x_nonfinite", topic["issues"])

    def test_missing_camera_calibration_is_invalid(self):
        message = Value(
            header=header(), width=640, height=480,
            K=[0.0] * 9, P=[0.0] * 12,
        )
        core = CollectorCore([("camera", "/camera_info", "camera_info")])
        core.observe("camera", message, 1.0, 100.0)
        topic = core.report()["topics"]["camera"]
        self.assertEqual("invalid", topic["status"])
        self.assertEqual(1, topic["issues"]["uncalibrated"])

    def test_image_checks_data_size_without_storing_payload(self):
        message = Value(
            header=header(), width=2, height=2, step=6,
            encoding="rgb8", data=bytearray(11),
        )
        core = CollectorCore([("image", "/image", "image")])
        core.observe("image", message, 1.0, 100.0)
        topic = core.report()["topics"]["image"]
        self.assertEqual("invalid", topic["status"])
        self.assertEqual(1, topic["issues"]["data_shorter_than_step_times_height"])
        self.assertNotIn("data", topic["summary"])

    def test_unavailable_imu_orientation_is_not_a_bad_quaternion(self):
        message = imu_message()
        message.orientation = Value(x=0, y=0, z=0, w=0)
        message.orientation_covariance = [-1] + [0] * 8
        core = CollectorCore([("imu", "/imu/data", "imu")])
        core.observe("imu", message, 1.0, 100.0)
        topic = core.report()["topics"]["imu"]
        self.assertEqual("observation_ok", topic["status"])
        self.assertEqual(1, topic["summary"]["observations"]["orientation_covariance_unavailable"])

    def test_partial_driver_message_cannot_look_complete(self):
        core = CollectorCore([("driver", "/driver_info", "driver")])
        core.observe("driver", Value(Power=86), 1.0, 100.0)
        topic = core.report()["topics"]["driver"]
        self.assertEqual("invalid", topic["status"])
        self.assertIn("missing_field.EmergencyStatus", topic["issues"])

    def test_validator_exceptions_are_recorded(self):
        core = CollectorCore([("camera", "/camera_info", "camera_info")])
        core.observe("camera", Value(width=640, height=480, K=None, P=None), 1.0, 100.0)
        self.assertEqual("error", core.report()["topics"]["camera"]["status"])

    def test_frame_identity_memory_is_bounded_and_report_is_snapshot(self):
        core = CollectorCore([("scan", "/scan", "scan")])
        for index in range(100):
            message = scan_message(seconds=100.0 + index)
            message.header.frame_id = "frame_" + str(index)
            core.observe("scan", message, float(index), 100.0 + index)
        first = core.report()
        self.assertLessEqual(len(first["topics"]["scan"]["frame_ids"]), 17)
        core.observe("scan", scan_message(), 101.0, 201.0)
        self.assertEqual(100, first["topics"]["scan"]["sample_count"])
        self.assertEqual(101, core.report()["topics"]["scan"]["sample_count"])


if __name__ == "__main__":
    unittest.main()

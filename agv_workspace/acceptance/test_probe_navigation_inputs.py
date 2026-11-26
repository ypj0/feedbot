from __future__ import print_function

import json
import math
import os
import sys
import unittest


sys.path.insert(0, os.path.dirname(__file__))
from probe_navigation_inputs import MAX_MAP_CELLS_ANALYZED, NavigationProbe, quaternion_yaw


class Value(object):
    def __init__(self, **values):
        self.__dict__.update(values)


class FakeStamp(object):
    def __init__(self, seconds):
        self.seconds = seconds
        self.secs = int(seconds)
        self.nsecs = int((seconds - self.secs) * 1000000000)

    def to_sec(self):
        return self.seconds


def quaternion(yaw=0.0):
    return Value(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def header(seconds=100.0, frame="frame"):
    return Value(stamp=FakeStamp(seconds), frame_id=frame)


def pose_message(x=0.0, y=0.0, yaw=0.0, seconds=100.0, frame="map"):
    return Value(header=header(seconds, frame), pose=Value(position=Value(x=x, y=y, z=0.0), orientation=quaternion(yaw)))


def odom_message(x=0.0, y=0.0, yaw=0.0, seconds=100.0, covariance=None):
    covariance = [0.0] * 36 if covariance is None else covariance
    pose = pose_message(x, y, yaw, seconds)
    return Value(header=pose.header, child_frame_id="base_footprint", pose=Value(pose=pose.pose, covariance=list(covariance)),
                 twist=Value(covariance=list(covariance)))


def map_message(width=2, height=2, data=None):
    map_header = header(100.0, "map")
    map_header.seq = 7
    return Value(header=map_header, info=Value(width=width, height=height, resolution=0.05,
                 map_load_time=FakeStamp(99.0),
                 origin=Value(position=Value(x=0.0, y=0.0, z=0.0), orientation=quaternion())),
                 data=[-1, 0, 50, 100] if data is None else data)


def scan_message(seconds=100.0):
    return Value(header=header(seconds, "scan"))


def topics():
    return {"map": "/map", "tracked_pose": "/tracked_pose", "wheel_odom": "/wheel_odom",
            "raw_odom": "/raw_odom", "scan": "/scan"}


class FakeTF(object):
    def __init__(self, fail_target=None):
        self.fail_target = fail_target
        self.calls = []

    def lookupTransform(self, target, source, stamp):
        self.calls.append((target, source, stamp))
        if target == self.fail_target:
            raise LookupError("missing transform")
        return [1.0, 2.0, 0.0], [0.0, 0.0, 0.0, 1.0]


class NavigationProbeTests(unittest.TestCase):
    def test_finite_and_quaternion_validation(self):
        self.assertAlmostEqual(0.5, quaternion_yaw(quaternion(0.5)))
        with self.assertRaises(ValueError):
            quaternion_yaw(Value(x=float("nan"), y=0, z=0, w=1))
        with self.assertRaises(ValueError):
            quaternion_yaw(Value(x=0, y=0, z=0, w=0))

    def test_map_dimensions_and_bounded_counts(self):
        core = NavigationProbe(topics())
        core.observe("map", map_message(), 1.0, 100.1)
        report = core.report()["topics"]["map"]
        self.assertEqual("observed", report["status"])
        self.assertEqual({"unknown": 1, "free": 1, "occupied": 1, "intermediate": 1, "invalid": 0},
                         report["summary"]["bounded_occupancy_counts"])
        self.assertFalse(report["summary"]["payload_retained"])
        invalid = NavigationProbe(topics())
        invalid.observe("map", map_message(MAX_MAP_CELLS_ANALYZED + 1, 1, []), 1.0, 100.1)
        bad = invalid.report()["topics"]["map"]
        self.assertEqual("invalid", bad["status"])
        self.assertIn("dimensions_invalid_or_over_limit", bad["invalid_observations"])

        over_cells = NavigationProbe(topics())
        over_cells.observe("map", map_message(5000, 2000, []), 1.0, 100.1)
        self.assertIn("cell_count_over_analysis_limit",
                      over_cells.report()["topics"]["map"]["invalid_observations"])

        malformed = map_message()
        malformed.info.resolution = float("nan")
        malformed.info.width = object()
        core = NavigationProbe(topics())
        core.observe("map", malformed, 1.0, 100.1)
        malformed_report = core.report()["topics"]["map"]
        self.assertEqual("invalid", malformed_report["status"])
        json.dumps(malformed_report, allow_nan=False)

    def test_pose_and_odom_drift_covariance(self):
        core = NavigationProbe(topics())
        core.observe("tracked_pose", pose_message(1.0, 1.0, 0.1), 1.0, 100.1)
        core.observe("tracked_pose", pose_message(4.0, 5.0, 0.4, 101.0), 2.0, 101.1)
        covariance = [1.0] * 36
        covariance[0] = -1.0
        core.observe("wheel_odom", odom_message(covariance=covariance), 1.0, 100.1)
        tracked = core.report()["topics"]["tracked_pose"]
        wheel = core.report()["topics"]["wheel_odom"]
        self.assertAlmostEqual(5.0, tracked["position_drift_from_first_m"])
        self.assertAlmostEqual(0.3, tracked["yaw_drift_from_first_rad"])
        self.assertEqual("invalid", wheel["status"])
        self.assertEqual(1, wheel["pose_covariance"]["negative_diagonal_sample_count"])

    def test_frame_change_is_flagged_and_excluded_from_drift(self):
        core = NavigationProbe(topics())
        core.observe("tracked_pose", pose_message(0.0, 0.0, frame="map"), 1.0, 100.1)
        core.observe("tracked_pose", pose_message(100.0, 0.0, frame="odom"), 2.0, 101.1)
        report = core.report()["topics"]["tracked_pose"]
        self.assertEqual("invalid", report["status"])
        self.assertEqual(1, report["invalid_observations"]["frame_id_changed"])
        self.assertEqual(0.0, report["max_position_displacement_from_first_m"])
        self.assertEqual({"map": 1, "odom": 1}, report["frame_ids"])

        first_odom = odom_message()
        changed_odom = odom_message()
        changed_odom.child_frame_id = "base_link"
        core.observe("raw_odom", first_odom, 1.0, 100.1)
        core.observe("raw_odom", changed_odom, 2.0, 101.1)
        raw = core.report()["topics"]["raw_odom"]
        self.assertEqual(1, raw["invalid_observations"]["child_frame_id_changed"])

    def test_no_data_is_missing_and_never_acceptance(self):
        report = NavigationProbe(topics()).report(5.0, 100.0, 5.0)
        self.assertEqual("missing", report["topics"]["map"]["status"])
        self.assertEqual("missing", report["topics"]["raw_odom"]["status"])
        self.assertFalse(report["all_hardware_accepted"])
        self.assertFalse(report["provenance"]["assumed_stationary"])
        json.dumps(report, allow_nan=False)

    def test_time_backwards_and_header_reversal(self):
        core = NavigationProbe(topics())
        core.observe("tracked_pose", pose_message(seconds=101.0), 2.0, 101.1)
        core.observe("tracked_pose", pose_message(seconds=100.0), 1.0, 100.1)
        report = core.report()["topics"]["tracked_pose"]
        self.assertEqual(1, report["header_reversal_count"])
        self.assertEqual(1, report["receive_monotonic_reversal_count"])
        self.assertIsNone(report["receive_frequency_hz"])

    def test_tf_uses_exact_scan_stamp_throttles_and_preserves_failure(self):
        listener = FakeTF("odom")
        core = NavigationProbe(topics(), listener)
        first = scan_message(100.0)
        core.observe_scan(first, 1.0, 100.2)
        core.observe_scan(scan_message(100.1), 1.1, 100.3)
        core.observe_scan(scan_message(101.0), 1.5, 101.3)
        report = core.report()
        self.assertEqual(4, len(listener.calls))
        self.assertIs(first.header.stamp, listener.calls[0][2])
        self.assertEqual("observed", report["transforms_at_received_scan_stamp"]["map"]["status"])
        odom = report["transforms_at_received_scan_stamp"]["odom"]
        self.assertEqual("invalid", odom["status"])
        self.assertEqual(2, odom["error_count"])
        self.assertIn("missing transform", odom["errors"][0])
        self.assertAlmostEqual(0.3, odom["max_scan_header_age_s"])


if __name__ == "__main__":
    unittest.main()

from __future__ import print_function

import os
import json
import struct
import sys
import unittest
import tempfile


sys.path.insert(0, os.path.dirname(__file__))
from probe_sensor_content import ScanContentObservation, SensorContentProbe, analyze_depth_image, analyze_rgb_image, lookup_transforms
import probe_sensor_content


class Value(object):
    def __init__(self, **values):
        self.__dict__.update(values)


def header(seconds=100.0, frame="sensor"):
    whole = int(seconds)
    return Value(stamp=Value(secs=whole, nsecs=int((seconds - whole) * 1000000000)), frame_id=frame)


def image(encoding, width, height, step, data, big_endian=False):
    return Value(header=header(), encoding=encoding, width=width, height=height, step=step, data=data, is_bigendian=big_endian)


def scan(ranges, seconds=100.0):
    return Value(
        header=header(seconds, "scan"),
        angle_min=-1.0,
        angle_max=1.0,
        angle_increment=0.5,
        time_increment=0.01,
        scan_time=0.1,
        range_min=0.05,
        range_max=20.0,
        ranges=ranges,
        intensities=[1.0] * len(ranges),
    )


class SensorContentTests(unittest.TestCase):
    def test_little_endian_depth_ignores_row_padding(self):
        payload = struct.pack("<HH", 0, 500) + b"\xff\xff" + struct.pack("<HH", 1000, 65535) + b"\x00\x00"
        summary = analyze_depth_image(image("16UC1", 2, 2, 6, payload))
        self.assertEqual(2, summary["row_padding_bytes"])
        self.assertEqual(1, summary["zero_count"])
        self.assertEqual(3, summary["nonzero_count"])
        self.assertEqual(500, summary["nonzero_raw_min"])
        self.assertEqual(65535, summary["nonzero_raw_max"])
        self.assertEqual(1, summary["nonzero_raw_distribution"][1]["count"])

    def test_big_endian_depth_is_decoded(self):
        payload = struct.pack(">HHH", 1, 256, 4096)
        summary = analyze_depth_image(image("16UC1", 3, 1, 6, payload, True))
        self.assertTrue(summary["is_bigendian"])
        self.assertEqual(1, summary["nonzero_raw_min"])
        self.assertEqual(4096, summary["nonzero_raw_max"])

    def test_depth_rejects_bad_encoding_and_truncation(self):
        with self.assertRaises(ValueError):
            analyze_depth_image(image("32FC1", 1, 1, 4, b"\x00" * 4))
        with self.assertRaises(ValueError):
            analyze_depth_image(image("16UC1", 2, 2, 4, b"\x00" * 7))

    def test_rgb_diversity_excludes_padding(self):
        payload = bytes(bytearray([0, 0, 0, 255, 128, 64, 17, 18]))
        summary = analyze_rgb_image(image("rgb8", 2, 1, 8, payload))
        self.assertEqual(2, summary["row_padding_bytes"])
        self.assertEqual(4, summary["distinct_byte_values"])
        self.assertEqual([2, 2, 2], summary["distinct_values_by_channel"])
        self.assertEqual(6, summary["analyzed_byte_count"])

    def test_rgb_rejects_malformed_layout(self):
        with self.assertRaises(ValueError):
            analyze_rgb_image(image("rgb8", 2, 1, 5, b"\x00" * 5))
        with self.assertRaises(ValueError):
            analyze_rgb_image(image("yuv422", 2, 1, 4, b"\x00" * 4))

    def test_constant_scan_with_advancing_header_is_detected(self):
        observation = ScanContentObservation()
        observation.observe(scan([1.0, 2.0], 100.0))
        observation.observe(scan([1.0, 2.0], 100.1))
        observation.observe(scan([1.0, 2.0], 100.2))
        report = observation.report()
        self.assertTrue(report["all_successful_payloads_identical"])
        self.assertEqual(2, report["identical_content_with_advancing_header_count"])
        self.assertEqual(0, report["content_change_count"])

    def test_varying_scan_is_detected(self):
        observation = ScanContentObservation()
        observation.observe(scan([1.0, 2.0], 100.0))
        observation.observe(scan([1.0, 3.0], 100.1))
        report = observation.report()
        self.assertFalse(report["all_successful_payloads_identical"])
        self.assertEqual(1, report["content_change_count"])
        self.assertEqual(2, report["unique_payload_fingerprints_lower_bound"])

    def test_no_data_and_analysis_error_are_honest(self):
        core = SensorContentProbe({"scan": "/scan", "imu": "/imu", "depth": "/depth", "rgb": "/rgb"})
        report = core.report()
        self.assertEqual("no_data", report["observations"]["scan"]["status"])
        self.assertEqual("no_data", report["observations"]["depth"]["status"])
        self.assertFalse(report["all_hardware_accepted"])
        core.observe_image("depth", image("16UC1", 2, 2, 4, b"\x00" * 7))
        self.assertEqual("error", core.report()["observations"]["depth"]["status"])

    def test_image_summary_retains_its_own_timestamp(self):
        core = SensorContentProbe({"depth": "/depth"})
        message = image("16UC1", 1, 1, 2, b"\x01\x00")
        core.observe_image("depth", message)
        message.header = header(200.0)
        core.observe_image("depth", message)
        summary = core.report()["observations"]["depth"]
        self.assertEqual(100.0, summary["analyzed_header_stamp"])
        self.assertEqual(200.0, summary["latest_header_stamp"])

    def test_tf_query_uses_recorded_common_time(self):
        class Listener(object):
            def getLatestCommonTime(self, target, source):
                return Value(to_sec=lambda: 123.0)

            def lookupTransform(self, target, source, stamp):
                if source == "missing":
                    raise ValueError("missing transform")
                if target != "base_footprint" or stamp.to_sec() != 123.0:
                    raise ValueError("incorrect query")
                return [0, 0, 0], [0, 0, 0, 1]

        result = lookup_transforms(Listener(), None, {"scan": "scan", "imu": "missing"})
        self.assertEqual("observation_ok", result["scan"]["status"])
        self.assertEqual(123.0, result["scan"]["latest_common_timestamp"])
        self.assertEqual("error", result["imu"]["status"])
        self.assertEqual("no_data", result["depth"]["status"])

    def test_nonfinite_header_is_unavailable(self):
        message = scan([1.0])
        message.header.stamp = Value(to_sec=lambda: float("nan"))
        observation = ScanContentObservation()
        observation.observe(message)
        self.assertIsNone(observation.report()["latest_header_stamp"])
        json.dumps(observation.report(), allow_nan=False)

    def test_nonfinite_transform_is_error(self):
        class Listener(object):
            def getLatestCommonTime(self, target, source):
                return Value(to_sec=lambda: 123.0)

            def lookupTransform(self, target, source, stamp):
                return [float("nan"), 0, 0], [0, 0, 0, 1]

        result = lookup_transforms(Listener(), None, {"scan": "scan"})
        self.assertEqual("error", result["scan"]["status"])
        json.dumps(result, allow_nan=False)

    def test_serialization_failure_writes_valid_error_report(self):
        original = probe_sensor_content.run_ros
        directory = tempfile.mkdtemp()
        path = os.path.join(directory, "report.json")
        try:
            probe_sensor_content.run_ros = lambda unused: {"unexpected": float("nan")}
            self.assertEqual(2, probe_sensor_content.main(["--output", path]))
            with open(path) as handle:
                report = json.load(handle)
            self.assertIn("collector_error", report)
            self.assertFalse(report["all_hardware_accepted"])
        finally:
            probe_sensor_content.run_ros = original
            os.unlink(path)
            os.rmdir(directory)


if __name__ == "__main__":
    unittest.main()

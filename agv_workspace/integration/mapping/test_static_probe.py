import importlib.util
from pathlib import Path
import unittest


MODULE_PATH = Path(__file__).parent / "static_probe.py"
SPEC = importlib.util.spec_from_file_location("static_probe", MODULE_PATH)
static_probe = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(static_probe)


def sample(received, stamp, **extra):
    return {"received_monotonic": received, "stamp_ns": stamp, **extra}


def valid_data():
    increment = 3.3 / 1074
    geometry = {"angle_min": -1.65, "angle_max": 1.65, "angle_increment": increment,
                "time_increment": 0.00006, "scan_time": 0.066, "range_min": 0.05,
                "range_max": 40.0, "beam_count": 1075, "intensity_count": 1075}
    data = {}
    for name in static_probe.CONTINUOUS_STREAMS:
        if name == "scan":
            extra = {"geometry": geometry, "frame_id": "scan"}
        elif name == "imu":
            extra = {"angular_velocity_covariance": [0.0] * 8 + [0.01]}
        else:
            extra = {"pose_covariance": [0.01] + [0.0] * 35,
                     "twist_covariance": [0.04] + [0.0] * 34 + [0.04]}
        rate = {"scan": 15.0, "imu": 200.0, "wheel": 50.0, "filtered": 50.0}[name]
        count = int(rate) + 1
        data[name] = [sample(1.0 + index / rate, 1_000_000_000 + int(index * 1e9 / rate), **extra)
                      for index in range(count)]
    data["map"] = [sample(1.0, 1_000_000_000)]
    data["diagnostics"] = [sample(2.0, 2_000_000_000, status=[{
        "name": "feedbot_chassis", "level": 0, "message": "ok",
        "values": {"command_nonzero_count": "0"},
    }])]
    return data


def valid_transforms():
    result = []
    for (parent, child), translation in static_probe.EXPECTED_FIXED_EDGES.items():
        result.append({"parent": parent, "child": child, "translation": list(translation),
                       "rotation": [0.0, 0.0, 0.0, 1.0]})
    result.extend([
        {"parent": "map", "child": "odom", "translation": [0.0, 0.0, 0.0],
         "rotation": [0.0, 0.0, 0.0, 1.0]},
        {"parent": "odom", "child": "base_link", "translation": [0.0, 0.0, 0.0],
         "rotation": [0.0, 0.0, 0.0, 1.0]},
    ])
    return result


class StaticProbeEvaluationTest(unittest.TestCase):
    def initial_ekf_diagnostic(self, stamp=900000000, received=0.9):
        return sample(received, stamp, status=[{
            "name": "ekf_filter_node: odometry/filtered topic status", "level": 2,
            "message": "No events recorded.",
            "values": {"Events since startup": "0", "Events in window": "0"},
        }])

    def test_initial_ekf_diagnostic_is_retained_as_not_ready(self):
        data = valid_data()
        initial = self.initial_ekf_diagnostic()
        data["diagnostics"].insert(0, initial)
        result = static_probe.evaluate(data, 2.05, valid_transforms())
        self.assertTrue(result["passed"])
        self.assertEqual(result["startup_diagnostics"][0]["status"], initial["status"][0])
        self.assertEqual(result["samples"]["diagnostics"][0], initial)
        self.assertEqual(result["all_streams_observed_monotonic"], 1.0)
        self.assertAlmostEqual(result["runtime_observed_s"], 1.05)

    def test_late_delivery_uses_diagnostic_source_stamp(self):
        data = valid_data()
        data["diagnostics"].insert(0, self.initial_ekf_diagnostic(received=1.2))
        result = static_probe.evaluate(data, 2.05, valid_transforms())
        self.assertTrue(result["passed"])
        self.assertEqual(len(result["startup_diagnostics"]), 1)

    def test_zero_events_after_first_output_still_fails(self):
        data = valid_data()
        data["diagnostics"].append(self.initial_ekf_diagnostic(stamp=1500000000, received=1.5))
        result = static_probe.evaluate(data, 2.05, valid_transforms())
        self.assertIn("diagnostic ERROR or STALE reported", result["failures"])
        self.assertEqual(result["startup_diagnostics"], [])

    def test_no_output_cannot_be_accepted_as_initialization(self):
        data = valid_data()
        data["filtered"] = []
        data["diagnostics"].insert(0, self.initial_ekf_diagnostic())
        result = static_probe.evaluate(data, 2.05, valid_transforms())
        self.assertFalse(result["passed"])
        self.assertIn("diagnostic ERROR or STALE reported", result["failures"])
        self.assertEqual(result["runtime_observed_s"], 0.0)

    def test_other_startup_faults_are_not_exempted(self):
        for field, value in (("name", "feedbot_imu"), ("level", 3), ("message", "gyro timeout")):
            data = valid_data()
            initial = self.initial_ekf_diagnostic()
            initial["status"][0][field] = value
            data["diagnostics"].insert(0, initial)
            result = static_probe.evaluate(data, 2.05, valid_transforms())
            self.assertIn("diagnostic ERROR or STALE reported", result["failures"])

    def test_previous_events_or_missing_counter_cannot_be_initialization(self):
        for value in ("1", None):
            data = valid_data()
            initial = self.initial_ekf_diagnostic()
            initial["status"][0]["values"]["Events since startup"] = value
            data["diagnostics"].insert(0, initial)
            result = static_probe.evaluate(data, 2.05, valid_transforms())
            self.assertIn("diagnostic ERROR or STALE reported", result["failures"])

    def test_valid_window_passes(self):
        result = static_probe.evaluate(valid_data(), 2.05, valid_transforms())
        self.assertTrue(result["passed"])
        self.assertAlmostEqual(result["streams"]["scan"]["rate_hz"], 15.0)

    def test_missing_stale_and_non_monotonic_fail(self):
        data = valid_data()
        data["map"] = []
        data["imu"][1]["stamp_ns"] = data["imu"][0]["stamp_ns"]
        result = static_probe.evaluate(data, 2.35, valid_transforms())
        self.assertFalse(result["passed"])
        self.assertIn("missing topic data: /map", result["failures"])
        self.assertIn("non-monotonic stamps: imu", result["failures"])
        self.assertIn("stream stale at finish: scan", result["failures"])

    def test_diagnostic_error_and_invalid_scan_fail(self):
        data = valid_data()
        data["diagnostics"][0]["status"] = [{
            "name": "driver", "level": 2, "message": "fault",
            "values": {"command_nonzero_count": "0"},
        }]
        data["scan"][-1]["geometry"]["beam_count"] = 0
        result = static_probe.evaluate(data, 2.05, valid_transforms())
        self.assertIn("diagnostic ERROR or STALE reported", result["failures"])
        self.assertIn("invalid scan geometry", result["failures"])

    def test_wrong_fixed_transform_fails(self):
        transforms = valid_transforms()
        transforms[0]["translation"][2] = 0.1
        result = static_probe.evaluate(valid_data(), 2.05, transforms)
        self.assertIn("unexpected translation: base_link -> base_footprint", result["failures"])

    def test_fused_yaw_covariance_cannot_be_zero(self):
        data = valid_data()
        data["wheel"][-1]["twist_covariance"][35] = 0.0
        result = static_probe.evaluate(data, 2.05, valid_transforms())
        self.assertIn("invalid fused covariance: wheel.twist_covariance", result["failures"])

    def test_missing_zero_command_evidence_fails(self):
        data = valid_data()
        data["diagnostics"][0]["status"] = []
        result = static_probe.evaluate(data, 2.05, valid_transforms())
        self.assertIn("command_nonzero_count missing from diagnostics", result["failures"])

    def test_diagnostic_level_matches_jazzy(self):
        self.assertEqual(static_probe.diagnostic_level(bytes([2])), 2)
        self.assertEqual(static_probe.diagnostic_level(0), 0)


if __name__ == "__main__":
    unittest.main()

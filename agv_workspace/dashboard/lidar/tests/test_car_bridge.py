import importlib.util
import json
import math
from pathlib import Path
import types
import unittest


MODULE_PATH = Path(__file__).resolve().parents[1] / "car_bridge.py"
SPEC = importlib.util.spec_from_file_location("car_bridge", MODULE_PATH)
car_bridge = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(car_bridge)


class Clock:
    def __init__(self, value=10.0):
        self.value = value

    def __call__(self):
        return self.value


def scan_message(ranges=None):
    angle_min = -1.647495366
    angle_increment = 0.00306796166
    return types.SimpleNamespace(
        header=types.SimpleNamespace(
            seq=7,
            stamp=types.SimpleNamespace(secs=100, nsecs=250000000),
            frame_id="scan",
        ),
        angle_min=angle_min,
        angle_max=angle_min + 1074 * angle_increment,
        angle_increment=angle_increment,
        range_min=0.05,
        range_max=40.0,
        scan_time=1.0 / 15.0,
        time_increment=(1.0 / 15.0) / 2048.0,
        ranges=[1.0] * 1075 if ranges is None else ranges,
    )


class SnapshotStoreTest(unittest.TestCase):
    def test_linux_clock_gettime_monotonic_helper_advances(self):
        first = car_bridge.ctypes_monotonic_seconds()
        second = car_bridge.ctypes_monotonic_seconds()
        self.assertGreaterEqual(second, first)
        self.assertGreater(first, 0)

    def test_monotonic_source_freshness_removes_stale_ranges(self):
        clock = Clock()
        store = car_bridge.SnapshotStore(clock=clock, stale_after=0.5)
        self.assertTrue(store.update_scan(scan_message()))
        clock.value += 0.2
        fresh = store.snapshot()
        self.assertTrue(fresh["live"])
        self.assertAlmostEqual(0.2, fresh["source_age_s"])
        self.assertEqual(1075, len(fresh["scan"]["ranges"]))
        clock.value += 0.31
        stale = store.snapshot()
        self.assertFalse(stale["live"])
        self.assertEqual("stale", stale["status"])
        self.assertIsNone(stale["scan"])
        self.assertIsNone(stale["stats"])

    def test_nonfinite_ranges_are_json_null(self):
        ranges = [1.0] * 1075
        ranges[1] = float("inf")
        ranges[2] = float("nan")
        ranges[3] = -float("inf")
        store = car_bridge.SnapshotStore(clock=Clock())
        self.assertTrue(store.update_scan(scan_message(ranges)))
        snapshot = store.snapshot()
        self.assertEqual([None, None, None], snapshot["scan"]["ranges"][1:4])
        self.assertEqual(1, snapshot["stats"]["no_return_count"])
        self.assertEqual(2, snapshot["stats"]["invalid_range_count"])
        encoded = car_bridge.encode_json(snapshot)
        decoded = json.loads(encoded.decode("utf-8"))
        self.assertIsNone(decoded["scan"]["ranges"][1])
        self.assertNotIn("NaN", encoded.decode("utf-8"))
        self.assertNotIn("Infinity", encoded.decode("utf-8"))

    def test_bad_geometry_is_rejected_without_replacing_latest(self):
        store = car_bridge.SnapshotStore(clock=Clock())
        message = scan_message()
        message.angle_max += message.angle_increment
        self.assertFalse(store.update_scan(message))
        snapshot = store.snapshot()
        self.assertEqual(1, snapshot["rejected_scan_count"])
        self.assertFalse(snapshot["live"])

    def test_diagnostic_values_expire_to_unknown(self):
        clock = Clock()
        store = car_bridge.SnapshotStore(clock=clock, diagnostic_stale_after=1.0)
        value = lambda key, item: types.SimpleNamespace(key=key, value=item)
        entry = types.SimpleNamespace(
            name="ltr2_managed_data",
            hardware_id="LT-R2-B",
            level=1,
            message="active",
            values=[value("ready", "true"), value("healthy", "true"), value("navigation_accepted", "false")],
        )
        self.assertTrue(store.update_diagnostics(types.SimpleNamespace(status=[entry])))
        diagnostic = store.snapshot()["diagnostic"]
        self.assertTrue(diagnostic["ready"])
        self.assertFalse(diagnostic["navigation_accepted"])
        clock.value += 1.1
        diagnostic = store.snapshot()["diagnostic"]
        self.assertFalse(diagnostic["fresh"])
        self.assertIsNone(diagnostic["ready"])
        self.assertIsNone(diagnostic["navigation_accepted"])


if __name__ == "__main__":
    unittest.main()

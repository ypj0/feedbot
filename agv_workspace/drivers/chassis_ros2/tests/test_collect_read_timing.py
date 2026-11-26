import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest


COLLECTOR = (Path(__file__).parents[3] / "acceptance" / "runs" /
             "20261122_chassis_read_timing_01" / "collect_read_timing.py")
RUNNER = COLLECTOR.with_name("run_once.py")
SPEC = importlib.util.spec_from_file_location("collect_read_timing", COLLECTOR)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def value(key, contents):
    return SimpleNamespace(key=key, value=str(contents))


class CollectorTest(unittest.TestCase):
    def test_status_record_keeps_every_timing_and_frame_field(self):
        fields = [value(key, index) for index, key in enumerate(MODULE.TIMING_KEYS + MODULE.FRAME_KEYS)]
        status = SimpleNamespace(level=[0], message="ok", values=fields)
        record = MODULE.status_record(status, 12.5)
        self.assertEqual(set(record["timing"]), set(MODULE.TIMING_KEYS))
        self.assertEqual(set(record["last_observed_frame"]), set(MODULE.FRAME_KEYS))
        self.assertEqual(record["observed_monotonic_s"], 12.5)

    def test_summary_reports_activation_feedback_and_fault_intervals(self):
        fields = {
            "activation_started_monotonic_s": "10.0",
            "first_feedback_monotonic_s": "10.025",
            "last_receive_fault_monotonic_s": "10.225",
        }
        records = [{"values": fields}]
        summary = MODULE.build_summary(9.0, 11.0, "fault", records, 7)
        self.assertAlmostEqual(summary["activation_to_first_feedback_ms"], 25.0)
        self.assertAlmostEqual(summary["activation_to_fault_ms"], 225.0)
        self.assertAlmostEqual(summary["first_feedback_to_fault_ms"], 200.0)
        self.assertEqual(summary["stop_reason"], "fault")
        self.assertEqual(summary["odom_count"], 7)

    def test_collector_source_has_no_publish_or_command_topic(self):
        source = COLLECTOR.read_text()
        self.assertNotIn("create_publisher", source)
        self.assertNotIn("cmd_vel", source)
        self.assertIn('"/diagnostics"', source)
        self.assertIn('"/wheel/odom"', source)

    def test_one_shot_runner_has_no_motion_publisher_and_always_restores(self):
        source = RUNNER.read_text()
        self.assertNotIn("cmd_vel", source)
        self.assertNotIn("create_publisher", source)
        self.assertEqual(source.count('"ros2 launch feedbot_chassis_ros2 chassis.launch.py "'), 1)
        self.assertLess(source.index("collect_read_timing.py /evidence/timing.jsonl"),
                        source.index('"ros2 launch feedbot_chassis_ros2 chassis.launch.py "'))
        self.assertIn("finally:", source)
        self.assertIn("restore_factory()", source)
        self.assertIn("--duration 600", source)
        self.assertIn("wait -n -p finished_pid", source)


if __name__ == "__main__":
    unittest.main()

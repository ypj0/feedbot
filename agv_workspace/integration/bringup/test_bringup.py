import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


HERE = Path(__file__).resolve().parent


def load(path, name):
    specification = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


SESSION = load(HERE / "session.py", "tested_base_session")
PROBE = load(HERE / "probe.py", "tested_base_probe")
FIXTURES = load(HERE.parent / "mapping/test_static_probe.py", "base_probe_fixtures")


class BringupTests(unittest.TestCase):
    def test_shutdown_uses_ros_launch_normal_shutdown_signal(self):
        stack = (HERE / "stack.sh").read_text()
        shutdown = stack[stack.index("shutdown_stack() {"):stack.index("trap 'shutdown_stack 0'")]
        self.assertIn('kill -INT "$pid"', shutdown)
        self.assertNotIn('kill -TERM "$pid"', shutdown)
        self.assertIn('wait "${pids[$index]}"', shutdown)
        self.assertIn("trap 'shutdown_stack 0' TERM INT EXIT", stack)
        self.assertIn('env --default-signal=INT "$@" &', stack)

    def test_transient_units_release_names_after_failure(self):
        for launcher in (HERE / "run.sh", HERE.parent / "mapping/start.sh"):
            script = launcher.read_text()
            self.assertIn("systemd-run --collect --unit=", script)
            self.assertIn("--property=Restart=no", script)
            self.assertNotIn("reset-failed", script)

    def test_base_probe_needs_no_map_but_still_checks_data(self):
        data = FIXTURES.valid_data()
        del data["map"]
        transforms = [item for item in FIXTURES.valid_transforms() if item["parent"] != "map"]
        self.assertTrue(PROBE.evaluate(data, 2.05, transforms)["passed"])
        data["imu"] = []
        self.assertFalse(PROBE.evaluate(data, 2.05, transforms)["passed"])

    def test_mapping_probe_still_requires_map_and_map_tf(self):
        data = FIXTURES.valid_data()
        data["map"] = []
        transforms = [item for item in FIXTURES.valid_transforms() if item["parent"] != "map"]
        report = FIXTURES.static_probe.evaluate(data, 2.05, transforms)
        self.assertIn("missing topic data: /map", report["failures"])
        self.assertIn("missing transform: map -> odom", report["failures"])

    def test_base_command_has_its_own_container_and_audit_permissions(self):
        command = Mock()
        hooks = SESSION.Hooks(SimpleNamespace(command=command), None, Mock(), Path("/tmp/results"))
        hooks.native_start("test-token")
        arguments = command.call_args[0][1]
        self.assertIn("feedbot-base", arguments)
        self.assertIn("SYS_PTRACE", arguments)
        self.assertEqual(arguments[arguments.index("--ulimit") + 1], "rtprio=50")
        self.assertIn("nice=30:30", arguments)
        self.assertNotIn("SYS_NICE", arguments)
        self.assertIn("apparmor=feedbot-serial-audit", arguments)
        self.assertEqual(arguments[-2], "/work/agv_workspace/integration/bringup/stack.sh")
        self.assertNotIn("--privileged", arguments)
        self.assertNotIn("feedbot-mapping-static", arguments)
        self.assertNotIn("test-token", command.call_args[1]["logged_argv"])

    def test_base_stack_has_no_map_or_web_child(self):
        stack = (HERE / "stack.sh").read_text()
        for component in ("chassis", "imu", "ekf", "lidar"):
            self.assertEqual(stack.count("start_process " + component + " "), 1)
        self.assertNotIn("start_process frames ", stack)
        for forbidden in ("mapping.launch.py", "localization.launch.py", "car_bridge", "cmd_vel"):
            self.assertNotIn(forbidden, stack)
        legacy = (HERE.parent / "mapping/static_stack.sh").read_text()
        self.assertIn("bringup/stack.sh", legacy)
        self.assertNotIn("start_process chassis", legacy)

    def test_base_hold_failure_is_not_silently_restarted(self):
        process = Mock()
        process.communicate.return_value = ("7\n", "")
        process.returncode = 0
        subprocess = SimpleNamespace(Popen=Mock(return_value=process), PIPE=-1)
        hooks = SESSION.Hooks(SimpleNamespace(subprocess=subprocess), None, Mock(), Path("/tmp/results"))
        with self.assertRaisesRegex(RuntimeError, "base stack exited unexpectedly with 7"):
            hooks.hold(SESSION.LIFECYCLE.StopRequest())
        self.assertEqual(subprocess.Popen.call_count, 1)
        self.assertEqual(subprocess.Popen.call_args[0][0][-1], "feedbot-base")

    def test_restore_failure_cannot_report_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "transaction.json").write_text(json.dumps({"state": "native_active"}))
            supervisor = SimpleNamespace(root=root, start=Mock(), needs_restore=True,
                                         restore=Mock(side_effect=RuntimeError("restore failed")), close=Mock())
            hooks = SimpleNamespace(root=root, ready=Mock(), hold=Mock(),
                                    shutdown_lifecycle=Mock(return_value=[]), native_stop_error=None)
            store = Mock()
            stop = SimpleNamespace(requested=True, signum=15)
            status = SESSION.LIFECYCLE.execute_session(supervisor, hooks, store, stop, lambda: "now")
            report = store.write_json.call_args[0][1]
            self.assertEqual(status, 1)
            self.assertFalse(report["passed"])
            self.assertFalse(report["factory_restored"])
            supervisor.restore.assert_called_once()
            supervisor.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()

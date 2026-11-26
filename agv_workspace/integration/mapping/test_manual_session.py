import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).parent
SPEC = importlib.util.spec_from_file_location("manual_session", ROOT / "manual_session.py")
SESSION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SESSION)


class Store:
    def __init__(self, root):
        self.root = root

    def write_json(self, name, value):
        (self.root / name).write_text(json.dumps(value) + "\n")


class Supervisor:
    def __init__(self, events, root):
        self.events = events
        self.root = root
        self.needs_restore = False

    def start(self):
        self.events.append("start")
        self.needs_restore = True

    def restore(self):
        self.events.append("restore")
        self.needs_restore = False

    def close(self):
        self.events.append("close")


class Hooks:
    root = SESSION.ROOT

    def __init__(self, events, fail_hold=False):
        self.events = events
        self.fail_hold = fail_hold
        self.native_stop_error = None

    def ready(self):
        self.events.append("ready")

    def hold(self, stop_request):
        self.events.append("hold")
        if self.fail_hold:
            raise RuntimeError("manual stack exited unexpectedly with 7")
        stop_request.requested = True
        stop_request.signum = 15

    def shutdown_lifecycle(self):
        self.events.append("shutdown_lifecycle")
        return []


class ManualSessionTest(unittest.TestCase):
    def transaction(self, state="native_active"):
        directory = tempfile.TemporaryDirectory()
        root = Path(directory.name)
        (root / "transaction.json").write_text(json.dumps({"state": state}) + "\n")
        return directory, root

    def run_session(self, fail_hold=False):
        directory, root = self.transaction()
        previous = SESSION.OWNER_DIRECTORY
        SESSION.OWNER_DIRECTORY = root
        self.addCleanup(setattr, SESSION, "OWNER_DIRECTORY", previous)
        self.addCleanup(directory.cleanup)
        events = []
        stop = SESSION.StopRequest()
        code = SESSION.execute_session(Supervisor(events, root), Hooks(events, fail_hold),
                                       Store(root), stop, lambda: "now")
        return code, events, json.loads((root / "session.json").read_text())

    def test_intentional_sigterm_restores_factory(self):
        code, events, report = self.run_session()
        self.assertEqual(code, 0)
        self.assertEqual(events, ["start", "ready", "hold", "shutdown_lifecycle", "restore", "close"])
        self.assertTrue(report["passed"])
        self.assertTrue(report["factory_restored"])
        self.assertEqual(report["intentional_stop_signal"], 15)
        self.assertFalse(report["agent_motion_commands_sent"])
        self.assertIsNone(report["human_command_count"])

    def test_unexpected_container_exit_restores_promptly(self):
        code, events, report = self.run_session(fail_hold=True)
        self.assertEqual(code, 1)
        self.assertEqual(events, ["start", "ready", "hold", "shutdown_lifecycle", "restore", "close"])
        self.assertIn("manual stack exited unexpectedly", report["error"])
        self.assertTrue(report["factory_restored"])
        self.assertFalse(report["passed"])

    def test_manual_stack_contract(self):
        stack = (ROOT / "manual_stack.sh").read_text()
        source = (ROOT / "manual_session.py").read_text()
        self.assertIn("static_stack.sh", stack)
        self.assertIn("dashboard/mapping/car_bridge.py", stack)
        self.assertIn("source /opt/ros/jazzy/setup.bash", stack)
        self.assertIn("/bin/bash /work/agv_workspace/integration/mapping/static_stack.sh", stack)
        self.assertLess(stack.index("static_stack.sh"), stack.index("dashboard/mapping/car_bridge.py"))
        self.assertIn("index=${#pids[@]}-1", stack)
        self.assertIn('CONTAINER = STATIC.CONTAINER', source)
        self.assertIn('"/run/feedbot-chassis-web:/key:ro"', source)
        self.assertIn('"FEEDBOT_TEST_LOG=/results/manual_trace.jsonl"', source)
        self.assertIn("sessionready.json", source)
        self.assertIn('["wait", CONTAINER]', source)
        self.assertNotIn("time.sleep", source)

    def test_bridge_trace_counts_human_commands(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            records = [
                {"event": "telemetry", "value": {}},
                {"event": "command", "value": {"body_velocity": [0.0, 0.0]}},
                {"event": "command", "value": {"body_velocity": [0.1, 0.0]}},
            ]
            (root / "manual_trace.jsonl").write_text(
                "".join(json.dumps(record) + "\n" for record in records))
            self.assertEqual(SESSION.bridge_command_summary(root), {
                "human_command_count": 2, "human_nonzero_command_count": 1})


if __name__ == "__main__":
    unittest.main()

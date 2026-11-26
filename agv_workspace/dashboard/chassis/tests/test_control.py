import importlib.util
import io
import json
from pathlib import Path
import sys
import threading
import time
import unittest
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from control import PulseControl
from car_bridge import EventLog, Handler, Runtime

spec = importlib.util.spec_from_file_location("rk_server", ROOT / "rk_server.py")
rk_server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rk_server)


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.control = PulseControl()

    def request(self, action="forward", sequence=1, issued=10.0):
        return {"epoch": self.control.epoch, "sequence": sequence, "action": action, "issued_at": issued}

    def test_manual_pulse_and_expiry(self):
        self.assertEqual(self.control.velocity(10, True), (0, 0))
        self.control.accept(self.request(), 10, True)
        self.assertEqual(self.control.velocity(10.49, True), (0.05, 0))
        self.assertEqual(self.control.velocity(10.5, True), (0, 0))
        self.assertEqual(self.control.velocity(11, True), (0, 0))

    def test_no_queue_or_renewal(self):
        self.control.accept(self.request(), 10, True)
        with self.assertRaisesRegex(ValueError, "不会排队"):
            self.control.accept(self.request(sequence=2), 10.1, True)
        self.assertEqual(self.control.deadline, 10.5)

    def test_calibration_durations(self):
        for duration in (5, 10):
            self.control = PulseControl()
            request = dict(self.request(), duration_s=duration)
            self.control.accept(request, 10, True)
            self.assertEqual(self.control.snapshot(11)["pulse_s"], duration)
            self.assertEqual(self.control.snapshot(11)["remaining_s"], duration - 1)
            self.assertEqual(self.control.velocity(10 + duration - 0.01, True), (0.05, 0))
            self.assertEqual(self.control.velocity(10 + duration, True), (0, 0))

    def test_duration_boundary(self):
        for duration in (None, True, "10", 0, -1, 0.6, 11, float("nan"), float("inf"), []):
            with self.subTest(duration=duration), self.assertRaisesRegex(ValueError, "测试时长"):
                self.control.accept(dict(self.request(), duration_s=duration), 10, True)
            self.assertEqual(self.control.sequence, 0)
            self.assertEqual(self.control.action, "stop")

    def test_long_pulse_stop_fault_and_no_extension(self):
        self.control.accept(dict(self.request(), duration_s=10), 10, True)
        with self.assertRaisesRegex(ValueError, "不会排队"):
            self.control.accept(dict(self.request(sequence=2), duration_s=5), 10.1, True)
        self.assertEqual(self.control.deadline, 20)
        self.assertEqual(self.control.duration, 10)
        self.control.accept(dict(self.request("stop", 3), duration_s="invalid"), 11, False)
        self.assertEqual(self.control.velocity(12, True), (0, 0))
        self.control.accept(dict(self.request(sequence=4, issued=13), duration_s=10), 13, True)
        self.assertEqual(self.control.velocity(14, False), (0, 0))
        self.assertEqual(self.control.velocity(15, True), (0, 0))

    def test_stop_and_out_of_order(self):
        self.control.accept(self.request(), 10, True)
        self.control.accept(self.request("stop", sequence=3, issued=0), 10.1, False)
        self.assertEqual(self.control.velocity(10.2, True), (0, 0))
        with self.assertRaises(ValueError):
            self.control.accept(self.request(sequence=2), 10.2, True)

    def test_expired_session_and_telemetry(self):
        with self.assertRaises(ValueError):
            self.control.accept(self.request(), 11, True)
        with self.assertRaises(ValueError):
            self.control.accept(self.request(), 10, False)
        old = self.request()
        old["epoch"] = "old"
        with self.assertRaises(ValueError):
            self.control.accept(old, 10, True)
        self.control.accept(self.request(), 10, True)
        self.assertEqual(self.control.velocity(10.1, False), (0, 0))
        self.assertEqual(self.control.velocity(10.2, True), (0, 0))
        self.assertTrue(self.control.error)

    def test_all_directions(self):
        for action, expected in PulseControl.ACTIONS.items():
            self.control = PulseControl()
            self.control.accept(self.request(action), 10, True)
            self.assertEqual(self.control.velocity(10.1, True), expected)


class RuntimeTests(unittest.TestCase):
    def test_blocked_flush_does_not_block_feedback_or_stop(self):
        entered = threading.Event()
        release = threading.Event()
        completed = threading.Event()
        errors = []

        def held_flush():
            entered.set()
            if not release.wait(2):
                raise RuntimeError("test flush was not released")

        def feedback_and_stop():
            try:
                self.runtime.update("odom", {"vx": 0.0, "wz": 0.0})
                self.runtime.command({"epoch": self.runtime.control.epoch,
                                      "sequence": 1, "action": "stop"})
                completed.set()
            except BaseException as error:
                errors.append(error)

        with patch.object(self.log, "flush", side_effect=held_flush):
            tick = threading.Thread(target=self.runtime.tick)
            control = threading.Thread(target=feedback_and_stop)
            tick.start()
            try:
                self.assertTrue(entered.wait(1))
                control.start()
                self.assertTrue(completed.wait(0.2), "control waited for disk flush")
                self.assertEqual(self.commands[-1], (0.0, 0.0))
                self.assertFalse(errors)
            finally:
                release.set()
                tick.join(2)
                if control.ident is not None:
                    control.join(2)

    def setUp(self):
        self.commands = []
        self.log = io.StringIO()
        self.event_log = EventLog(self.log)
        self.addCleanup(self.event_log.close)
        self.runtime = Runtime(lambda *velocity: self.commands.append(velocity), self.event_log)
        self.runtime.update("joints", {"left_wheel_joint": 0.0, "right_wheel_joint": 0.0})
        self.runtime.update("odom", {"vx": 0.0, "wz": 0.0})
        self.values = {"initialized": "1", "emergency": "0", "charge": "0", "left_alarm": "0",
                       "right_alarm": "0", "shutdown": "0"}
        self.runtime.update("diagnostics", {"level": 0, "values": self.values})

    def test_ready_expiry_and_flags(self):
        self.assertTrue(self.runtime.snapshot()["ready"])
        self.values["charge"] = "1"
        self.assertFalse(self.runtime.snapshot()["ready"])
        self.values["charge"] = "0"
        self.runtime.samples["odom"]["received"] -= 1
        self.assertFalse(self.runtime.snapshot()["ready"])

    def test_published_request_and_fault_log(self):
        snapshot = self.runtime.snapshot()
        request = {"action": "forward", "sequence": 1, "epoch": snapshot["control"]["epoch"],
                   "issued_at": snapshot["sampled_at"]}
        self.runtime.command(request)
        self.assertEqual(self.commands[-1], (0.05, 0.0))
        self.runtime.samples["joints"]["received"] -= 1
        self.runtime.tick()
        self.assertEqual(self.commands[-1], (0.0, 0.0))
        self.assertEqual(self.runtime.control.action, "stop")
        self.event_log.close()
        events = [json.loads(line) for line in self.log.getvalue().splitlines()]
        self.assertTrue(any(event["event"] == "pulse_end" for event in events))


class HTTPTests(unittest.TestCase):
    def serve(self, handler):
        server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        server.token = "test-only-token"
        event_log = EventLog(io.StringIO())
        self.addCleanup(event_log.close)
        server.runtime = Runtime(lambda *_: None, event_log)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        return server

    def request(self, server, method, path, body=None, headers=None):
        connection = HTTPConnection("127.0.0.1", server.server_port, timeout=2)
        connection.request(method, path, body=body, headers=headers or {})
        response = connection.getresponse()
        result = response.status, response.read()
        connection.close()
        return result

    def test_car_auth_and_unknown_endpoint(self):
        server = self.serve(Handler)
        self.assertEqual(self.request(server, "GET", "/api/state")[0], 403)
        headers = {"X-Test-Token": server.token}
        self.assertEqual(self.request(server, "GET", "/api/state", headers=headers)[0], 200)
        self.assertEqual(self.request(server, "POST", "/api/command", "{}", headers)[0], 409)
        self.assertEqual(self.request(server, "GET", "/arbitrary", headers=headers)[0], 404)

    def test_rk_csrf_and_local_host(self):
        server = self.serve(rk_server.Handler)
        self.assertEqual(self.request(server, "GET", "/")[0], 403)
        headers = {"Host": "127.0.0.1:8088"}
        self.assertEqual(self.request(server, "GET", "/", headers=headers)[0], 200)
        self.assertEqual(self.request(server, "POST", "/api/command", "{}", headers)[0], 403)
        headers.update({"Origin": "http://evil.example", "X-Test-UI": "1"})
        self.assertEqual(self.request(server, "POST", "/api/command", "{}", headers)[0], 403)

    def test_rk_no_retry_on_upstream_error(self):
        server = self.serve(rk_server.Handler)
        with patch.object(rk_server, "urlopen", side_effect=rk_server.URLError("offline")) as opener:
            status, body = self.request(server, "GET", "/api/state", headers={"Host": "127.0.0.1:8088"})
            self.assertEqual(status, 502)
            self.assertIn("车机连接失败", json.loads(body)["error"])
            self.assertEqual(opener.call_count, 1)


if __name__ == "__main__":
    unittest.main()

import io
import json
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from car_bridge import Runtime
from event_log import EventLog


class HeldStream(io.StringIO):
    def __init__(self):
        super().__init__()
        self.entered = threading.Event()
        self.release = threading.Event()

    def flush(self):
        self.entered.set()
        if not self.release.wait(5):
            raise RuntimeError("test did not release stream")


class EventLogTests(unittest.TestCase):
    def test_order_and_drain_on_close(self):
        stream = io.StringIO()
        log = EventLog(stream)
        for index in range(20):
            log.submit(str(index) + "\n")
        log.close()
        self.assertEqual(stream.getvalue(), "".join(str(index) + "\n" for index in range(20)))
        with self.assertRaisesRegex(RuntimeError, "closed"):
            log.submit("late\n")

    def test_worker_failure_reaches_tick_and_does_not_prevent_stop(self):
        for operation in ("write", "flush"):
            with self.subTest(operation=operation):
                stream = io.StringIO()
                log = EventLog(stream)
                commands = []
                runtime = Runtime(lambda *value: commands.append(value), log)
                with patch.object(stream, operation, side_effect=OSError("disk failure")):
                    runtime.tick()
                    with log.condition:
                        self.assertTrue(log.condition.wait_for(lambda: log.error is not None, timeout=1))
                with self.assertRaisesRegex(RuntimeError, "disk failure"):
                    runtime.tick()
                with self.assertRaisesRegex(RuntimeError, "disk failure"):
                    runtime.command({"action": "forward"})
                self.assertEqual(runtime.control.sequence, 0)
                with self.assertRaisesRegex(RuntimeError, "disk failure"):
                    runtime.command({"action": "stop", "sequence": 1,
                                     "epoch": runtime.control.epoch})
                self.assertEqual(commands[-1], (0.0, 0.0))
                self.assertEqual(runtime.control.action, "stop")
                with self.assertRaisesRegex(RuntimeError, "disk failure"):
                    log.close()

    def test_backlog_reports_failure_without_blocking(self):
        stream = HeldStream()
        log = EventLog(stream)
        try:
            log.submit("first\n")
            self.assertTrue(stream.entered.wait(1))
            for index in range(128):
                log.submit(str(index) + "\n")
            with self.assertRaisesRegex(RuntimeError, "128 records"):
                log.submit("overflow\n")
            self.assertEqual(len(log.records), 128)
        finally:
            stream.release.set()
            with self.assertRaisesRegex(RuntimeError, "128 records"):
                log.close()

    def test_stalled_writer_exit_is_bounded_and_reported(self):
        stream = HeldStream()
        log = EventLog(stream)
        try:
            log.submit("first\n")
            self.assertTrue(stream.entered.wait(1))
            with self.assertRaisesRegex(RuntimeError, "did not stop within 2 seconds"):
                log.__exit__(None, None, None)
            self.assertFalse(stream.closed)
        finally:
            stream.release.set()
            log.close()
            stream.close()

    def test_full_backlog_rejects_motion_before_state_or_publish(self):
        stream = HeldStream()
        log = EventLog(stream)
        commands = []
        runtime = Runtime(lambda *value: commands.append(value), log)
        runtime.update("joints", {})
        runtime.update("odom", {})
        runtime.update("diagnostics", {"level": 0, "values": {
            "initialized": "1", "emergency": "0", "charge": "0", "left_alarm": "0",
            "right_alarm": "0", "shutdown": "0",
        }})
        try:
            log.submit("first\n")
            self.assertTrue(stream.entered.wait(1))
            for index in range(128):
                log.submit(str(index) + "\n")
            state = runtime.snapshot()
            with self.assertRaisesRegex(RuntimeError, "128 records"):
                runtime.command({"action": "forward", "sequence": 1,
                                 "epoch": runtime.control.epoch, "issued_at": state["sampled_at"]})
            self.assertEqual(runtime.control.sequence, 0)
            self.assertEqual(commands, [])
            with self.assertRaisesRegex(RuntimeError, "128 records"):
                runtime.command({"action": "stop", "sequence": 1,
                                 "epoch": runtime.control.epoch})
            self.assertEqual(commands, [(0.0, 0.0)])
        finally:
            stream.release.set()
            with self.assertRaisesRegex(RuntimeError, "128 records"):
                log.close()

    def test_event_payload_is_frozen_before_handoff(self):
        stream = HeldStream()
        log = EventLog(stream)
        runtime = Runtime(lambda *_: None, log)
        try:
            runtime.event("first", {})
            self.assertTrue(stream.entered.wait(1))
            payload = {"value": 1}
            runtime.event("second", payload)
            payload["value"] = 2
        finally:
            stream.release.set()
            log.close()
        events = [json.loads(line) for line in stream.getvalue().splitlines()]
        self.assertEqual(events[1]["value"], {"value": 1})


if __name__ == "__main__":
    unittest.main()

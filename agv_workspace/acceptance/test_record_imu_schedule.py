#!/usr/bin/env python3
import json
import os
from pathlib import Path
import shutil
import tempfile
import unittest

import record_imu_schedule as subject


class FakeEvent(object):
    def __init__(self, on_wait=None):
        self.on_wait = on_wait
        self.set_called = False
        self.wait_timeout = None

    def set(self):
        self.set_called = True

    def wait(self, timeout=None):
        self.wait_timeout = timeout
        if self.on_wait is not None:
            self.on_wait()
        return self.set_called


class Harness(object):
    def __init__(self, root, stop_event=None):
        self.root = Path(root)
        self.stop_event = stop_event or FakeEvent()
        self.trace_root = self.root / "tracing"
        (self.trace_root / "instances").mkdir(parents=True)
        self.global_marker = self.trace_root / "tracing_on"
        self.global_marker.write_text("GLOBAL")

    def create_instance(self, instance):
        for name in ("tracing_on", "trace_clock", "trace", "buffer_size_kb"):
            (instance / name).write_text("tail\n" if name == "trace" else "")
        for event in subject.EVENT_FILTERS:
            event_path = instance / "events" / event
            event_path.mkdir(parents=True)
            (event_path / "filter").write_text("")
            (event_path / "format").write_text("format for %s\n" % event)
            (event_path / "enable").write_text("0")
        stats = instance / "per_cpu" / "cpu0"
        stats.mkdir(parents=True)
        (stats / "stats").write_text(
            "overrun: 0\ncommit overrun: 0\ndropped events: 0\n")

    def recorder(self, output, created=None):
        recorder = subject.Recorder(
            output, trace_root=self.trace_root, monotonic=lambda: 12.5,
            instance_created=created or self.create_instance,
            instance_remover=shutil.rmtree, stop_event=self.stop_event)
        file_write = recorder._write

        def trace_write(path, value):
            file_write(path, "local [mono] global" if path.name == "trace_clock" else value)

        recorder._write = trace_write
        return recorder


class RecorderTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)

    def tearDown(self):
        self.temporary.cleanup()

    def load_manifest(self, output):
        return json.loads((output / "manifest.json").read_text())

    def test_exact_static_filters_and_bounded_finish(self):
        harness = Harness(self.root)
        output = self.root / "result"
        self.assertEqual(harness.recorder(output).run(), 0)
        configuration = json.loads((output / "event_configuration.json").read_text())
        self.assertEqual({key: value["filter"] for key, value in configuration.items()},
                         subject.EVENT_FILTERS)
        self.assertEqual(self.load_manifest(output)["status"], "finished")
        self.assertEqual(json.loads((output / "recorder_ready.json").read_text()),
                         {"monotonic_start": 12.5, "ready": True})
        self.assertEqual(harness.stop_event.wait_timeout, 60.0)
        self.assertEqual(json.loads((output / "buffer_configuration.json").read_text()),
                         {"readback_kb": "32768", "requested_kb": 32768})
        self.assertEqual((output / "trace.txt").read_text(), "tail\n")
        self.assertEqual(harness.global_marker.read_text(), "GLOBAL")
        self.assertFalse(any((harness.trace_root / "instances").iterdir()))

    def test_setup_failure_is_failed_and_does_not_touch_global_trace(self):
        harness = Harness(self.root)
        output = self.root / "failed"

        def incomplete_instance(instance):
            (instance / "tracing_on").write_text("")

        self.assertEqual(harness.recorder(output, incomplete_instance).run(), 1)
        self.assertIn("FileNotFoundError", self.load_manifest(output)["errors"][0])
        self.assertEqual(harness.global_marker.read_text(), "GLOBAL")

    def test_sigterm_request_stops_and_cleans_up(self):
        stop_event = FakeEvent()
        harness = Harness(self.root, stop_event)
        output = self.root / "signal"
        recorder = harness.recorder(output)
        stop_event.on_wait = recorder.request_stop
        self.assertEqual(recorder.run(), 0)
        self.assertTrue(stop_event.set_called)
        self.assertFalse(any((harness.trace_root / "instances").iterdir()))

    def test_stop_before_ready_does_not_claim_capture(self):
        harness = Harness(self.root)
        output = self.root / "not_started"
        recorder = harness.recorder(output)
        recorder.request_stop()
        self.assertEqual(recorder.run(), 1)
        self.assertFalse((output / "recorder_ready.json").exists())

    def test_filter_rejection_preserves_original_error_and_cleans_instance(self):
        harness = Harness(self.root)
        output = self.root / "bad_filter"
        recorder = harness.recorder(output)
        original_write = recorder._write

        def reject_filter(path, value):
            if path.name == "filter":
                raise OSError("kernel rejected filter")
            original_write(path, value)

        recorder._write = reject_filter
        self.assertEqual(recorder.run(), 1)
        self.assertIn("kernel rejected filter", self.load_manifest(output)["errors"][0])
        self.assertFalse(any((harness.trace_root / "instances").iterdir()))

    def test_signal_during_trace_enable_cannot_claim_ready_or_finished(self):
        harness = Harness(self.root)
        output = self.root / "enable_interrupted"
        recorder = harness.recorder(output)
        original_write = recorder._write

        def stop_at_enable(path, value):
            original_write(path, value)
            if path.name == "tracing_on" and value == "1":
                recorder.request_stop()

        recorder._write = stop_at_enable
        self.assertEqual(recorder.run(), 1)
        self.assertFalse((output / "recorder_ready.json").exists())
        self.assertFalse(any((harness.trace_root / "instances").iterdir()))

    def test_missing_stats_is_failure_not_empty_success(self):
        harness = Harness(self.root)
        output = self.root / "missing_stats"

        def without_stats(instance):
            harness.create_instance(instance)
            shutil.rmtree(instance / "per_cpu")

        self.assertEqual(harness.recorder(output, without_stats).run(), 1)
        self.assertEqual(self.load_manifest(output)["status"], "failed")

    def test_trace_loss_is_explicit_failure_with_evidence(self):
        harness = Harness(self.root)
        output = self.root / "loss"

        def with_loss(instance):
            harness.create_instance(instance)
            (instance / "per_cpu" / "cpu0" / "stats").write_text(
                "overrun: 1\ncommit overrun: 2\ndropped events: 3\n")

        self.assertEqual(harness.recorder(output, with_loss).run(), 1)
        self.assertIn("trace capture lost events", self.load_manifest(output)["errors"][0])
        self.assertTrue((output / "trace.txt").exists())
        self.assertTrue((output / "trace_stats.json").exists())

    def test_missing_loss_statistic_is_explicit_failure(self):
        harness = Harness(self.root)
        output = self.root / "incomplete_stats"

        def with_incomplete_stats(instance):
            harness.create_instance(instance)
            (instance / "per_cpu" / "cpu0" / "stats").write_text(
                "overrun: 0\ncommit overrun: 0\n")

        self.assertEqual(harness.recorder(output, with_incomplete_stats).run(), 1)
        self.assertIn("missing dropped_events statistic",
                      self.load_manifest(output)["errors"][0])

    def test_trace_is_read_only_after_tracing_is_stopped(self):
        harness = Harness(self.root)
        output = self.root / "ordered"
        recorder = harness.recorder(output)
        original_read = recorder._read

        def checked_read(path):
            if path.name == "trace":
                self.assertEqual((recorder.instance / "tracing_on").read_text(), "0")
            return original_read(path)

        recorder._read = checked_read
        self.assertEqual(recorder.run(), 0)

    def test_has_no_proc_or_live_userspace_drain_dependency(self):
        source = Path(subject.__file__).read_text()
        self.assertNotIn("/proc", source)
        self.assertNotIn("trace_pipe", source)
        self.assertNotIn("subprocess", source)
        self.assertNotIn("/bin/cat", source)

    def test_existing_output_is_never_overwritten(self):
        harness = Harness(self.root)
        output = self.root / "existing"
        output.mkdir()
        marker = output / "keep"
        marker.write_text("preserved")
        with self.assertRaises(FileExistsError):
            harness.recorder(output).run()
        self.assertEqual(marker.read_text(), "preserved")

    def test_existing_instance_is_not_removed(self):
        harness = Harness(self.root)
        instance = (harness.trace_root / "instances" /
                    (subject.INSTANCE_PREFIX + str(os.getpid())))
        instance.mkdir()
        marker = instance / "foreign"
        marker.write_text("preserved")
        output = self.root / "collision"
        self.assertEqual(harness.recorder(output).run(), 1)
        self.assertEqual(marker.read_text(), "preserved")
        self.assertEqual(self.load_manifest(output)["status"], "failed")


if __name__ == "__main__":
    unittest.main()

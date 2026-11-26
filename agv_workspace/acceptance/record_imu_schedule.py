#!/usr/bin/env python3
import json
import os
from pathlib import Path
import signal
import sys
import threading
import time


TRACE_ROOT = Path("/sys/kernel/debug/tracing")
INSTANCE_PREFIX = "feedbot_imu_schedule_"
RECORD_SECONDS = 60.0
BUFFER_SIZE_KB = 32768
EVENT_FILTERS = {
    "sched/sched_switch": 'prev_comm == "imu_node" || next_comm == "imu_node"',
    "sched/sched_wakeup": 'comm == "imu_node"',
    "sched/sched_wakeup_new": 'comm == "imu_node"',
    "sched/sched_process_exit": 'comm == "imu_node"',
    "sched/sched_process_exec":
        'filename == "/work/install/lib/feedbot_imu_ros2/imu_node"',
}


class Recorder(object):
    def __init__(self, output, trace_root=TRACE_ROOT, monotonic=time.monotonic,
                 instance_created=None, instance_remover=None, stop_event=None):
        self.output = Path(output).resolve()
        self.trace_root = Path(trace_root)
        self.monotonic = monotonic
        self.instance_created = instance_created
        self.instance_remover = instance_remover
        self.instance = None
        self.instance_owned = False
        self.stop_event = stop_event or threading.Event()
        self.stop_requested = False
        self.ready = False
        self.stopped_before_ready = False
        self.started = None
        self.ended = None

    def request_stop(self, unused_signum=None, unused_frame=None):
        self.stop_requested = True
        self.stop_event.set()
        if not self.ready:
            self.stopped_before_ready = True

    @staticmethod
    def _write(path, value):
        with path.open("w") as output:
            output.write(value)

    @staticmethod
    def _read(path):
        with path.open("r") as source:
            return source.read()

    def _save_json(self, name, value):
        with (self.output / name).open("w") as destination:
            json.dump(value, destination, indent=2, sort_keys=True)
            destination.write("\n")

    def _trace_stats(self):
        result = {}
        per_cpu = self.instance / "per_cpu"
        for cpu in sorted(per_cpu.iterdir(), key=lambda item: item.name):
            result[cpu.name] = self._read(cpu / "stats")
        if not result:
            raise RuntimeError("trace instance has no per-CPU statistics")
        return result

    def _configure(self):
        self._write(self.instance / "tracing_on", "0")
        self._write(self.instance / "buffer_size_kb", str(BUFFER_SIZE_KB))
        buffer_size = self._read(self.instance / "buffer_size_kb").strip()
        self._save_json("buffer_configuration.json",
                        {"requested_kb": BUFFER_SIZE_KB,
                         "readback_kb": buffer_size})
        if buffer_size != str(BUFFER_SIZE_KB):
            raise RuntimeError("trace buffer size readback mismatch: " + buffer_size)
        self._write(self.instance / "trace_clock", "mono")
        clock = self._read(self.instance / "trace_clock").strip()
        self._save_json("clock_configuration.json", {"readback": clock})
        if "[mono]" not in clock.split():
            raise RuntimeError("trace clock did not select mono: " + clock)
        actual = {}
        for event, event_filter in EVENT_FILTERS.items():
            event_path = self.instance / "events" / event
            self._write(event_path / "filter", event_filter)
            actual[event] = {
                "filter": self._read(event_path / "filter").strip(),
                "format": self._read(event_path / "format"),
            }
            self._write(event_path / "enable", "1")
        self._save_json("event_configuration.json", actual)

    @staticmethod
    def _lost_events(stats):
        lost = []
        for cpu, content in stats.items():
            values = {}
            for line in content.splitlines():
                if ":" in line:
                    name, value = line.split(":", 1)
                    values[name.strip().replace(" ", "_")] = value.strip()
            for name in ("overrun", "commit_overrun", "dropped_events"):
                if name not in values:
                    raise RuntimeError("missing %s statistic for %s" % (name, cpu))
                try:
                    count = int(values[name])
                except ValueError:
                    raise RuntimeError("invalid %s statistic for %s" % (name, cpu))
                if count > 0:
                    lost.append("%s %s=%d" % (cpu, name, count))
        return lost

    def _collect_trace_and_stats(self):
        with (self.output / "trace.txt").open("w") as destination:
            destination.write(self._read(self.instance / "trace"))
        stats = self._trace_stats()
        self._save_json("trace_stats.json", stats)
        lost = self._lost_events(stats)
        if lost:
            raise RuntimeError("trace capture lost events: " + ", ".join(lost))

    def run(self):
        self.output.mkdir(parents=True, exist_ok=False)
        manifest = {"status": "failed", "exit_code": 1,
                    "duration_limit_seconds": RECORD_SECONDS,
                    "trace_clock": "mono"}
        errors = []
        try:
            instances = self.trace_root / "instances"
            self.instance = instances / (INSTANCE_PREFIX + str(os.getpid()))
            self.instance.mkdir()
            self.instance_owned = True
            if self.instance_created is not None:
                self.instance_created(self.instance)
            self._configure()
            if self.stop_requested:
                raise RuntimeError("recording stopped before ready")
            self.started = self.monotonic()
            self._write(self.instance / "tracing_on", "1")
            self.ready = True
            if self.stopped_before_ready:
                raise RuntimeError("recording stopped before ready")
            self._save_json("recorder_ready.json",
                            {"monotonic_start": self.started, "ready": True})
            self.stop_event.wait(RECORD_SECONDS)
            self.ended = self.monotonic()
            if self.stopped_before_ready:
                raise RuntimeError("recording stopped before ready")
            self._write(self.instance / "tracing_on", "0")
            manifest["status"] = "finished"
            manifest["exit_code"] = 0
        except Exception as caught:
            errors.append("%s: %s" % (type(caught).__name__, caught))
        finally:
            if self.started is not None:
                if self.ended is None:
                    self.ended = self.monotonic()
                manifest["monotonic_start"] = self.started
                manifest["monotonic_end"] = self.ended
            if self.instance_owned and self.instance.exists():
                try:
                    self._write(self.instance / "tracing_on", "0")
                except Exception as caught:
                    errors.append("disable tracing: %s: %s" % (type(caught).__name__, caught))
                try:
                    self._collect_trace_and_stats()
                except Exception as caught:
                    errors.append("save trace: %s: %s" % (type(caught).__name__, caught))
                try:
                    if self.instance_remover is None:
                        self.instance.rmdir()
                    else:
                        self.instance_remover(self.instance)
                except OSError as caught:
                    errors.append("remove instance: %s: %s" % (type(caught).__name__, caught))
            if errors:
                manifest["status"] = "failed"
                manifest["exit_code"] = 1
                manifest["errors"] = errors
            self._save_json("manifest.json", manifest)
        return manifest["exit_code"]


def main(argv=None):
    if argv is None:
        argv = sys.argv
    if len(argv) != 2:
        raise SystemExit("usage: record_imu_schedule.py OUTPUT_DIRECTORY")
    recorder = Recorder(argv[1])
    signal.signal(signal.SIGTERM, recorder.request_stop)
    signal.signal(signal.SIGINT, recorder.request_stop)
    return recorder.run()


if __name__ == "__main__":
    raise SystemExit(main())

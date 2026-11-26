#!/usr/bin/env python3
import importlib.util
from pathlib import Path


SPEC = importlib.util.spec_from_file_location(
    "shared_base_probe", str(Path(__file__).resolve().parents[1] / "bringup/probe.py"))
PROBE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROBE)
CONTINUOUS_STREAMS = PROBE.CONTINUOUS_STREAMS
EXPECTED_FIXED_EDGES = PROBE.EXPECTED_FIXED_EDGES
REQUIRED_STREAMS = PROBE.REQUIRED_STREAMS + ("map",)
diagnostic_level = PROBE.diagnostic_level
read_system_time_parameter = PROBE.read_system_time_parameter


def evaluate(data, finished_monotonic, transforms=None, finished_wall_ns=None):
    return PROBE.evaluate(data, finished_monotonic, transforms, finished_wall_ns, require_map=True)


def main():
    return PROBE.main(require_map=True)


if __name__ == "__main__":
    raise SystemExit(main())

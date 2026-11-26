#!/usr/bin/env python3
"""Run disconnected tests and preserve command, source and result provenance."""

import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time


ROOT = Path(__file__).resolve().parents[2]


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def execute(name, command, output, timeout=180, cwd=ROOT):
    started = time.monotonic()
    record = {"name": name, "argv": [str(part) for part in command],
              "cwd": str(cwd), "timeout_seconds": timeout,
              "started_utc": datetime.datetime.now(datetime.timezone.utc).isoformat()}
    environment = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    log_path = output / (name + ".log")
    with log_path.open("wb") as log:
        try:
            process = subprocess.Popen(record["argv"], cwd=str(cwd), env=environment,
                                       stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            try:
                returncode = process.wait(timeout=timeout)
                record.update(returncode=returncode,
                              status="pass" if returncode == 0 else "fail")
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
                record.update(returncode=None, status="timeout")
        except OSError as error:
            record.update(returncode=None, status="error", error=str(error))
    record.update(elapsed_seconds=time.monotonic() - started,
                  log=log_path.name, log_sha256=digest(log_path))
    print("{}: {}".format(name, record["status"]), flush=True)
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--differential-python")
    arguments = parser.parse_args()
    output = Path(arguments.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    replacement = ROOT / "navigation_runtime/replacement"
    report = {"scope": "offline software checks only", "all_hardware_accepted": False,
              "checks": [], "source_sha256": {}, "limitations": [
                  "No physical device, production ROS integration or stopping validation.",
                  "A passing emulator comparison is not a firmware flashing qualification."]}
    source_roots = [replacement / name for name in ("chassis", "imu", "lidar")]
    source_roots.append(ROOT / "navigation_runtime/reverse")
    for directory in source_roots:
        for source in sorted(directory.rglob("*")):
            if source.is_file() and (source.suffix in (".py", ".hpp", ".h", ".cpp", ".c", ".sh") or source.name == "CMakeLists.txt"):
                report["source_sha256"][str(source.relative_to(ROOT))] = digest(source)
    for name in ("校验资料包.py", "文件清单.json", "SHA256SUMS.txt"):
        report["source_sha256"][name] = digest(ROOT / name)
    for source in sorted(Path(__file__).parent.glob("*.py")):
        report["source_sha256"][str(source.relative_to(ROOT))] = digest(source)
    fixture = ROOT / "analysis/body_navigation_20261107/lidar_native/pair_1794045504/raw.bin"
    report["source_sha256"][str(fixture.relative_to(ROOT))] = digest(fixture)
    report["source_sha256"][str(Path(__file__).resolve().relative_to(ROOT))] = digest(Path(__file__).resolve())

    def check(name, command, timeout=180, cwd=ROOT):
        record = execute(name, command, output, timeout, cwd)
        report["checks"].append(record)
        (output / "results.json").write_text(json.dumps(report, indent=2) + "\n")
        return record["status"] == "pass"

    check("archive_integrity", [sys.executable, ROOT / "校验资料包.py"], 300)
    check("acceptance_tools", [sys.executable, "-B", "-m", "unittest", "discover",
                               "-s", ROOT / "agv_workspace/acceptance", "-p", "test_*.py", "-v"])
    check("chassis_disconnected", [sys.executable, "-B", replacement / "chassis/run_tests.py"])
    check("lidar_supervisor", [sys.executable, "-B", replacement / "lidar/test_supervisor.py"])
    check("chassis_restoration", [sys.executable, "-B", replacement / "chassis/test_restoration.py"])
    with tempfile.TemporaryDirectory(prefix="agv-offline-build-") as temporary:
        build = Path(temporary)
        for name, source, parameters in (
                ("imu", replacement / "imu/test.cpp", []),
                ("lidar_lifecycle", replacement / "lidar/lifecycle_test.cpp", []),
                ("lidar_packet", replacement / "lidar/test.cpp", [fixture])):
            binary = build / name
            command = ["clang++", "-std=c++17", "-O1", "-g", "-Wall", "-Wextra",
                       "-fsanitize=address,undefined", "-pthread", source, "-o", binary]
            if check(name + "_build", command):
                check(name + "_test", [binary] + parameters)
        model_build = build / "models"
        if check("models_configure", ["cmake", "-S", ROOT / "navigation_runtime/reverse/recovered",
                                      "-B", model_build]):
            check("models_build", ["cmake", "--build", model_build, "-j", "2"])
    if arguments.differential_python:
        work = output / "differential_work"
        for relative in ("navigation_runtime/reverse", "analysis/reverse_artifacts", "analysis/chassis_stack"):
            shutil.copytree(ROOT / relative, work / relative)
        interpreter = Path(arguments.differential_python).expanduser().absolute()
        check("differential_dependencies", [interpreter, "-c",
              "import sys, unicorn, elftools; print(sys.executable); print(unicorn.__version__); print(elftools.__version__)"])
        check("binary_differential", ["sh", work / "navigation_runtime/reverse/verify_all.sh",
                                      interpreter], 600, work)
    else:
        report["checks"].append({"name": "binary_differential", "status": "not_run",
                                  "reason": "Supply a Python with Unicorn and pyelftools."})
    report["all_selected_checks_passed"] = all(record["status"] == "pass" for record in report["checks"])
    (output / "results.json").write_text(json.dumps(report, indent=2) + "\n")
    return 0 if report["all_selected_checks_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())

import json
import os
from pathlib import Path
import pty
import re
import signal
import struct
import subprocess
import sys
import threading
import time
import tty

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def packet(kind, raw=0, valid=True):
    frame = bytearray([0x55, kind]) + bytearray(struct.pack("<hhhh", 0, 0, raw, 0))
    frame.append(sum(frame) & 255)
    if not valid:
        frame[-1] ^= 0x01
    return frame


def owners(device):
    device_number = os.stat(device).st_rdev
    found = set()
    for process in Path("/proc").iterdir():
        if not process.name.isdigit():
            continue
        try:
            descriptors = (process / "fd").iterdir()
        except OSError:
            continue
        for descriptor in descriptors:
            try:
                if os.stat(descriptor).st_rdev == device_number:
                    found.add(int(process.name))
            except OSError:
                pass
    return sorted(found)


def values(status):
    return {item.key: item.value for item in status.values}


def level(status):
    return status.level[0] if isinstance(status.level, bytes) else int(status.level)


def main():
    result_directory = Path(sys.argv[1])
    imu_node = sys.argv[2]
    result_directory.mkdir(parents=True, exist_ok=False)
    assert Path("/.dockerenv").exists()
    assert not os.path.lexists("/dev/rikiimu")

    master, slave = pty.openpty()
    os.set_blocking(master, False)
    tty.setraw(slave)
    os.symlink(os.ttyname(slave), "/dev/rikiimu")
    os.close(slave)
    rclpy.init()
    observer = rclpy.create_node("imu_observation_fixture")
    messages = []
    diagnostics = []

    def receive_diagnostics(message):
        diagnostics.append({"message": message, "received_monotonic": time.monotonic()})

    subscriptions = [
        observer.create_subscription(Imu, "/imu", messages.append, qos_profile_sensor_data),
        observer.create_subscription(DiagnosticArray, "/diagnostics", receive_diagnostics, 100),
    ]
    stop = threading.Event()
    inject_bad = threading.Event()
    feed_records = []
    feed_errors = []
    node_log = (result_directory / "imu_node.log").open("w")
    process = None
    outcome = {"status": "FAILED", "hardware": "isolated container PTY only", "checks": []}

    def feed():
        try:
            while not stop.wait(0.01):
                if inject_bad.is_set():
                    os.write(master, packet(0x52, 37, False))
                    inject_bad.clear()
                    feed_records.append({"event": "bad_checksum", "monotonic": time.monotonic()})
                for kind in (0x51, 0x52, 0x53):
                    os.write(master, packet(kind, 94 if kind == 0x52 else 0))
                feed_records.append({"event": "valid_triplet", "monotonic": time.monotonic()})
        except Exception as error:
            feed_errors.append(str(error))

    feeder = threading.Thread(target=feed)
    feeder.start()

    def spin():
        rclpy.spin_once(observer, timeout_sec=0.002)
        assert not feed_errors, feed_errors

    def wait_for(predicate, label, duration=10.0):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            spin()
            if predicate():
                return
        raise AssertionError(label)

    def imu_statuses():
        return [status for record in diagnostics for status in record["message"].status
                if status.name == "feedbot_imu/WIT"]

    def imu_status_records():
        return [(record, status) for record in diagnostics for status in record["message"].status
                if status.name == "feedbot_imu/WIT"]

    try:
        time.sleep(0.2)
        process = subprocess.Popen([imu_node], stdout=node_log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        wait_for(lambda: len(messages) >= 20 and len(owners("/dev/rikiimu")) == 1 and imu_statuses(),
                 "IMU node did not establish data flow and exclusive ownership")
        initial_count = len(messages)
        rejection_baseline = int(values(imu_statuses()[-1]).get("decoder_rejected_bytes", "0"))
        inject_bad.set()
        wait_for(lambda: any(int(values(status).get("decoder_rejected_bytes", "0")) > rejection_baseline
                             for status in imu_statuses()), "checksum rejection diagnostic missing")
        rejected_status = next(status for status in imu_statuses()
                               if int(values(status).get("decoder_rejected_bytes", "0")) > rejection_baseline)
        count_after_reject = len(messages)
        wait_for(lambda: len(imu_statuses()) >= 3 and len(messages) > count_after_reject + 20,
                 "three diagnostics and continued valid gyro flow missing", 8.0)
        normal_records = [(record, status) for record, status in imu_status_records() if level(status) == 0]
        assert len(normal_records) >= 3
        selected_records = normal_records[:3]
        diagnostic_intervals = [after[0]["received_monotonic"] - before[0]["received_monotonic"]
                                for before, after in zip(selected_records, selected_records[1:])]
        assert all(0.8 <= interval <= 1.2 for interval in diagnostic_intervals)
        timing_keys = ("callback_gap_mean_s", "callback_gap_max_s", "read_mean_s", "read_max_s",
                       "parse_excluding_publish_mean_s", "parse_excluding_publish_max_s",
                       "publish_handoff_mean_s", "publish_handoff_max_s", "gyro_read_return_batch_gap_mean_s",
                       "gyro_read_return_batch_gap_max_s")
        histogram_keys = ("callback_gap_histogram_sample_count", "callback_gap_histogram_counts")
        samples = []
        for record, status in selected_records:
            metric = values(status)
            assert all(key in metric for key in timing_keys)
            assert all(key in metric for key in histogram_keys)
            histogram_counts = [int(count) for count in metric["callback_gap_histogram_counts"].split(",")]
            assert len(histogram_counts) == 107
            assert all(count >= 0 for count in histogram_counts)
            assert sum(histogram_counts) == int(metric["callback_gap_histogram_sample_count"])
            assert sum(histogram_counts) > 0
            assert 0.0 < float(metric["callback_gap_mean_s"]) <= float(metric["callback_gap_max_s"])
            assert 0.0 <= float(metric["read_mean_s"]) <= float(metric["read_max_s"])
            assert 0.0 <= float(metric["parse_excluding_publish_mean_s"]) <= float(
                metric["parse_excluding_publish_max_s"])
            assert 0.0 <= float(metric["publish_handoff_mean_s"]) <= float(
                metric["publish_handoff_max_s"])
            assert 0.0 < float(metric["gyro_read_return_batch_gap_mean_s"]) <= float(
                metric["gyro_read_return_batch_gap_max_s"])
            samples.append({"received_monotonic": record["received_monotonic"],
                            "header_stamp_ns": record["message"].header.stamp.sec * 10**9 +
                            record["message"].header.stamp.nanosec,
                            "values": {key: metric[key] for key in timing_keys + histogram_keys}})
        rejection_after = int(values(rejected_status)["decoder_rejected_bytes"])
        assert rejection_after > rejection_baseline
        assert len(messages) > count_after_reject > initial_count
        assert len(feed_records) > 128
        outcome["checks"].extend([
            "three one-second diagnostics with bounded timing and complete callback-gap histograms",
            "bad checksum increments rejection count",
            "valid gyro publishing continues after rejection",
            "single PTY serial owner",
        ])

        stop.set()
        feeder.join(timeout=2)
        stopped_at = time.monotonic()
        wait_for(lambda: process.poll() is not None, "200 ms gyro timeout did not exit", 3.0)
        exited_at = time.monotonic()
        assert process.returncode == 1
        assert exited_at - stopped_at >= 0.18
        wait_for(lambda: owners("/dev/rikiimu") == [], "serial owner remained after timeout")
        node_log.flush()
        log_text = (result_directory / "imu_node.log").read_text()
        journal = [line for line in log_text.splitlines() if "IMU batch callback_enter=" in line]
        assert "IMU gyro timeout (200 ms)" in log_text
        assert "IMU completed callback journal: 128 recent callback batches" in log_text
        assert len(journal) == 128
        callback_times = [float(re.search(r"callback_enter=([0-9.]+)", line).group(1)) for line in journal]
        assert all(after > before for before, after in zip(callback_times, callback_times[1:]))
        assert "rejected=" in journal[-1] and "age=" in journal[-1]
        assert float(re.search(r"age=([0-9.]+)", journal[-1]).group(1)) >= 0.2
        assert int(re.search(r"rejected=([0-9]+)", journal[-1]).group(1)) > 0
        outcome["checks"].extend([
            "unchanged 200 ms timeout exits once without restart",
            "128-entry journal is chronological and includes timeout batch",
            "journal preserves cumulative rejected-byte evidence",
            "serial closes before process exit",
        ])
        outcome.update(status="PASS", imu_messages=len(messages), diagnostics=len(imu_statuses()),
                       timeout_elapsed_s=exited_at - stopped_at, timing_samples=samples,
                       diagnostic_intervals_s=diagnostic_intervals,
                       rejection_baseline=rejection_baseline, rejection_after=rejection_after,
                       journal_entries=len(journal), final_journal=journal[-1])
    finally:
        stop.set()
        feeder.join(timeout=2)
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=10)
        node_log.close()
        outcome["remaining_owners"] = owners("/dev/rikiimu")
        (result_directory / "result.json").write_text(json.dumps(outcome, indent=2) + "\n")
        (result_directory / "feed.json").write_text(json.dumps(feed_records, indent=2) + "\n")
        os.close(master)
        Path("/dev/rikiimu").unlink()
        observer.destroy_node()
        rclpy.shutdown()
    print(json.dumps(outcome, indent=2))


if __name__ == "__main__":
    main()

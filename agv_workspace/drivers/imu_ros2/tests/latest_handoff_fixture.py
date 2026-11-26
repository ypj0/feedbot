import json
import math
import os
from pathlib import Path
import pty
import signal
import subprocess
import sys
import threading
import time
import tty

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from observation_fixture import owners, packet, values


def main():
    directory = Path(sys.argv[1])
    binary, mode = sys.argv[2:4]
    assert mode in ("paced", "burst", "slow_publish")
    assert Path("/.dockerenv").exists() and not os.path.lexists("/dev/rikiimu")
    directory.mkdir(parents=True, exist_ok=False)
    master, slave = pty.openpty()
    tty.setraw(slave)
    os.symlink(os.ttyname(slave), "/dev/rikiimu")
    os.close(slave)
    rclpy.init()
    observer = rclpy.create_node("latest_handoff_fixture")
    messages, diagnostics, sent, errors = [], [], [], []
    scale = 2000 * math.pi / 180 / 32768
    subscriptions = [
        observer.create_subscription(Imu, "/imu", lambda message: messages.append({
            "sequence": round(message.angular_velocity.z / scale),
            "stamp_ns": message.header.stamp.sec * 10**9 + message.header.stamp.nanosec,
            "receipt_monotonic": time.monotonic(),
        }), QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)),
        observer.create_subscription(DiagnosticArray, "/diagnostics", lambda message: diagnostics.extend(
            values(status) for status in message.status if status.name == "feedbot_imu/WIT"), 100),
    ]
    stop = threading.Event()
    group = 10 if mode == "burst" else 1

    def feed():
        sequence = 0
        deadline = time.monotonic()
        try:
            while not stop.is_set():
                payload = b"".join(packet(kind, sequence + offset if kind == 0x52 else 0)
                                   for offset in range(group) for kind in (0x51, 0x52, 0x53))
                assert os.write(master, payload) == len(payload)
                sent.append(sequence + group - 1)
                sequence += group
                deadline += .005 * group
                stop.wait(max(0, deadline - time.monotonic()))
        except BaseException as error:
            errors.append(str(error))

    feeder = threading.Thread(target=feed)
    result = {"status": "FAILED", "mode": mode, "hardware": "isolated PTY only"}
    process = None
    with (directory / "imu.log").open("w") as output:
        try:
            environment = dict(os.environ)
            if mode == "slow_publish":
                environment["LD_PRELOAD"] = sys.argv[4]
            process = subprocess.Popen([binary], env=environment, stdout=output, stderr=subprocess.STDOUT)
            feeder.start()
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline and process.poll() is None:
                rclpy.spin_once(observer, timeout_sec=.002)
            assert process.poll() is None and not errors, errors
            assert len(diagnostics) >= 3
            stop.set()
            feeder.join()
            deadline = time.monotonic() + .12
            while time.monotonic() < deadline:
                rclpy.spin_once(observer, timeout_sec=.002)
            assert messages[-1]["sequence"] == sent[-1], "newest pending sample was not delivered"
            assert all(after["sequence"] > before["sequence"] and after["stamp_ns"] > before["stamp_ns"]
                       for before, after in zip(messages, messages[1:])), "repeated or reordered sample"
            for fields in diagnostics:
                assert fields["publish_policy"] == "latest_pending"
                assert "publish_handoff_drop_count" not in fields
                assert int(fields["received_gyro_count"]) == int(fields["publish_handoff_count"])
                assert int(fields["publish_handoff_count"]) == sum(int(fields[key]) for key in (
                    "publish_pending_overwrite_count", "publish_completed_count", "publish_pending",
                    "publish_in_flight", "publish_discarded_on_stop_count", "publish_failure_count"))
            if mode == "slow_publish":
                assert int(diagnostics[-1]["publish_pending_overwrite_count"]) > 0
                assert int(diagnostics[-1]["received_gyro_count"]) > 5 * int(diagnostics[-1]["publish_completed_count"])
            process.send_signal(signal.SIGTERM)
            assert process.wait(timeout=5) == 0
            assert owners("/dev/rikiimu") == []
            result.update(status="PASS", last_sent=sent[-1], last_received=messages[-1]["sequence"])
        finally:
            stop.set()
            if feeder.ident is not None:
                feeder.join()
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGTERM)
                process.wait(timeout=5)
            result.update(received=messages, diagnostics=diagnostics, errors=errors,
                          exit_code=None if process is None else process.returncode,
                          remaining_owners=owners("/dev/rikiimu"))
            (directory / "result.json").write_text(json.dumps(result, indent=2) + "\n")
            os.close(master)
            os.unlink("/dev/rikiimu")
            observer.destroy_node()
            rclpy.shutdown()
    print(json.dumps({key: value for key, value in result.items() if key not in ("received", "diagnostics")}))


if __name__ == "__main__":
    main()

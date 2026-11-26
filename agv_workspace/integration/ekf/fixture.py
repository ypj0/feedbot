import json
import math
import os
from pathlib import Path
import pty
import signal
import struct
import subprocess
import sys
import threading
import time
import tty

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

sys.path.insert(0, "/work/agv_workspace/drivers/chassis_ros2/tests")
from controller_fixture import MCU


def main():
    results = Path(sys.argv[1])
    results.mkdir(parents=True, exist_ok=False)
    failure = sys.argv[2]
    assert failure in ("runtime", "timeout")
    assert not os.path.lexists("/dev/rikiimu")
    mcu = MCU()
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    tty.setraw(slave)
    imu_path = os.ttyname(slave)
    os.symlink(imu_path, "/dev/rikiimu")
    os.close(slave)
    rclpy.init()
    node = rclpy.create_node("integration_fixture")
    samples = {"imu": [], "odom": [], "filtered": [], "diagnostics": [], "tf": []}
    subscriptions = [
        node.create_subscription(Imu, "/imu", samples["imu"].append, qos_profile_sensor_data),
        node.create_subscription(Odometry, "/wheel/odom", samples["odom"].append, 100),
        node.create_subscription(Odometry, "/odometry/filtered", samples["filtered"].append, 100),
        node.create_subscription(DiagnosticArray, "/diagnostics", samples["diagnostics"].append, 100),
    ]
    gyro_raw = 0
    imu_enabled = True
    feed_stop = threading.Event()
    feed_records = []
    feed_errors = []
    processes = []
    logs = []
    outcome = {"status": "FAILED", "hardware": "isolated synthetic PTY only", "fault": failure}

    def launch(name, arguments):
        log = (results / (name + ".log")).open("w")
        logs.append(log)
        process = subprocess.Popen(arguments, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    def feed_imu():
        try:
            while not feed_stop.is_set():
                if imu_enabled:
                    raw = gyro_raw
                    for kind in (0x51, 0x52, 0x53):
                        packet = bytearray([0x55, kind]) + bytearray(struct.pack("<hhhh", 0, 0, raw if kind == 0x52 else 0, 0))
                        packet.append(sum(packet) & 255)
                        os.write(master, packet)
                    feed_records.append({"monotonic": time.monotonic(), "raw": raw})
                feed_stop.wait(0.005)
        except Exception as error:
            feed_errors.append(str(error))

    feeder = threading.Thread(target=feed_imu)
    feeder.start()

    def spin():
        rclpy.spin_once(node, timeout_sec=0.001)
        assert mcu.error is None, mcu.error
        assert not feed_errors, feed_errors

    def wait_for(predicate, label, duration=15):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            spin()
            if predicate():
                return
        raise AssertionError(label)

    def collect(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            spin()

    def mcu_errors():
        return [status for message in samples["diagnostics"] for status in message.status
                if status.name == "feedbot_chassis/MCU" and status.level == b"\x02"]

    try:
        launch("tf_edges", ["/work/build/tf_probe/tf_edge_probe"])
        observer = launch("observer", ["python3", "-B", "/work/agv_workspace/integration/ekf/observe.py",
                                       str(results / "observation.jsonl"), "5"])
        launch("chassis", ["ros2", "launch", "feedbot_chassis_ros2", "chassis.launch.py",
                            "covariance_file:=/work/agv_workspace/integration/ekf/config/wheel_covariance.yaml"])
        imu = launch("imu", ["ros2", "run", "feedbot_imu_ros2", "imu_node"])
        launch("ekf", ["ros2", "launch", "/work/agv_workspace/integration/ekf/ekf.launch.py"])
        wait_for(lambda: len(samples["filtered"]) > 30 and len(samples["imu"]) > 30 and
                 len(samples["odom"]) > 30, "integration data missing")
        assert samples["odom"][-1].twist.covariance[0] == 0.04
        assert samples["odom"][-1].twist.covariance[35] == 0.04
        assert samples["imu"][-1].angular_velocity_covariance[8] == 0.01
        assert abs(samples["filtered"][-1].twist.twist.angular.z) < 1e-5
        gyro_raw = 94
        mcu.increment = (-45, 45)
        print(json.dumps({"phase": "positive_gyro", "monotonic": time.monotonic(), "raw": gyro_raw}), flush=True)
        collect(2)
        print(json.dumps({"phase": "positive_observation", "monotonic": time.monotonic(),
                          "raw": gyro_raw, "last_sent": feed_records[-1],
                          "last_received_wz": samples["imu"][-1].angular_velocity.z,
                          "imu_exit": imu.poll()}), flush=True)
        assert abs(samples["imu"][-1].angular_velocity.z - 94 / 32768 * 2000 * math.pi / 180) < 1e-10
        assert samples["filtered"][-1].twist.twist.angular.z > 0.05
        assert samples["filtered"][-1].pose.pose.orientation.z > 0
        assert all(command == (0, 0) for command in mcu.requests)
        edges = [json.loads(line) for line in (results / "tf_edges.log").read_text().splitlines() if line.startswith("{")]
        edges = [edge for edge in edges if edge["parent"] == "odom" and edge["child"] == "base_link"]
        endpoints = {bytes(info.endpoint_gid).hex(): info.node_name for info in node.get_publishers_info_by_topic("/tf")}
        assert edges and {endpoints[edge["gid"]] for edge in edges} == {"ekf_filter_node"}
        gyro_raw = 0
        mcu.increment = (0, 0)
        collect(0.5)
        os.write(mcu.master, b"\x11\x22\x33")
        wait_for(lambda: any(int(dict((item.key, item.value) for item in status.values).get(
            "sync_checksum_rejected_bytes", "0")) > 0 for message in samples["diagnostics"]
            for status in message.status if status.name == "feedbot_chassis/MCU"), "rejection counter missing")
        assert not mcu_errors()
        mcu.enabled = False
        if failure == "runtime":
            frame = bytearray(40)
            frame[:5] = bytes([127, 127, 40, 3, 0x60])
            frame[38:] = b"\r\n"
            frame[37] = -sum(frame[:37]) % 256
            os.write(mcu.master, frame)
        wait_for(lambda: mcu_errors() and mcu.owners() == [], "fault must report ERROR and close serial")
        error = mcu_errors()[0]
        values = {item.key: item.value for item in error.values}
        assert int(values["runtime_field_error_count"]) == (1 if failure == "runtime" else 0)
        assert int(values["feedback_timeout_count"]) == (1 if failure == "timeout" else 0)
        assert float(values["last_receive_fault_monotonic_s"]) > 0
        if failure == "runtime":
            assert "frame_hex=" + frame.hex() in error.message
        collect(0.4)
        odom_count = len(samples["odom"])
        mcu.enabled = True
        collect(0.5)
        assert len(samples["odom"]) == odom_count and mcu.handshakes == 1 and mcu.owners() == []
        imu_enabled = False
        wait_for(lambda: imu.poll() is not None, "IMU timeout must exit")
        assert imu.returncode == 1
        collect(0.3)
        imu_count = len(samples["imu"])
        collect(0.3)
        assert len(samples["imu"]) == imu_count
        stamps = [message.header.stamp.sec * 10**9 + message.header.stamp.nanosec for message in samples["imu"]]
        assert all(after > before for before, after in zip(stamps, stamps[1:]))
        wait_for(lambda: observer.poll() is not None, "read-only observer must complete")
        assert observer.returncode == 0
        observation = json.loads((results / "observation.summary.json").read_text())
        assert observation["counts"]["imu"] > 0 and observation["imu"]["strictly_monotonic"]
        outcome.update(status="PASS", imu_messages=len(stamps), odom_messages=odom_count,
                       tf_edge_endpoints=endpoints, diagnostic_values=values,
                       checks=["fresh gyro and units", "conservative covariance propagated", "synthetic yaw follows",
                               "EKF-only odom TF by GID", "zero MCU commands only", "rejection diagnostics",
                               "classified fault closes serial and stops odom", "no reconnect",
                               "IMU timeout exits without stale data"])
    finally:
        feed_stop.set()
        feeder.join(timeout=2)
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        for log in logs:
            log.close()
        outcome["remaining_mcu_owners"] = mcu.owners()
        (results / "result.json").write_text(json.dumps(outcome, indent=2) + "\n")
        (results / "imu_feed.json").write_text(json.dumps(feed_records) + "\n")
        mcu.close()
        os.close(master)
        Path("/dev/rikiimu").unlink()
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(outcome, indent=2))


if __name__ == "__main__":
    main()

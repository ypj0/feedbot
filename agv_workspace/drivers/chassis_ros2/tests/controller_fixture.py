import errno
import json
import os
from pathlib import Path
import pty
import select
import signal
import struct
import subprocess
import sys
import threading
import time
import traceback
import tty

import rclpy
from controller_manager_msgs.srv import SetHardwareComponentState, SwitchController
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from lifecycle_msgs.msg import State
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


class MCU:
    def __init__(self):
        if not Path("/.dockerenv").exists() or os.path.lexists("/dev/rikibase"):
            raise RuntimeError("Fixture requires a container WITHOUT /dev/rikibase mapping")
        self.master, slave = pty.openpty()
        tty.setraw(slave)
        self.slave_path = os.ttyname(slave)
        os.symlink(self.slave_path, "/dev/rikibase")
        os.close(slave)
        self.increment = (0, 0)
        self.flags = (0, 0, 0, 0)
        self.enabled = True
        self.alive = True
        self.handshakes = 0
        self.requests = []
        self.error = None
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
        buffer = bytearray()
        try:
            while self.alive:
                ready, _, _ = select.select([self.master], [], [], 0.02)
                if not ready:
                    continue
                try:
                    chunk = os.read(self.master, 4096)
                except OSError as error:
                    if error.errno != errno.EIO:
                        raise
                    time.sleep(0.005)
                    continue
                buffer.extend(chunk)
                while len(buffer) >= 40:
                    request = bytes(buffer[:40])
                    del buffer[:40]
                    assert request[:3] == b"\x7f\x7f\x28" and request[38:] == b"\r\n", request.hex()
                    assert sum(request[:38]) % 256 == 0, request.hex()
                    response = bytearray(40)
                    response[:5] = bytes([127, 127, 40, request[3], 0x60])
                    response[38:] = b"\r\n"
                    if request[3] == 1:
                        response[5:9] = struct.pack(">I", 115200)
                        self.handshakes += 1
                    else:
                        assert request[3] == 2
                        self.requests.append(struct.unpack(">hh", request[10:14]))
                        if not self.enabled:
                            continue
                        response[5], response[8], response[9], response[10] = self.flags
                        response[6] = 1
                        response[7] = 62
                        response[11:15] = struct.pack(">hh", *self.increment)
                        response[33] = 2
                        response[35:37] = struct.pack(">h", 441)
                    response[37] = -sum(response[:37]) % 256
                    os.write(self.master, response)
        except BaseException:
            self.error = traceback.format_exc()

    def owners(self):
        device = os.stat(self.slave_path).st_rdev
        owners = set()
        for process in Path("/proc").iterdir():
            if not process.name.isdigit():
                continue
            for descriptor in (process / "fd").glob("*"):
                try:
                    if descriptor.stat().st_rdev == device:
                        owners.add(int(process.name))
                except FileNotFoundError:
                    pass
        return sorted(owners)

    def close(self):
        self.alive = False
        self.thread.join(timeout=2)
        os.close(self.master)
        Path("/dev/rikibase").unlink()


def main():
    results = Path(sys.argv[1])
    results.mkdir(parents=True, exist_ok=False)
    covariance = results / "synthetic_covariance.yaml"
    covariance.write_text("wheel:\n  ros__parameters:\n"
                          "    twist_covariance_diagonal: [0.01, 0.02, 0.03, 0.04, 0.05, 0.06]\n"
                          "    pose_covariance_diagonal: [0.11, 0.12, 0.13, 0.14, 0.15, 0.16]\n")
    mcu = MCU()
    rclpy.init()
    node = rclpy.create_node("chassis_fixture_probe")
    samples = {"joints": [], "odom": [], "diagnostics": [], "odom_tf": []}

    def joints(message):
        samples["joints"].append(dict(zip(message.name, message.position)))

    def odometry(message):
        samples["odom"].append({"vx": message.twist.twist.linear.x,
                                "wz": message.twist.twist.angular.z,
                                "twist_covariance": list(message.twist.covariance),
                                "pose_covariance": list(message.pose.covariance)})

    def diagnostics(message):
        for status in message.status:
            if status.name == "feedbot_chassis/MCU":
                samples["diagnostics"].append({"level": status.level[0], "message": status.message,
                                               "values": {value.key: value.value for value in status.values}})

    def transforms(message):
        for transform in message.transforms:
            if transform.header.frame_id == "odom" and transform.child_frame_id == "base_link":
                samples["odom_tf"].append(str(transform))

    subscriptions = [
        node.create_subscription(JointState, "/joint_states", joints, 100),
        node.create_subscription(Odometry, "/wheel/odom", odometry, 100),
        node.create_subscription(DiagnosticArray, "/diagnostics", diagnostics, 100),
        node.create_subscription(TFMessage, "/tf", transforms, 100),
    ]
    commands = node.create_publisher(TwistStamped, "/wheel/cmd_vel", 10)
    outcome = {"status": "FAILED", "hardware": "synthetic PTY ONLY", "checks": []}

    def wait_for(predicate, label, timeout=12):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if mcu.error:
                raise AssertionError(mcu.error)
            if predicate():
                return
        raise AssertionError("timeout: " + label)

    def collect(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)

    def call(service_type, path, request):
        client = node.create_client(service_type, path)
        assert client.wait_for_service(timeout_sec=5), path
        future = client.call_async(request)
        wait_for(future.done, path)
        response = future.result()
        node.destroy_client(client)
        return response

    log = (results / "controller.log").open("w")
    process = subprocess.Popen(["ros2", "launch", "feedbot_chassis_ros2", "chassis.launch.py",
                                "covariance_file:=" + str(covariance)], stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True)
    try:
        wait_for(lambda: len(samples["odom"]) > 10 and len(samples["joints"]) > 10, "controller data")
        assert mcu.handshakes == 1 and len(mcu.owners()) == 1
        outcome["checks"].append("handshake and single serial owner")
        timing_keys = [
            "receive_total_last_us", "receive_total_max_us",
            "read_syscall_total_last_us", "read_syscall_total_max_us",
            "parse_validate_total_last_us", "parse_validate_total_max_us",
            "joint_update_last_us", "joint_update_max_us",
            "diagnostic_build_last_us", "diagnostic_build_max_us",
            "diagnostic_publish_last_us", "diagnostic_publish_max_us",
            "read_callback_entry_monotonic_s", "read_callback_finished_monotonic_s",
            "read_callback_last_us", "read_callback_last_complete_us",
            "read_callback_previous_complete_us", "read_callback_max_complete_us",
        ]
        wait_for(lambda: all(key in samples["diagnostics"][-1]["values"] for key in timing_keys),
                 "timing diagnostics")
        values = samples["diagnostics"][-1]["values"]
        for prefix in ("receive_total", "read_syscall_total", "parse_validate_total",
                       "joint_update", "diagnostic_build", "diagnostic_publish"):
            assert 0.0 <= float(values[prefix + "_last_us"]) <= float(values[prefix + "_max_us"])
        assert len(values["last_observed_frame_hex"]) == 80
        assert values["last_observed_frame_type"] == "2" and values["last_observed_frame_ack"] == "96"
        assert float(values["activation_started_monotonic_s"]) > 0.0
        assert float(values["first_read_return_monotonic_s"]) > 0.0
        assert float(values["first_feedback_monotonic_s"]) >= float(values["activation_started_monotonic_s"])
        assert (0.0 < float(values["read_callback_entry_monotonic_s"])
                <= float(values["read_callback_finished_monotonic_s"]))
        assert 0.0 <= float(values["read_callback_last_us"])
        assert (0.0 <= float(values["read_callback_last_complete_us"])
                <= float(values["read_callback_max_complete_us"]))
        assert (0.0 <= float(values["read_callback_previous_complete_us"])
                <= float(values["read_callback_max_complete_us"]))
        assert int(values["command_zero_count"]) > 0 and int(values["command_nonzero_count"]) == 0
        outcome["checks"].append("read path timing, last observed frame and zero command diagnostics")
        request = GetParameters.Request(names=["open_loop", "position_feedback", "enable_odom_tf"])
        parameters = call(GetParameters, "/wheel/get_parameters", request)
        assert [value.bool_value for value in parameters.values] == [False, True, False]
        outcome["checks"].append("runtime closed-loop position feedback and no odom TF")
        latest = samples["odom"][-1]
        assert latest["twist_covariance"][0] == 0.01 and latest["twist_covariance"][35] == 0.06
        assert latest["pose_covariance"][0] == 0.11 and latest["pose_covariance"][35] == 0.16
        outcome["checks"].append("synthetic covariance values propagated, NOT physical calibration")
        for increments, left_moves, right_moves in [((100, 0), True, False), ((0, 100), False, True), ((100, 100), True, True)]:
            mcu.increment = (0, 0)
            collect(0.3)
            before = samples["joints"][-1]
            mcu.increment = increments
            collect(0.8)
            after = samples["joints"][-1]
            delta_left = after["left_wheel_joint"] - before["left_wheel_joint"]
            delta_right = after["right_wheel_joint"] - before["right_wheel_joint"]
            assert (delta_left > 0.1 if left_moves else abs(delta_left) < 1e-10)
            assert (delta_right > 0.1 if right_moves else abs(delta_right) < 1e-10)
            assert samples["odom"][-1]["vx"] > 0.01
            assert all(request == (0, 0) for request in mcu.requests)
        outcome["checks"].append("independent left/right/same-sign accumulation, odom moves with feedback and zero commands")
        mcu.increment = (0, 0)
        collect(0.3)
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            command = TwistStamped()
            command.header.stamp = node.get_clock().now().to_msg()
            command.twist.linear.x = 0.1
            command.twist.angular.z = 0.2
            commands.publish(command)
            rclpy.spin_once(node, timeout_sec=0.02)
        assert (100, 200) in mcu.requests
        assert abs(samples["odom"][-1]["vx"]) < 1e-10 and abs(samples["odom"][-1]["wz"]) < 1e-10
        collect(0.4)
        assert mcu.requests[-1] == (0, 0)
        outcome["checks"].append("controller wheel commands encode body v/w; odom ignores commanded motion; command expiry")
        mcu.flags = (1, 1, 7, 9)
        wait_for(lambda: samples["diagnostics"][-1]["values"].get("right_alarm") == "9", "status diagnostics")
        values = samples["diagnostics"][-1]["values"]
        assert values["emergency"] == "1" and values["charge"] == "1" and values["left_alarm"] == "7"
        outcome["checks"].append("emergency/charge/alarms diagnostics")
        mcu.flags = (0, 0, 0, 0)
        switch = SwitchController.Request()
        switch.deactivate_controllers = ["wheel", "joint_state_broadcaster"]
        switch.strictness = SwitchController.Request.STRICT
        assert call(SwitchController, "/controller_manager/switch_controller", switch).ok
        state = SetHardwareComponentState.Request(name="FeedbotChassis", target_state=State(id=State.PRIMARY_STATE_INACTIVE))
        assert call(SetHardwareComponentState, "/controller_manager/set_hardware_component_state", state).ok
        assert mcu.owners() == []
        assert mcu.requests[-1] == (0, 0)
        state.target_state = State(id=State.PRIMARY_STATE_ACTIVE)
        assert call(SetHardwareComponentState, "/controller_manager/set_hardware_component_state", state).ok
        assert mcu.handshakes == 2 and len(mcu.owners()) == 1
        switch.activate_controllers = ["wheel", "joint_state_broadcaster"]
        switch.deactivate_controllers = []
        assert call(SwitchController, "/controller_manager/switch_controller", switch).ok
        collect(0.4)
        assert mcu.requests[-1] == (0, 0)
        outcome["checks"].append("deactivate releases serial; explicit activation re-handshakes without command replay")
        mcu.enabled = False
        wait_for(lambda: mcu.owners() == [], "feedback timeout closes serial")
        mcu.enabled = True
        collect(0.4)
        assert mcu.handshakes == 2 and mcu.owners() == []
        timeout_entries = [entry for entry in samples["diagnostics"]
                           if "feedback timeout" in entry["message"]]
        assert timeout_entries
        timeout_values = timeout_entries[-1]["values"]
        assert int(timeout_values["feedback_timeout_count"]) == 1
        assert float(timeout_values["last_receive_fault_monotonic_s"]) > 0.0
        assert len(timeout_values["last_observed_frame_hex"]) == 80
        timeout_entry = float(timeout_values["read_callback_entry_monotonic_s"])
        timeout_finished = float(timeout_values["read_callback_finished_monotonic_s"])
        timeout_last_us = float(timeout_values["read_callback_last_us"])
        assert timeout_entry <= timeout_finished
        assert abs(timeout_last_us - (timeout_finished - timeout_entry) * 1e6) < 3.0
        assert (float(timeout_values["read_callback_last_complete_us"])
                <= float(timeout_values["read_callback_max_complete_us"]))
        assert (float(timeout_values["read_callback_previous_complete_us"])
                <= float(timeout_values["read_callback_max_complete_us"]))
        for prefix in ("receive_total", "read_syscall_total", "parse_validate_total"):
            assert 0.0 <= float(timeout_values[prefix + "_last_us"]) <= float(timeout_values[prefix + "_max_us"])
        assert samples["odom_tf"] == []
        outcome["checks"].append("feedback loss reports fault, closes serial, no automatic reconnect/replay; no odom TF")
        outcome["status"] = "PASS"
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        log.close()
        outcome["handshakes"] = mcu.handshakes
        outcome["remaining_owners"] = mcu.owners()
        outcome["simulator_error"] = mcu.error
        (results / "result.json").write_text(json.dumps(outcome, indent=2) + "\n")
        (results / "samples.json").write_text(json.dumps(samples) + "\n")
        (results / "requests.json").write_text(json.dumps(mcu.requests) + "\n")
        mcu.close()
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(outcome, indent=2))


if __name__ == "__main__":
    main()

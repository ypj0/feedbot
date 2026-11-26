import json
import math
from pathlib import Path
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def transition(node, name):
    identifiers = {"configure": (1, 2), "activate": (3, 3), "deactivate": (4, 2), "cleanup": (2, 1)}
    identifier, expected = identifiers[name]
    client = node.create_client(ChangeState, "/ltr2_driver/change_state")
    if not client.wait_for_service(timeout_sec=8):
        raise RuntimeError("lifecycle service absent")
    request = ChangeState.Request()
    request.transition.id = identifier
    started = time.monotonic()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=12)
    if not future.done() or not future.result().success:
        raise RuntimeError(name + " transition failed")
    elapsed = time.monotonic() - started
    state_client = node.create_client(GetState, "/ltr2_driver/get_state")
    if not state_client.wait_for_service(timeout_sec=3):
        raise RuntimeError("lifecycle state service absent")
    future = state_client.call_async(GetState.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=3)
    if not future.done() or future.result().current_state.id != expected:
        raise RuntimeError(name + " returned wrong lifecycle state")
    return {"passed": True, "transition": name, "elapsed_s": elapsed, "state": expected}


def collect(node, silent, destination):
    frames = []
    diagnostics = []

    def receive(message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        geometry = message.angle_max - message.angle_min - (len(message.ranges) - 1) * message.angle_increment
        if len(message.ranges) != 1075 or len(message.intensities) != 1075:
            raise RuntimeError("crop count/intensity count mismatch")
        if abs(geometry) > 1e-6:
            raise RuntimeError("angular endpoint mismatch")
        if message.header.frame_id != "scan":
            raise RuntimeError("unexpected frame id")
        if frames and stamp <= frames[-1]["stamp"]:
            raise RuntimeError("timestamps not increasing")
        age = time.time() - stamp
        frames.append({"stamp": stamp, "received_monotonic": time.monotonic(),
                       "receipt_age_s": age, "count": len(message.ranges),
                       "angle_min": message.angle_min, "angle_max": message.angle_max,
                       "angle_increment": message.angle_increment, "geometry_error_rad": geometry,
                       "ranges": [value if math.isfinite(value) else None for value in message.ranges],
                       "intensities": list(message.intensities)})

    def diagnostic(message):
        for entry in message.status:
            if entry.name == "ltr2_ros2":
                diagnostics.append({"message": entry.message,
                                    "values": {value.key: value.value for value in entry.values}})

    node.create_subscription(LaserScan, "/scan", receive, qos_profile_sensor_data)
    node.create_subscription(DiagnosticArray, "/diagnostics", diagnostic, 10)
    started = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            if silent and now - started >= 2:
                break
            if not silent:
                if frames and now - frames[0]["received_monotonic"] >= 6:
                    break
                if not frames and now - started >= 8:
                    raise RuntimeError("no scan discovered within 8 seconds")
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        Path(destination).write_text(json.dumps({"passed": False, "frames": frames,
                                    "probe_started_monotonic": started,
                                    "diagnostics": diagnostics}, indent=2, allow_nan=False) + "\n")
    if silent:
        if frames:
            raise RuntimeError("scan published while inactive")
        return {"passed": True, "silence_s": time.monotonic() - started, "frames": []}
    if len(frames) < 80:
        raise RuntimeError("fewer than 80 fresh scans: received %d; diagnostic samples %d" %
                           (len(frames), len(diagnostics)))
    intervals = [later["received_monotonic"] - earlier["received_monotonic"]
                 for earlier, later in zip(frames, frames[1:])]
    rate = (len(frames) - 1) / (frames[-1]["received_monotonic"] - frames[0]["received_monotonic"])
    if not 14 <= rate <= 16 or max(intervals) > 0.2:
        raise RuntimeError("unstable scan rate: %s, max gap %s" % (rate, max(intervals)))
    if not diagnostics:
        raise RuntimeError("no diagnostics")
    values = diagnostics[-1]["values"]
    if values["stamp_source"] != "complete_frame_receive_system_time":
        raise RuntimeError("wrong stamp policy")
    if values["absolute_time_alignment_verified"] != "false":
        raise RuntimeError("unexpected absolute alignment claim")
    if not 13 <= float(values["frame_rate_hz"]) <= 17:
        raise RuntimeError("diagnostic frame rate outside expected range")
    int(values["discarded_frames"])
    return {"passed": True, "hz": rate, "maximum_gap_s": max(intervals),
            "probe_started_monotonic": started,
            "discovery_delay_s": frames[0]["received_monotonic"] - started,
            "receipt_age_s_range": [min(frame["receipt_age_s"] for frame in frames),
                                    max(frame["receipt_age_s"] for frame in frames)],
            "frames": frames, "diagnostics": diagnostics,
            "absolute_time_alignment_verified": False}


def main():
    rclpy.init()
    node = rclpy.create_node("ltr2_acceptance_probe")
    try:
        mode, destination = sys.argv[1:3]
        report = transition(node, mode) if mode in ("configure", "activate", "deactivate", "cleanup") else collect(node, mode == "silence", destination)
        Path(destination).write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        print(json.dumps({key: value for key, value in report.items() if key not in ("frames", "diagnostics")}))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import json
import math
from pathlib import Path
import sys
import time


FRESHNESS_LIMIT_SECONDS = 0.2
CONTINUOUS_STREAMS = ("scan", "imu", "wheel", "filtered")
REQUIRED_STREAMS = CONTINUOUS_STREAMS + ("diagnostics",)
EXPECTED_RATE_RANGES = {"scan": (14.0, 16.0), "imu": (190.0, 210.0), "wheel": (45.0, 55.0)}
EXPECTED_FIXED_EDGES = {
    ("base_link", "base_footprint"): (0.0, 0.0, 0.0),
    ("base_link", "left_wheel_link"): (0.0, 0.2275, 0.085),
    ("base_link", "right_wheel_link"): (0.0, -0.2275, 0.085),
    ("base_link", "imu_link"): (0.0, 0.0, 0.0),
    ("base_link", "laser_link"): (0.27, 0.0, 0.20),
    ("laser_link", "scan"): (0.0, 0.0, 0.0),
}


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def diagnostic_level(value):
    if isinstance(value, int):
        return value
    return int(value[0])


def summarize_samples(samples, finished_monotonic):
    received = [sample["received_monotonic"] for sample in samples]
    stamps = [sample["stamp_ns"] for sample in samples]
    result = {
        "count": len(samples),
        "last_receive_age_s": finished_monotonic - received[-1],
        "stamp_strictly_monotonic": all(after > before for before, after in zip(stamps, stamps[1:])),
    }
    if len(samples) > 1:
        duration = received[-1] - received[0]
        result["rate_hz"] = (len(samples) - 1) / duration if duration > 0 else None
        result["maximum_receive_gap_s"] = max(
            after - before for before, after in zip(received, received[1:]))
    else:
        result["rate_hz"] = None
        result["maximum_receive_gap_s"] = None
    return result


def evaluate(data, finished_monotonic, transforms=None, finished_wall_ns=None, require_map=False):
    required_streams = REQUIRED_STREAMS + (("map",) if require_map else ())
    streams = {}
    failures = []
    for name in required_streams:
        samples = data[name]
        if not samples:
            streams[name] = {"count": 0}
            failures.append("missing topic data: /{}".format({
                "wheel": "wheel/odom", "filtered": "odometry/filtered"
            }.get(name, name)))
            continue
        streams[name] = summarize_samples(samples, finished_monotonic)

    for name in CONTINUOUS_STREAMS:
        summary = streams[name]
        if summary["count"] < 2:
            failures.append("insufficient samples: {}".format(name))
            continue
        if not summary["stamp_strictly_monotonic"]:
            failures.append("non-monotonic stamps: {}".format(name))
        if summary["maximum_receive_gap_s"] > FRESHNESS_LIMIT_SECONDS:
            failures.append("receive gap exceeds 0.2 s: {}".format(name))
        if summary["last_receive_age_s"] > FRESHNESS_LIMIT_SECONDS:
            failures.append("stream stale at finish: {}".format(name))
        if finished_wall_ns is not None:
            summary["last_stamp_age_s"] = (finished_wall_ns - data[name][-1]["stamp_ns"]) / 1e9
            if abs(summary["last_stamp_age_s"]) > FRESHNESS_LIMIT_SECONDS:
                failures.append("message stamp differs from system time by more than 0.2 s: {}".format(name))
        if name in EXPECTED_RATE_RANGES and summary["rate_hz"] is not None:
            minimum, maximum = EXPECTED_RATE_RANGES[name]
            if not minimum <= summary["rate_hz"] <= maximum:
                failures.append("rate outside {}-{} Hz: {}".format(minimum, maximum, name))

    covariance_fields = {
        "imu": {"angular_velocity_covariance": (8,)},
        "wheel": {"twist_covariance": (0, 35)},
        "filtered": {"twist_covariance": (0, 35)},
    }
    for name, fields in covariance_fields.items():
        if not data[name]:
            continue
        for field, indices in fields.items():
            if any(not all(len(sample[field]) > index and math.isfinite(sample[field][index])
                           and sample[field][index] > 0 for index in indices)
                   for sample in data[name]):
                failures.append("invalid fused covariance: {}.{}".format(name, field))

    diagnostic_errors = []
    startup_diagnostics = []
    first_filtered_stamp = data["filtered"][0]["stamp_ns"] if data["filtered"] else None
    for sample in data["diagnostics"]:
        for status in sample["status"]:
            if status["level"] >= 2:
                values = status.get("values", {})
                if (first_filtered_stamp is not None and 0 < sample["stamp_ns"] < first_filtered_stamp
                        and status["name"] == "ekf_filter_node: odometry/filtered topic status"
                        and status["level"] == 2 and status["message"] == "No events recorded."
                        and values.get("Events since startup") == "0"
                        and values.get("Events in window") == "0"):
                    startup_diagnostics.append({"stamp_ns": sample["stamp_ns"],
                                                "received_monotonic": sample["received_monotonic"],
                                                "status": status})
                else:
                    diagnostic_errors.append(status)
    if diagnostic_errors:
        failures.append("diagnostic ERROR or STALE reported")
    command_nonzero_counts = []
    for sample in data["diagnostics"]:
        for status in sample["status"]:
            values = status.get("values", {})
            if "command_nonzero_count" in values:
                command_nonzero_counts.append(int(values["command_nonzero_count"]))
    if not command_nonzero_counts:
        failures.append("command_nonzero_count missing from diagnostics")
    elif any(value != 0 for value in command_nonzero_counts):
        failures.append("nonzero chassis command observed")

    scan_geometry = data["scan"][-1]["geometry"] if data["scan"] else None
    for sample in data["scan"]:
        geometry = sample["geometry"]
        finite = all(math.isfinite(value) for value in geometry.values())
        closed_angle = geometry["angle_min"] + geometry["angle_increment"] * (geometry["beam_count"] - 1)
        if (not finite or sample["frame_id"] != "scan" or geometry["range_min"] < 0
                or geometry["range_max"] <= geometry["range_min"]
                or geometry["angle_increment"] <= 0 or geometry["beam_count"] != 1075
                or geometry["intensity_count"] != 1075
                or abs(closed_angle - geometry["angle_max"]) >= 1e-6):
            failures.append("invalid scan geometry")
            break

    transform_summary = {}
    if transforms is not None:
        latest = {(item["parent"], item["child"]): item for item in transforms}
        for edge, expected_translation in EXPECTED_FIXED_EDGES.items():
            label = "{} -> {}".format(*edge)
            if edge not in latest:
                failures.append("missing transform: {}".format(label))
                continue
            item = latest[edge]
            transform_summary[label] = item
            if any(abs(actual - expected) > 1e-9
                   for actual, expected in zip(item["translation"], expected_translation)):
                failures.append("unexpected translation: {}".format(label))
            if edge not in (("base_link", "left_wheel_link"), ("base_link", "right_wheel_link")) and any(
                    abs(actual - expected) > 1e-9
                    for actual, expected in zip(item["rotation"], (0.0, 0.0, 0.0, 1.0))):
                failures.append("unexpected rotation: {}".format(label))
        for edge in ((("map", "odom"),) if require_map else ()) + (("odom", "base_link"),):
            if edge not in latest:
                failures.append("missing transform: {} -> {}".format(*edge))

    ready_monotonic = (max(data[name][0]["received_monotonic"] for name in required_streams)
                       if all(data[name] for name in required_streams) else None)
    return {
        "passed": not failures,
        "freshness_limit_s": FRESHNESS_LIMIT_SECONDS,
        "streams": streams,
        "scan_geometry": scan_geometry,
        "diagnostic_errors": diagnostic_errors,
        "startup_diagnostics": startup_diagnostics,
        "all_streams_observed_monotonic": ready_monotonic,
        "runtime_observed_s": (finished_monotonic - ready_monotonic
                               if ready_monotonic is not None else 0.0),
        "command_nonzero_counts": command_nonzero_counts,
        "transforms": transform_summary,
        "samples": data,
        "transform_samples": transforms if transforms is not None else [],
        "failures": failures,
    }


def read_system_time_parameter(node, full_name):
    import rclpy
    from rclpy.parameter_client import AsyncParameterClient

    client = AsyncParameterClient(node, full_name)
    if not client.wait_for_services(timeout_sec=0.5):
        return {"available": False}
    future = client.get_parameters(["use_sim_time"])
    rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
    if not future.done() or future.exception() is not None:
        return {"available": False}
    value = future.result().values[0]
    return {"available": value.type == 1, "value": value.bool_value}


def main(require_map=False):
    required_streams = REQUIRED_STREAMS + (("map",) if require_map else ())
    if len(sys.argv) != 3:
        raise SystemExit("usage: static_probe.py OUTPUT_JSON DURATION_SECONDS")
    output_path = Path(sys.argv[1])
    duration = float(sys.argv[2])
    if duration <= 0:
        raise SystemExit("DURATION_SECONDS must be positive")

    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, LaserScan
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import DurabilityPolicy, QoSProfile

    rclpy.init()
    node = rclpy.create_node("feedbot_mapping_static_probe" if require_map else "feedbot_base_probe")
    data = {name: [] for name in required_streams}
    transforms = []

    def record(name, message, extra=None):
        sample = {"received_monotonic": time.monotonic(), "stamp_ns": stamp_ns(message)}
        if extra:
            sample.update(extra)
        data[name].append(sample)

    def record_transforms(topic, message):
        received = time.monotonic()
        for transform in message.transforms:
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            transforms.append({
                "topic": topic, "received_monotonic": received,
                "stamp_ns": stamp_ns(transform), "parent": transform.header.frame_id,
                "child": transform.child_frame_id,
                "translation": [translation.x, translation.y, translation.z],
                "rotation": [rotation.x, rotation.y, rotation.z, rotation.w],
            })

    subscriptions = [
        node.create_subscription(LaserScan, "/scan", lambda message: record("scan", message, {
            "geometry": {
                "angle_min": message.angle_min, "angle_max": message.angle_max,
                "angle_increment": message.angle_increment, "time_increment": message.time_increment,
                "scan_time": message.scan_time, "range_min": message.range_min,
                "range_max": message.range_max, "beam_count": len(message.ranges),
                "intensity_count": len(message.intensities),
            }, "frame_id": message.header.frame_id,
        }), qos_profile_sensor_data),
        node.create_subscription(Imu, "/imu", lambda message: record("imu", message, {
            "frame_id": message.header.frame_id,
            "angular_velocity": [message.angular_velocity.x, message.angular_velocity.y,
                                 message.angular_velocity.z],
            "angular_velocity_covariance": list(message.angular_velocity_covariance),
            "orientation_covariance": list(message.orientation_covariance),
            "linear_acceleration_covariance": list(message.linear_acceleration_covariance),
        }), qos_profile_sensor_data),
        node.create_subscription(Odometry, "/wheel/odom", lambda message: record(
            "wheel", message, {"frame_id": message.header.frame_id,
                               "child_frame_id": message.child_frame_id,
                               "pose": [message.pose.pose.position.x, message.pose.pose.position.y,
                                        message.pose.pose.position.z],
                               "twist": [message.twist.twist.linear.x, message.twist.twist.linear.y,
                                         message.twist.twist.angular.z],
                               "pose_covariance": list(message.pose.covariance),
                               "twist_covariance": list(message.twist.covariance)}), 100),
        node.create_subscription(Odometry, "/odometry/filtered", lambda message: record(
            "filtered", message, {"frame_id": message.header.frame_id,
                                  "child_frame_id": message.child_frame_id,
                                  "pose": [message.pose.pose.position.x, message.pose.pose.position.y,
                                           message.pose.pose.position.z],
                                  "twist": [message.twist.twist.linear.x, message.twist.twist.linear.y,
                                            message.twist.twist.angular.z],
                                  "pose_covariance": list(message.pose.covariance),
                                  "twist_covariance": list(message.twist.covariance)}), 100),
        node.create_subscription(DiagnosticArray, "/diagnostics", lambda message: record(
            "diagnostics", message, {"status": [
                {"name": status.name, "level": diagnostic_level(status.level), "message": status.message,
                 "hardware_id": status.hardware_id,
                 "values": {value.key: value.value for value in status.values}}
                for status in message.status
            ]}), 100),
        node.create_subscription(TFMessage, "/tf", lambda message: record_transforms("/tf", message), 100),
        node.create_subscription(
            TFMessage, "/tf_static", lambda message: record_transforms("/tf_static", message),
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
    ]
    if require_map:
        subscriptions.append(node.create_subscription(OccupancyGrid, "/map", lambda message: record(
            "map", message, {"frame_id": message.header.frame_id, "width": message.info.width,
                             "height": message.info.height, "resolution": message.info.resolution}), 10))
    start = time.monotonic()
    deadline = start + duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
    finished = time.monotonic()
    finished_wall_ns = time.time_ns()
    for subscription in subscriptions:
        node.destroy_subscription(subscription)
    frozen_data = {name: list(samples) for name, samples in data.items()}
    frozen_transforms = list(transforms)
    output_path.with_suffix(".samples.json").write_text(json.dumps({
        "samples": frozen_data, "transform_samples": frozen_transforms,
        "started_monotonic": start, "finished_monotonic": finished,
        "finished_wall_ns": finished_wall_ns,
    }) + "\n")

    endpoint_topics = ("/scan", "/imu", "/wheel/odom", "/odometry/filtered",
                       "/diagnostics", "/tf", "/tf_static") + (("/map",) if require_map else ())
    endpoints = {
        topic: [{"node_name": info.node_name, "node_namespace": info.node_namespace,
                 "gid": bytes(info.endpoint_gid).hex()}
                for info in node.get_publishers_info_by_topic(topic)]
        for topic in endpoint_topics
    }
    publisher_nodes = sorted({
        (entry["node_namespace"].rstrip("/") + "/" + entry["node_name"]).replace("//", "/")
        for entries in endpoints.values() for entry in entries
    })
    use_sim_time = {}
    for full_name in publisher_nodes:
        use_sim_time[full_name] = read_system_time_parameter(node, full_name)

    result = evaluate(frozen_data, finished, frozen_transforms, finished_wall_ns, require_map)
    result.update({
        "duration_requested_s": duration,
        "duration_observed_s": finished - start,
        "finished_monotonic": finished,
        "endpoints": endpoints,
        "use_sim_time": use_sim_time,
        "tf_provenance": "See session tf_edge_probe, topic info, and view_frames artifacts",
    })
    for name, record_value in use_sim_time.items():
        if not record_value["available"]:
            result["failures"].append("use_sim_time unavailable: {}".format(name))
        elif record_value["value"] is not False:
            result["failures"].append("use_sim_time is not false: {}".format(name))
    result["passed"] = not result["failures"]
    output_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps({key: value for key, value in result.items()
                      if key not in ("samples", "transform_samples")}, indent=2, sort_keys=True))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

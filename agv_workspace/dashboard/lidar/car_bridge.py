#!/usr/bin/env python2
"""Read-only ROS1 LaserScan to bounded HTTP snapshot bridge."""

from __future__ import print_function

import argparse
import collections
import ctypes
import ctypes.util
import json
import math
import os
import threading
import time

try:
    from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
    from SocketServer import ThreadingMixIn
except ImportError:
    from http.server import BaseHTTPRequestHandler, HTTPServer
    from socketserver import ThreadingMixIn


SCAN_TOPIC = "/candidate/ltr2/managed_scan"
DIAGNOSTIC_TOPIC = "/candidate/ltr2/managed_diagnostics"
POINT_COUNT = 1075
MAX_RESPONSE_BYTES = 131072


class Timespec(ctypes.Structure):
    _fields_ = [("seconds", ctypes.c_long), ("nanoseconds", ctypes.c_long)]


def ctypes_monotonic_seconds():
    library = ctypes.CDLL(None, use_errno=True)
    try:
        clock_gettime = library.clock_gettime
    except AttributeError:
        realtime_library = ctypes.util.find_library("rt") or "librt.so.1"
        clock_gettime = ctypes.CDLL(realtime_library, use_errno=True).clock_gettime
    clock_gettime.argtypes = [ctypes.c_int, ctypes.POINTER(Timespec)]
    clock_gettime.restype = ctypes.c_int
    value = Timespec()
    clock_monotonic = getattr(time, "CLOCK_MONOTONIC", 1)
    if clock_gettime(clock_monotonic, ctypes.byref(value)) != 0:
        error_number = ctypes.get_errno()
        raise OSError(error_number, os.strerror(error_number))
    return float(value.seconds) + float(value.nanoseconds) * 1e-9


def monotonic_seconds():
    native_monotonic = getattr(time, "monotonic", None)
    if native_monotonic is not None:
        return native_monotonic()
    return ctypes_monotonic_seconds()


def finite_number(value):
    try:
        return math.isfinite(float(value))
    except AttributeError:
        number = float(value)
        return not math.isnan(number) and not math.isinf(number)
    except (TypeError, ValueError):
        return False


def json_number(value):
    return float(value) if finite_number(value) else None


def stamp_seconds(stamp):
    if hasattr(stamp, "to_sec"):
        return json_number(stamp.to_sec())
    seconds = getattr(stamp, "secs", 0)
    nanoseconds = getattr(stamp, "nsecs", 0)
    return json_number(float(seconds) + float(nanoseconds) * 1e-9)


def boolean_value(value):
    if value == "true":
        return True
    if value == "false":
        return False
    return None


class SnapshotStore(object):
    def __init__(self, clock=None, stale_after=0.5, diagnostic_stale_after=1.5):
        self.clock = clock or monotonic_seconds
        self.stale_after = stale_after
        self.diagnostic_stale_after = diagnostic_stale_after
        self.lock = threading.Lock()
        self.scan = None
        self.diagnostic = None
        self.receive_times = collections.deque(maxlen=32)
        self.received_scan_count = 0
        self.rejected_scan_count = 0
        self.last_error = None

    def update_scan(self, message):
        received_monotonic = self.clock()
        try:
            scan = self._build_scan(message, received_monotonic)
        except (TypeError, ValueError) as error:
            with self.lock:
                self.rejected_scan_count += 1
                self.last_error = str(error)
            return False
        with self.lock:
            self.scan = scan
            self.receive_times.append(received_monotonic)
            self.received_scan_count += 1
            self.last_error = None
        return True

    def _build_scan(self, message, received_monotonic):
        ranges = list(message.ranges)
        if len(ranges) != POINT_COUNT:
            raise ValueError("scan must contain exactly 1075 ranges")
        geometry_names = (
            "angle_min", "angle_max", "angle_increment", "range_min", "range_max",
            "scan_time", "time_increment",
        )
        geometry = {}
        for name in geometry_names:
            value = getattr(message, name)
            if not finite_number(value):
                raise ValueError("scan geometry contains a non-finite value")
            geometry[name] = float(value)
        if geometry["angle_increment"] <= 0:
            raise ValueError("angle_increment must be positive")
        if geometry["range_min"] <= 0 or geometry["range_max"] <= geometry["range_min"]:
            raise ValueError("range bounds are invalid")
        implied_count = (
            (geometry["angle_max"] - geometry["angle_min"]) /
            geometry["angle_increment"] + 1.0
        )
        if abs(implied_count - POINT_COUNT) > 0.05:
            raise ValueError("angular geometry does not describe 1075 samples")

        serial_ranges = []
        valid_return_count = 0
        finite_out_of_range_count = 0
        no_return_count = 0
        invalid_range_count = 0
        for value in ranges:
            try:
                number = float(value)
            except (TypeError, ValueError):
                serial_ranges.append(None)
                invalid_range_count += 1
                continue
            if math.isinf(number) and number > 0:
                serial_ranges.append(None)
                no_return_count += 1
            elif not finite_number(number):
                serial_ranges.append(None)
                invalid_range_count += 1
            else:
                serial_ranges.append(number)
                if geometry["range_min"] <= number <= geometry["range_max"]:
                    valid_return_count += 1
                else:
                    finite_out_of_range_count += 1

        header = getattr(message, "header", None)
        return {
            "received_monotonic_s": received_monotonic,
            "header_seq": int(getattr(header, "seq", 0)),
            "header_stamp_s": stamp_seconds(getattr(header, "stamp", None)),
            "frame_id": str(getattr(header, "frame_id", "")),
            "geometry": geometry,
            "ranges": serial_ranges,
            "stats": {
                "point_count": POINT_COUNT,
                "valid_return_count": valid_return_count,
                "finite_out_of_range_count": finite_out_of_range_count,
                "no_return_count": no_return_count,
                "invalid_range_count": invalid_range_count,
            },
        }

    def update_diagnostics(self, message):
        received_monotonic = self.clock()
        for entry in message.status:
            if entry.name != "ltr2_managed_data":
                continue
            values = dict((item.key, item.value) for item in entry.values)
            diagnostic = {
                "received_monotonic_s": received_monotonic,
                "hardware_id": str(entry.hardware_id),
                "level": int(entry.level),
                "state": str(entry.message),
                "ready": boolean_value(values.get("ready")),
                "healthy": boolean_value(values.get("healthy")),
                "navigation_accepted": boolean_value(values.get("navigation_accepted")),
            }
            with self.lock:
                self.diagnostic = diagnostic
            return True
        return False

    def snapshot(self):
        sampled_monotonic = self.clock()
        with self.lock:
            scan = self.scan
            diagnostic = self.diagnostic
            receive_times = list(self.receive_times)
            received_scan_count = self.received_scan_count
            rejected_scan_count = self.rejected_scan_count
            last_error = self.last_error

        source_age = None
        live = False
        if scan is not None:
            source_age = max(0.0, sampled_monotonic - scan["received_monotonic_s"])
            live = source_age <= self.stale_after

        diagnostic_payload = {
            "fresh": False,
            "age_s": None,
            "hardware_id": None,
            "level": None,
            "state": "unavailable",
            "ready": None,
            "healthy": None,
            "navigation_accepted": None,
        }
        if diagnostic is not None:
            diagnostic_age = max(0.0, sampled_monotonic - diagnostic["received_monotonic_s"])
            diagnostic_payload["age_s"] = diagnostic_age
            if diagnostic_age <= self.diagnostic_stale_after:
                diagnostic_payload.update(
                    fresh=True,
                    hardware_id=diagnostic["hardware_id"],
                    level=diagnostic["level"],
                    state=diagnostic["state"],
                    ready=diagnostic["ready"],
                    healthy=diagnostic["healthy"],
                    navigation_accepted=diagnostic["navigation_accepted"],
                )

        frequency = None
        if len(receive_times) > 1:
            elapsed = receive_times[-1] - receive_times[0]
            if elapsed > 0:
                frequency = float(len(receive_times) - 1) / elapsed

        payload = {
            "schema_version": 1,
            "origin": "managed_candidate_not_factory_scan",
            "scan_topic": SCAN_TOPIC,
            "diagnostic_topic": DIAGNOSTIC_TOPIC,
            "live": live,
            "status": "live" if live else ("stale" if scan is not None else "waiting"),
            "source_age_s": source_age,
            "source_sample_monotonic_s": sampled_monotonic,
            "frequency_hz": frequency,
            "received_scan_count": received_scan_count,
            "rejected_scan_count": rejected_scan_count,
            "last_error": last_error,
            "diagnostic": diagnostic_payload,
            "scan": None,
            "stats": None,
        }
        if live:
            payload["scan"] = {
                "header_seq": scan["header_seq"],
                "header_stamp_s": scan["header_stamp_s"],
                "frame_id": scan["frame_id"],
                "geometry": scan["geometry"],
                "ranges": scan["ranges"],
            }
            payload["stats"] = scan["stats"]
        return payload


def encode_json(payload):
    encoded = json.dumps(payload, allow_nan=False, separators=(",", ":"), sort_keys=True)
    if not isinstance(encoded, bytes):
        encoded = encoded.encode("utf-8")
    if len(encoded) > MAX_RESPONSE_BYTES:
        raise ValueError("snapshot exceeds response size limit")
    return encoded


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class SnapshotHandler(BaseHTTPRequestHandler):
    server_version = "FeedbotLidarSnapshot/1"

    def do_GET(self):
        if self.path != "/api/scan":
            self.send_error(404)
            return
        payload = self.server.snapshot_store.snapshot()
        status = 200 if payload["live"] else 503
        try:
            body = encode_json(payload)
        except ValueError:
            self.send_error(500)
            return
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, message_format, *arguments):
        return


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="10.7.5.88")
    parser.add_argument("--port", type=int, default=8096)
    parser.add_argument("--stale-after", type=float, default=0.5)
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    if not 1 <= arguments.port <= 65535:
        raise SystemExit("port must be in [1, 65535]")
    if not 0.1 <= arguments.stale_after <= 2.0:
        raise SystemExit("stale-after must be in [0.1, 2.0] seconds")

    import rospy
    from diagnostic_msgs.msg import DiagnosticArray
    from sensor_msgs.msg import LaserScan

    rospy.init_node("ltr2_readonly_dashboard_bridge", anonymous=False)
    store = SnapshotStore(stale_after=arguments.stale_after)
    rospy.Subscriber(SCAN_TOPIC, LaserScan, store.update_scan, queue_size=1)
    rospy.Subscriber(DIAGNOSTIC_TOPIC, DiagnosticArray, store.update_diagnostics, queue_size=1)
    server = ThreadingHTTPServer((arguments.bind, arguments.port), SnapshotHandler)
    server.snapshot_store = store
    print("read-only lidar snapshot listening on %s:%d" % (arguments.bind, arguments.port))
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

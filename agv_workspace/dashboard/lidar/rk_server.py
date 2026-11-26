#!/usr/bin/env python3
"""Serve the lidar dashboard and proxy only the fixed car snapshot endpoint."""

import argparse
import json
import math
from pathlib import Path
import socket
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


UPSTREAM_URL = "http://10.7.5.88:8096/api/scan"
UPSTREAM_TIMEOUT_SECONDS = 0.8
MAX_UPSTREAM_BYTES = 131072
POINT_COUNT = 1075
STATIC_DIRECTORY = Path(__file__).resolve().parent / "static"
STATIC_FILES = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/app.js": ("app.js", "text/javascript; charset=utf-8"),
    "/style.css": ("style.css", "text/css; charset=utf-8"),
}


def strict_json_loads(data):
    def reject_constant(value):
        raise ValueError("non-standard JSON constant: " + value)
    return json.loads(data.decode("utf-8"), parse_constant=reject_constant)


def finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def validate_snapshot(payload, require_live):
    if not isinstance(payload, dict):
        raise ValueError("snapshot is not an object")
    if payload.get("schema_version") != 1:
        raise ValueError("unsupported snapshot schema")
    if payload.get("origin") != "managed_candidate_not_factory_scan":
        raise ValueError("unexpected data origin")
    if not isinstance(payload.get("live"), bool):
        raise ValueError("live flag is missing")
    source_age = payload.get("source_age_s")
    if source_age is not None and (not finite_number(source_age) or source_age < 0):
        raise ValueError("source age is invalid")
    if require_live and not payload["live"]:
        raise ValueError("upstream returned non-live data with success status")
    if not payload["live"]:
        if payload.get("scan") is not None:
            raise ValueError("non-live snapshot must not contain scan data")
        return payload

    scan = payload.get("scan")
    stats = payload.get("stats")
    if not isinstance(scan, dict) or not isinstance(stats, dict):
        raise ValueError("live snapshot is incomplete")
    ranges = scan.get("ranges")
    if not isinstance(ranges, list) or len(ranges) != POINT_COUNT:
        raise ValueError("scan must contain exactly 1075 ranges")
    for value in ranges:
        if value is not None and not finite_number(value):
            raise ValueError("range is not finite or null")
    geometry = scan.get("geometry")
    if not isinstance(geometry, dict):
        raise ValueError("scan geometry is missing")
    required_geometry = (
        "angle_min", "angle_max", "angle_increment", "range_min", "range_max",
        "scan_time", "time_increment",
    )
    for name in required_geometry:
        if not finite_number(geometry.get(name)):
            raise ValueError("scan geometry is invalid")
    if geometry["angle_increment"] <= 0:
        raise ValueError("angle increment is invalid")
    implied_count = (
        (geometry["angle_max"] - geometry["angle_min"]) /
        geometry["angle_increment"] + 1.0
    )
    if abs(implied_count - POINT_COUNT) > 0.05:
        raise ValueError("scan geometry does not match 1075 points")
    if stats.get("point_count") != POINT_COUNT:
        raise ValueError("point statistics do not match scan")
    valid_return_count = stats.get("valid_return_count")
    if not isinstance(valid_return_count, int) or not 0 <= valid_return_count <= POINT_COUNT:
        raise ValueError("valid return count is invalid")
    return payload


def error_payload(status, detail):
    return {
        "schema_version": 1,
        "origin": "managed_candidate_not_factory_scan",
        "live": False,
        "status": status,
        "detail": detail,
        "source_age_s": None,
        "scan": None,
        "stats": None,
    }


def read_bounded(response):
    data = response.read(MAX_UPSTREAM_BYTES + 1)
    if len(data) > MAX_UPSTREAM_BYTES:
        raise ValueError("upstream response exceeds size limit")
    return data


def fetch_upstream(opener=urlopen):
    request = Request(UPSTREAM_URL, headers={"Accept": "application/json"}, method="GET")
    try:
        response = opener(request, timeout=UPSTREAM_TIMEOUT_SECONDS)
        with response:
            payload = strict_json_loads(read_bounded(response))
            validate_snapshot(payload, require_live=True)
            return 200, payload
    except HTTPError as error:
        if error.code != 503:
            return 502, error_payload("upstream_http_error", "car bridge returned HTTP %d" % error.code)
        try:
            with error:
                payload = strict_json_loads(read_bounded(error))
            validate_snapshot(payload, require_live=False)
            return 503, payload
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError):
            return 502, error_payload("invalid_upstream", "car bridge stale response was invalid")
    except (URLError, socket.timeout, TimeoutError):
        return 502, error_payload("upstream_unavailable", "car bridge is unreachable")
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError):
        return 502, error_payload("invalid_upstream", "car bridge response failed validation")


def encode_json(payload):
    return json.dumps(payload, allow_nan=False, separators=(",", ":"), sort_keys=True).encode("utf-8")


class DashboardHandler(BaseHTTPRequestHandler):
    server_version = "FeedbotLidarDashboard/1"

    def do_GET(self):
        if self.path == "/api/scan":
            status, payload = fetch_upstream()
            self.send_body(status, encode_json(payload), "application/json; charset=utf-8")
            return
        static_file = STATIC_FILES.get(self.path)
        if static_file is None:
            self.send_error(404)
            return
        filename, content_type = static_file
        try:
            body = (STATIC_DIRECTORY / filename).read_bytes()
        except OSError:
            self.send_error(500)
            return
        self.send_body(200, body, content_type)

    def send_body(self, status, body, content_type):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'self'; connect-src 'self'; style-src 'self'; script-src 'self'; object-src 'none'; base-uri 'none'")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, message_format, *arguments):
        return


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8086)
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    if not 1 <= arguments.port <= 65535:
        raise SystemExit("port must be in [1, 65535]")
    server = ThreadingHTTPServer((arguments.bind, arguments.port), DashboardHandler)
    print("read-only lidar dashboard listening on %s:%d" % (arguments.bind, arguments.port))
    print("fixed upstream: " + UPSTREAM_URL)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

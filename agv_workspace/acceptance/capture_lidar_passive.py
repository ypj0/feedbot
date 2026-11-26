#!/usr/bin/env python3
"""Capture only incoming lidar UDP frames without claiming the UDP control port."""

import argparse
import hashlib
import json
from pathlib import Path
import socket
import struct
import time


def lidar_udp(packet):
    if len(packet) < 42 or packet[12:14] != b"\x08\x00":
        return False
    if packet[14] >> 4 != 4 or packet[23] != 17:
        return False
    if packet[26:30] != socket.inet_aton("192.168.10.160"):
        return False
    header_size = (packet[14] & 15) * 4
    total_size = struct.unpack_from("!H", packet, 16)[0]
    if header_size < 20 or total_size < header_size + 8 or len(packet) < 14 + total_size:
        return False
    if struct.unpack_from("!H", packet, 20)[0] & 0x3fff:
        return False
    offset = 14 + header_size
    source_port, target_port, udp_size = struct.unpack_from("!HHH", packet, offset)
    return source_port == target_port == 2105 and udp_size == total_size - header_size


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="enp2s0")
    parser.add_argument("--duration", type=float, default=10)
    parser.add_argument("--output", required=True)
    arguments = parser.parse_args()
    if not 1 <= arguments.duration <= 30:
        parser.error("duration must be in [1, 30] seconds")
    output = Path(arguments.output)
    report = {"interface": arguments.interface, "duration_requested": arguments.duration,
              "timestamp_source": "host userspace receive wall clock; not hardware capture time",
              "scope": "incoming IPv4 lidar UDP only; no promiscuous mode or sends",
              "all_hardware_accepted": False, "packets": 0, "bytes": 0, "complete": False}
    digest = hashlib.sha256()
    start = time.monotonic()
    with output.open("xb") as capture:
        with socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0800)) as receiver:
            receiver.bind((arguments.interface, 0))
            receiver.settimeout(0.2)
            while time.monotonic() - start < arguments.duration:
                try:
                    packet, address = receiver.recvfrom(65535)
                except socket.timeout:
                    continue
                if address[2] == socket.PACKET_OUTGOING or not lidar_udp(packet):
                    continue
                record = struct.pack("<dI", time.time(), len(packet)) + packet
                if report["bytes"] + len(record) > 32 * 1024 * 1024:
                    report["error"] = "capture size limit reached"
                    break
                capture.write(record)
                digest.update(record)
                report["packets"] += 1
                report["bytes"] += len(record)
            else:
                report["complete"] = True
    report.update(elapsed_seconds=time.monotonic() - start, sha256=digest.hexdigest())
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

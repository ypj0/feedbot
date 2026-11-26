#!/usr/bin/env python3
"""Bounded host inventory without opening robot devices or changing services."""

import datetime
import json
from pathlib import Path
import subprocess


def collect():
    commands = {
        "kernel": ["uname", "-a"],
        "os": ["cat", "/etc/os-release"],
        "memory": ["free", "-m"],
        "disk": ["df", "-h", "/", "/tmp"],
        "usb": ["lsusb"],
        "usb_tree": ["lsusb", "-t"],
        "devices": ["ls", "-l", "/dev/rikibase", "/dev/rikiimu"],
        "owners": ["fuser", "-v", "/dev/rikibase", "/dev/rikiimu"],
        "chassis_udev": ["udevadm", "info", "--query=property", "--name=/dev/ttyUSB1"],
        "imu_udev": ["udevadm", "info", "--query=property", "--name=/dev/ttyUSB0"],
        "addresses": ["ip", "-br", "addr"],
        "ports": ["ss", "-lntup"],
        "factory": ["systemctl", "show", "spirit", "-p", "ActiveState", "-p", "MainPID"],
        "maintenance_observer": ["systemctl", "show", "feedbot-maintenance-observer", "-p", "ActiveState"],
        "process_load": ["ps", "-eo", "pid,comm,pcpu,pmem", "--sort=-pcpu"],
        "compiler": ["g++", "--version"],
    }
    report = {"started_utc": datetime.datetime.utcnow().isoformat() + "Z",
              "scope": "read-only inventory", "all_hardware_accepted": False, "commands": {}}
    for name, command in sorted(commands.items()):
        try:
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    universal_newlines=True, timeout=5)
            report["commands"][name] = {"argv": command, "returncode": result.returncode,
                                         "output": result.stdout[:65536]}
        except (OSError, subprocess.TimeoutExpired) as error:
            report["commands"][name] = {"argv": command, "returncode": None, "error": str(error)}
    report["usb_identifiers"] = []
    for device in sorted(Path("/sys/bus/usb/devices").glob("*")):
        identity = {"sysfs": str(device)}
        for name in ("idVendor", "idProduct", "manufacturer", "product", "serial", "speed"):
            candidate = device / name
            if candidate.is_file():
                try:
                    identity[name] = candidate.read_text().strip()
                except OSError as error:
                    identity[name] = str(error)
        if "idVendor" in identity:
            report["usb_identifiers"].append(identity)
    report["finished_utc"] = datetime.datetime.utcnow().isoformat() + "Z"
    return report


if __name__ == "__main__":
    print(json.dumps(collect(), indent=2, sort_keys=True))

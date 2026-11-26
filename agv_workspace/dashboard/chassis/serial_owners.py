import json
import os
from pathlib import Path
import stat


def main():
    device = os.stat("/dev/rikibase").st_rdev
    owners = []
    for process in Path("/proc").iterdir():
        if not process.name.isdigit():
            continue
        try:
            descriptors = list((process / "fd").iterdir())
        except FileNotFoundError:
            continue
        for descriptor in descriptors:
            try:
                info = descriptor.stat()
                if stat.S_ISCHR(info.st_mode) and info.st_rdev == device:
                    owners.append({"pid": int(process.name), "fd": descriptor.name,
                                   "command": (process / "cmdline").read_bytes().replace(b"\0", b" ").decode()})
            except FileNotFoundError:
                continue
    print(json.dumps({"major": os.major(device), "minor": os.minor(device), "owners": owners}, indent=2))
    return 1 if owners else 0


if __name__ == "__main__":
    raise SystemExit(main())

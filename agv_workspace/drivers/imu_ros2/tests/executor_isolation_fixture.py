import json
import os
from pathlib import Path
import pty
import re
import signal
import subprocess
import sys
import threading
import time
import tty

from observation_fixture import owners, packet


def main():
    directory = Path(sys.argv[1])
    binary, injection, expected = sys.argv[2:5]
    assert expected in ("timer", "thread")
    assert Path("/.dockerenv").exists() and not os.path.lexists("/dev/rikiimu")
    directory.mkdir(parents=True, exist_ok=False)
    master, slave = pty.openpty()
    tty.setraw(slave)
    os.symlink(os.ttyname(slave), "/dev/rikiimu")
    os.close(slave)
    stop = threading.Event()
    errors = []
    sent = []
    log_path = directory / "imu_node.log"
    result = {"status": "FAILED", "expected": expected,
              "scope": "artificial executor delay, not historical root-cause evidence"}

    def feed():
        deadline = time.monotonic()
        payload = b"".join(packet(kind, 94 if kind == 0x52 else 0)
                           for kind in (0x51, 0x52, 0x53))
        try:
            while not stop.is_set():
                assert os.write(master, payload) == len(payload)
                sent.append(time.monotonic())
                deadline += 0.005
                stop.wait(max(0, deadline - time.monotonic()))
        except Exception as error:
            errors.append(str(error))

    feeder = threading.Thread(target=feed)
    process = None
    with log_path.open("w") as output:
        try:
            environment = dict(os.environ, LD_PRELOAD=injection)
            process = subprocess.Popen([binary], env=environment, stdout=output,
                                       stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 5
            while "IMU started:" not in log_path.read_text():
                assert process.poll() is None, "node exited before startup"
                assert time.monotonic() < deadline, "startup deadline"
                time.sleep(0.001)
            feeder.start()
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline and process.poll() is None:
                time.sleep(0.005)
            log = log_path.read_text()
            match = re.search(r"delay end imu_publications=(\d+)", log)
            assert match, "executor delay was not injected"
            published = int(match.group(1))
            assert not errors, errors
            if expected == "timer":
                assert process.poll() == 1
                assert "IMU serial backlog exceeds 1024 bytes" in log
                assert published == 0
            else:
                assert process.poll() is None, "isolated collector failed"
                assert published >= 20, "no sustained publication during executor delay"
                process.send_signal(signal.SIGTERM)
                assert process.wait(timeout=5) == 0
            assert owners("/dev/rikiimu") == []
            result.update(status="PASS", publications_during_delay=published,
                          exit_code=process.returncode, sent_frames=len(sent))
        finally:
            stop.set()
            if feeder.ident is not None:
                feeder.join()
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGTERM)
                process.wait(timeout=5)
            result["remaining_owners"] = owners("/dev/rikiimu")
            (directory / "result.json").write_text(json.dumps(result, indent=2) + "\n")
            os.close(master)
            os.unlink("/dev/rikiimu")
    print(json.dumps(result))


if __name__ == "__main__":
    main()

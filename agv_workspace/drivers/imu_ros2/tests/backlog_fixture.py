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
    result_directory = Path(sys.argv[1])
    imu_node = sys.argv[2]
    result_directory.mkdir(parents=True, exist_ok=False)
    assert Path("/.dockerenv").exists()
    assert not os.path.lexists("/dev/rikiimu")
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    tty.setraw(slave)
    os.symlink(os.ttyname(slave), "/dev/rikiimu")
    os.close(slave)
    stop = threading.Event()
    feed_errors = []
    node_log = (result_directory / "imu_node.log").open("w")
    process = None
    outcome = {"status": "FAILED", "hardware": "isolated container PTY only"}

    def feed():
        try:
            while not stop.wait(0.01):
                os.write(master, packet(0x52, 94))
        except Exception as error:
            feed_errors.append(str(error))

    feeder = threading.Thread(target=feed)
    feeder.start()
    try:
        process = subprocess.Popen([imu_node], stdout=node_log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        deadline = time.monotonic() + 10.0
        while owners("/dev/rikiimu") != [process.pid]:
            assert process.poll() is None, "node exited before serial ownership"
            assert time.monotonic() < deadline, "serial ownership not established"
            time.sleep(0.01)
        time.sleep(1.5)
        assert not feed_errors, feed_errors
        assert process.poll() is None
        stop.set()
        feeder.join(timeout=2)
        os.kill(process.pid, signal.SIGSTOP)
        stopped_pid, stopped_status = os.waitpid(process.pid, os.WUNTRACED)
        assert stopped_pid == process.pid and os.WIFSTOPPED(stopped_status)
        injected = packet(0x52, 94) * 192
        written = os.write(master, injected)
        assert written == len(injected) == 2112
        time.sleep(0.02)
        os.kill(process.pid, signal.SIGCONT)
        process.wait(timeout=5)
        assert process.returncode == 1
        node_log.flush()
        log_text = (result_directory / "imu_node.log").read_text()
        assert "IMU serial backlog exceeds 1024 bytes" in log_text
        assert "IMU gyro timeout (200 ms)" not in log_text
        failed = [line for line in log_text.splitlines() if "IMU failed attempt stage=" in line]
        assert len(failed) == 1
        fields = dict(re.findall(r"([a-z_]+)=([^ ]+)", failed[0]))
        assert fields["stage"] == "queue_limit"
        assert int(fields["queued"]) >= written
        assert fields["read_attempted"] == "0"
        journal = [line for line in log_text.splitlines() if "IMU batch callback_enter=" in line]
        assert len(journal) == 128
        callback_times = [float(re.search(r"callback_enter=([0-9.]+)", line).group(1)) for line in journal]
        assert all(after > before for before, after in zip(callback_times, callback_times[1:]))
        assert callback_times[-1] < float(fields["callback_enter"])
        assert owners("/dev/rikiimu") == []
        outcome.update(status="PASS", injected_bytes=written, failure_fields=fields,
                       journal_entries=len(journal), exit_code=process.returncode,
                       injection="SIGSTOP, 2112 PTY bytes, SIGCONT once; not a vehicle root-cause finding")
    finally:
        stop.set()
        feeder.join(timeout=2)
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGCONT)
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=10)
        node_log.close()
        outcome["remaining_owners"] = owners("/dev/rikiimu")
        (result_directory / "result.json").write_text(json.dumps(outcome, indent=2) + "\n")
        os.close(master)
        Path("/dev/rikiimu").unlink()
    print(json.dumps(outcome, indent=2))


if __name__ == "__main__":
    main()

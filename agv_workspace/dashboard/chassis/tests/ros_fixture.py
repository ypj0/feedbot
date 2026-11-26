import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from urllib.error import URLError
from urllib.request import Request, urlopen

sys.path.insert(0, "/work/agv_workspace/drivers/chassis_ros2/tests")
from controller_fixture import MCU


def main():
    root = Path(sys.argv[1])
    root.mkdir(parents=True, exist_ok=False)
    token = Path(os.environ["FEEDBOT_TEST_TOKEN_FILE"]).read_text().strip()
    mcu = MCU()
    outcome = {"status": "FAILED", "hardware": "synthetic PTY only", "checks": []}
    log = (root / "process.log").open("w")
    process = subprocess.Popen(["bash", "/web/run_car.sh", "/web/factory_covariance.yaml"], stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True)

    def request(path, payload=None):
        data = json.dumps(payload).encode() if payload is not None else None
        message = Request("http://10.7.5.88:8098" + path, data=data,
                          headers={"X-Test-Token": token, "Content-Type": "application/json"})
        with urlopen(message, timeout=1) as response:
            return json.load(response)

    try:
        deadline = time.monotonic() + 20
        state = None
        while time.monotonic() < deadline:
            try:
                state = request("/api/state")
                if state["ready"]:
                    break
            except URLError:
                pass
            if process.poll() is not None:
                raise AssertionError("stack exited during startup")
            time.sleep(0.1)
        assert state and state["ready"], "stack readiness deadline"
        assert all(command == (0, 0) for command in mcu.requests)
        outcome["checks"].append("startup sends no motion")
        for sequence, (action, expected) in enumerate([
                ("forward", (50, 0)), ("backward", (-50, 0)),
                ("left", (0, 100)), ("right", (0, -100))], 1):
            state = request("/api/state")
            before = len(mcu.requests)
            request("/api/command", {"epoch": state["control"]["epoch"], "sequence": sequence,
                                     "issued_at": state["sampled_at"], "action": action})
            time.sleep(0.8)
            commands = mcu.requests[before:]
            assert expected in commands, (action, commands)
            assert commands[-1] == (0, 0)
            assert sum(command != (0, 0) for command in commands) <= 30
        outcome["checks"].append("all HTTP directions traverse diff_drive/plugin/MCU encoder and expire")
        state = request("/api/state")
        request("/api/command", {"epoch": state["control"]["epoch"], "sequence": 5,
                                 "issued_at": state["sampled_at"], "action": "forward"})
        request("/api/command", {"epoch": state["control"]["epoch"], "sequence": 6,
                                 "issued_at": state["sampled_at"], "action": "stop"})
        time.sleep(0.15)
        assert mcu.requests[-1] == (0, 0)
        mcu.increment = (100, 100)
        time.sleep(0.4)
        state = request("/api/state")
        assert state["samples"]["joints"]["value"]["left_wheel_joint"] > 0
        assert state["samples"]["odom"]["value"]["vx"] > 0
        assert state["control"]["action"] == "stop"
        outcome["checks"].append("explicit stop; real ROS joint/odom feedback visible over HTTP")
        mcu.increment = (0, 0)
        for sequence, duration in ((7, 5), (8, 10)):
            state = request("/api/state")
            before = len(mcu.requests)
            accepted = request("/api/command", {"epoch": state["control"]["epoch"], "sequence": sequence,
                                                "issued_at": state["sampled_at"], "action": "forward",
                                                "duration_s": duration})
            assert accepted["control"]["pulse_s"] == duration
            time.sleep(1)
            assert request("/api/state")["control"]["action"] == "forward"
            assert mcu.requests[-1] == (50, 0)
            time.sleep(duration - 0.6)
            assert request("/api/state")["control"]["action"] == "stop"
            assert mcu.requests[-1] == (0, 0)
            assert (50, 0) in mcu.requests[before:]
        outcome["checks"].append("5/10 second calibration requests reach MCU and expire to zero")
        state = request("/api/state")
        request("/api/command", {"epoch": state["control"]["epoch"], "sequence": 9,
                                 "issued_at": state["sampled_at"], "action": "left", "duration_s": 10})
        time.sleep(0.2)
        assert mcu.requests[-1] == (0, 100)
        request("/api/command", {"epoch": state["control"]["epoch"], "sequence": 10,
                                 "action": "stop"})
        time.sleep(0.2)
        assert mcu.requests[-1] == (0, 0)
        outcome["checks"].append("manual stop interrupts extended pulse")
        state = request("/api/state")
        outcome["state"] = state
        outcome["status"] = "PASS"
    finally:
        process.send_signal(signal.SIGTERM)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        log.close()
        outcome["remaining_owners"] = mcu.owners()
        outcome["simulator_error"] = mcu.error
        outcome["shutdown_returncode"] = process.returncode
        if outcome["remaining_owners"] or outcome["simulator_error"] is not None or process.returncode != 0:
            outcome["status"] = "FAILED"
        (root / "result.json").write_text(json.dumps(outcome, indent=2) + "\n")
        (root / "requests.json").write_text(json.dumps(mcu.requests) + "\n")
        mcu.close()
    assert outcome["remaining_owners"] == [] and outcome["simulator_error"] is None and process.returncode == 0
    print(json.dumps(outcome, indent=2))


if __name__ == "__main__":
    main()

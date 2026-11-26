import json
import os
from pathlib import Path
import re
import signal
import sys
import time

import real_lidar_trial as base


ROOT = Path("/var/tmp/feedbot-ltr2-comparison-20261122-01")
HERE = Path(__file__).resolve().parent
OFFICIAL_UNIT = "feedbot-ltr2-comparison-official.service"
base.EVIDENCE = ROOT
base.OWNER_DIRECTORY = Path("/run/feedbot-ltr2-comparison-20261122-01")
base.NATIVE_UNIT = "feedbot-ltr2-comparison-native.service"
base.BINARY = HERE / "managed_scan.sh"


def verdict(data_passed, restored):
    if not restored:
        return "对照未完成，恢复异常"
    return "实车静态通过（雷达对照）" if data_passed else "对照未完成，原厂已恢复"


def lidar_owners(store):
    return [line for line in base.socket_owners(store)
            if "2105" in [field.rsplit(":", 1)[-1] for field in line.split()[3:5]]]


def ownership(store, name, expected):
    lines = lidar_owners(store)
    pids = set(int(value) for line in lines for value in re.findall(r"pid=(\d+)", line))
    executables = {str(pid): os.readlink("/proc/%d/exe" % pid) for pid in pids}
    store.write_json(name + ".json", {"sockets": lines, "executables": executables})
    if expected is None:
        if lines:
            raise RuntimeError("unexpected lidar owner: " + " | ".join(lines))
    elif len(pids) != 1 or set(executables.values()) != {expected} or len(lines) < 2:
        raise RuntimeError("TCP/UDP owner does not match " + expected)


def probe(store, mode):
    return base.command(store, ["/bin/bash", "-c", "source /opt/ros/melodic/setup.bash; exec /usr/bin/python2 \"$@\"",
                               "_", str(HERE / "comparison_probe.py"), mode, str(ROOT)],
                        55, env=base.ros_environment(mode != "factory"))


class ComparisonHooks:
    def __init__(self, original, store):
        self.original = original
        self.store = store
        self.inhibit = original.inhibit
        self.wait_free = original.wait_free
        self.transition = original.transition
        self.desired_awake = original.desired_awake
        self.native_healthy = original.native_healthy
        self.factory_on = original.factory_on
        self.started = 0

    def factory_off(self):
        self.original.factory_off()
        for index in range(4):
            if index:
                time.sleep(10)
            ownership(self.store, "stopped_ports_%d" % index, None)
            output = base.command(self.store, ["/bin/ps", "-eo", "pid,ppid,args"], 10)
            self.store.write_log("stopped_processes_%d.log" % index, output)
            if any("/lib/ltme_node/ltme_node" in line or "mapserver_manager.py" in line for line in output.splitlines()):
                raise RuntimeError("factory driver/manager survived or restarted after spirit stop")

    def native_start(self, token):
        self.original.native_start(token)
        probe(self.store, "official")
        self.stop_official()
        ownership(self.store, "official_released_ports", None)

    def stop_official(self):
        state = base.systemd_state(self.store, OFFICIAL_UNIT)
        if state.get("LoadState") != "not-found":
            base.command(self.store, ["/bin/systemctl", "stop", OFFICIAL_UNIT], 25)

    def native_stop(self):
        self.stop_official()
        self.original.native_stop()

    def factory_healthy(self):
        self.original.factory_healthy()
        base.run_probe(self.store, "restored_chassis", False, ["preflight", "--timeout", "12"], 20)
        probe(self.store, "factory")
        output = base.command(self.store, ["/bin/bash", "-c", "source /opt/ros/melodic/setup.bash; timeout -s INT 6 rostopic hz /scan"],
                              10, accepted=(0, 124), env=base.ros_environment(False))
        self.store.write_log("restored_rostopic_hz.log", output)
        if "average rate:" not in output:
            raise RuntimeError("rostopic hz did not report a scan rate")
        ownership(self.store, "restored_ports", "/opt/ros/melodic/lib/ltme_node/ltme_node")


def run():
    store = base.EvidenceStore(ROOT)
    store.claim_run()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, base.signal_interrupted)
    module = base.load_supervisor_module()
    original = base.build_hooks(module, store)
    hooks = ComparisonHooks(original, store)
    supervisor = module.Supervisor(str(base.OWNER_DIRECTORY), hooks)
    report = {"passed": False, "factory_restored": False, "absolute_time_alignment_verified": False,
              "clock_modified": False, "motion_commands_sent": False}
    data_passed = False
    try:
        ownership(store, "initial_ports", "/opt/ros/melodic/lib/ltme_node/ltme_node")
        store.write_log("initial_services.log", base.command(store, ["/bin/systemctl", "list-units", "--type=service", "--all", "--no-pager"], 15))
        supervisor.start()
        ownership(store, "managed_ports", "/opt/feedbot-ltr2/releases/20261122-managed-v3/build/ltr2_managed_ros1")
        probe(store, "managed")
        original.transition("cleanup")
        data_passed = True
    except BaseException as error:
        report["error"] = "%s: %s" % (type(error).__name__, error)
    finally:
        cleanup_errors = original.shutdown_lifecycle()
        if cleanup_errors:
            report["cleanup_errors"] = cleanup_errors
            data_passed = False
        try:
            if supervisor.needs_restore:
                transaction = json.loads((base.OWNER_DIRECTORY / "transaction.json").read_text())
                if transaction["state"] == "rollback_failed":
                    raise RuntimeError("prior restoration failed: " + str(transaction["error"]))
                supervisor.restore()
            report["factory_restored"] = not supervisor.needs_restore
        except BaseException as error:
            report["restore_error"] = "%s: %s" % (type(error).__name__, error)
        supervisor.close()
        report["data_checks_passed"] = data_passed
        report["status"] = verdict(False, report["factory_restored"])
        report["runtime_checks_passed"] = data_passed and report["factory_restored"]
        report["beam_index_capture_verification"] = "pending offline replay of captured packets"
        store.write_json("trial.json", report)
    return 0 if report["runtime_checks_passed"] else 1


if __name__ == "__main__":
    sys.exit(run())

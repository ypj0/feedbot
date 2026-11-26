import json
import os
from pathlib import Path
import re
import signal
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "validation"))
import real_lidar_trial as base


RELEASE = Path("/opt/feedbot-ltr2/releases/20261122-ros2-v1")
ROOT = Path("/var/tmp/feedbot-ltr2-ros2-20261122-05")
CONTAINER = "feedbot-ltr2-ros2-trial-05"
DOCKER = ["/opt/feedbot-nav2/docker/docker", "-H", "unix:///run/feedbot-nav2-docker.sock"]
base.OWNER_DIRECTORY = Path("/run/feedbot-ltr2-ros2-20261122-05")
base.NATIVE_UNIT = "feedbot-ltr2-ros2-unused.service"


def ports(store, name, expected):
    lines = base.socket_owners(store)
    pids = set(int(value) for line in lines for value in re.findall(r"pid=(\d+)", line))
    executables = {str(pid): os.readlink("/proc/%d/exe" % pid) for pid in pids}
    store.write_json(name + ".json", {"sockets": lines, "executables": executables})
    if expected == "free":
        if lines:
            raise RuntimeError("ports not released: " + " | ".join(lines))
    elif expected == "udp_free":
        if any(line.split()[0] == "UNCONN" and line.split()[3].endswith(":2105") for line in lines):
            raise RuntimeError("UDP 2105 not released")
    else:
        if len(pids) != 1 or len(lines) != 2 or any(not value.endswith(expected) for value in executables.values()):
            raise RuntimeError("TCP/UDP owner mismatch: " + str(executables))


class Hooks:
    def __init__(self, original, store):
        self.original = original
        self.store = store
        self.inhibit = original.inhibit
        self.wait_free = original.wait_free
        self.factory_on = original.factory_on
        self.desired_awake = original.desired_awake
        self.started_container = False
        self.state = "unconfigured"
        self.index = 0

    def factory_off(self):
        self.original.factory_off()
        for index in range(4):
            if index:
                time.sleep(10)
            ports(self.store, "stopped_ports_%d" % index, "free")
            processes = base.command(self.store, ["/bin/ps", "-eo", "pid,ppid,args"], 10)
            self.store.write_log("stopped_processes_%d.log" % index, processes)
            if any("/lib/ltme_node/ltme_node" in line or "mapserver_manager.py" in line for line in processes.splitlines()):
                raise RuntimeError("factory lidar/manager survived or restarted")

    def native_start(self, token):
        argv = DOCKER + ["run", "-d", "--name", CONTAINER, "--network", "host",
                        "-e", "ROS_DOMAIN_ID=89", "-e", "RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
                        "-e", "ROS_LOCALHOST_ONLY=1", "-v", str(RELEASE) + ":/work:ro",
                        "-v", str(ROOT) + ":/results",
                        "-v", str(base.OWNER_DIRECTORY) + ":/run/feedbot-lidar",
                        "--entrypoint", "/bin/bash", "feedbot-navigation:jazzy-20261122",
                        "-c", "source /opt/ros/jazzy/setup.bash; exec /work/build/ltr2_ros2 --ros-args -p owner_token:=\"$1\"",
                        "_", token]
        self.original.started = time.monotonic()
        base.command(self.store, argv, 20, logged_argv=argv[:-1] + ["<owner-token-redacted>"])
        self.started_container = True

    def execute(self, script, timeout=20, accepted=(0,)):
        return base.command(self.store, DOCKER + ["exec", CONTAINER, "/bin/bash", "-c",
                            "source /opt/ros/jazzy/setup.bash; " + script], timeout, accepted=accepted)

    def transition(self, name):
        self.index += 1
        self.execute("python3 /work/source/ltr2/ros2/tests/probe.py %s /results/%02d_%s.json" %
                     (name, self.index, name))
        self.state = {"configure": "inactive", "activate": "active", "deactivate": "inactive", "cleanup": "unconfigured"}[name]

    def native_healthy(self, awake):
        self.original.native_healthy(awake)

    def collect(self, label, silent=False):
        self.execute("python3 /work/source/ltr2/ros2/tests/probe.py %s /results/%s.json" %
                     ("silence" if silent else "collect", label))

    def native_stop(self):
        if not self.started_container:
            return
        base.command(self.store, DOCKER + ["stop", "-t", "15", CONTAINER], 20)
        self.store.write_log("driver.log", base.command(self.store, DOCKER + ["logs", CONTAINER], 10))
        state = json.loads(base.command(self.store, DOCKER + ["inspect", "--format", "{{json .State}}", CONTAINER], 10))
        self.store.write_json("container_state.json", state)
        if state["Running"]:
            raise RuntimeError("trial container still running")

    def factory_healthy(self):
        self.original.factory_healthy()
        base.run_probe(self.store, "restored_chassis", False, ["preflight", "--timeout", "12"], 20)
        base.command(self.store, ["/bin/bash", "-c",
                     "source /opt/ros/melodic/setup.bash; exec /usr/bin/python2 \"$@\"", "_",
                     str(Path(__file__).resolve().parents[2] / "validation/comparison_probe.py"),
                     "factory", str(ROOT)], 15, env=base.ros_environment(False))
        output = base.command(self.store, ["/bin/bash", "-c",
                              "source /opt/ros/melodic/setup.bash; timeout -s INT 6 rostopic hz /scan"],
                              10, accepted=(0, 124), env=base.ros_environment(False))
        self.store.write_log("restored_hz.log", output)
        if "average rate:" not in output:
            raise RuntimeError("no restored rostopic hz output")
        ports(self.store, "restored_ports", "/opt/ros/melodic/lib/ltme_node/ltme_node")


def run():
    store = base.EvidenceStore(ROOT)
    store.claim_run()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, base.signal_interrupted)
    module = base.load_supervisor_module()
    original = base.build_hooks(module, store)
    hooks = Hooks(original, store)
    supervisor = module.Supervisor(str(base.OWNER_DIRECTORY), hooks)
    report = {"passed": False, "factory_restored": False, "absolute_time_alignment_verified": False,
              "motion_commands_sent": False, "clock_modified": False}
    try:
        ports(store, "initial_ports", "/opt/ros/melodic/lib/ltme_node/ltme_node")
        supervisor.start()
        hooks.collect("configured_silence", True)
        for label in ("first", "restarted"):
            hooks.transition("activate")
            ports(store, label + "_active_ports", "/ltr2_ros2")
            output = hooks.execute("timeout -s INT 6 ros2 topic hz /scan", 10, accepted=(0, 124))
            store.write_log(label + "_topic_hz.log", output)
            store.write_log(label + "_diagnostics.log", hooks.execute("timeout 8 ros2 topic echo /diagnostics diagnostic_msgs/msg/DiagnosticArray --once", 12))
            store.write_log(label + "_topic_info.log", hooks.execute("ros2 topic info /scan -v --no-daemon --spin-time 3", 8))
            hooks.collect(label + "_scans")
            rates = [float(value) for value in re.findall(r"average rate: ([0-9.]+)", output)]
            if not rates or not 14 <= rates[-1] <= 16:
                raise RuntimeError("ros2 topic hz outside 14-16 Hz")
            store.write_log(label + "_topic_echo.log", hooks.execute("timeout 8 ros2 topic echo /scan sensor_msgs/msg/LaserScan --once --full-length", 12))
            verification = hooks.execute("python3 - <<'PY'\n"
                "import json,yaml\n"
                "message=list(yaml.safe_load_all(open('/results/%s_topic_echo.log')))[0]\n"
                "count=len(message['ranges'])\n"
                "assert count==len(message['intensities'])==1075\n"
                "error=message['angle_max']-message['angle_min']-(count-1)*message['angle_increment']\n"
                "assert abs(error)<1e-6\n"
                "print(json.dumps({'passed':True,'count':count,'geometry_error_rad':error}))\nPY" % label)
            store.write_json(label + "_echo_verification.json", json.loads(verification))
            hooks.transition("deactivate")
            ports(store, label + "_stopped_ports", "udp_free")
            hooks.collect(label + "_stopped_silence", True)
        hooks.transition("cleanup")
        ports(store, "cleaned_ports", "free")
        report["data_checks_passed"] = True
    except BaseException as error:
        report["error"] = "%s: %s" % (type(error).__name__, error)
    finally:
        try:
            if hooks.state == "active":
                hooks.transition("deactivate")
            if hooks.state != "unconfigured":
                hooks.transition("cleanup")
        except Exception as error:
            report["cleanup_error"] = str(error)
        try:
            if supervisor.needs_restore:
                transaction = json.loads((base.OWNER_DIRECTORY / "transaction.json").read_text())
                if transaction["state"] == "rollback_failed":
                    raise RuntimeError("prior restoration failed: " + str(transaction["error"]))
                supervisor.restore()
            report["factory_restored"] = not supervisor.needs_restore
        except BaseException as error:
            report["restore_error"] = str(error)
        supervisor.close()
        report["passed"] = bool(report.get("data_checks_passed") and report["factory_restored"] and
                                "cleanup_error" not in report and "error" not in report)
        report["status"] = "实车静态通过（ROS2 雷达）" if report["passed"] else (
            "未通过，原厂已恢复" if report["factory_restored"] else "未通过，恢复异常")
        store.write_json("acceptance.json", report)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(run())

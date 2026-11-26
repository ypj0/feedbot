#!/usr/bin/env python3
import importlib.util
import json
import os
from pathlib import Path
import re
import signal
import sys


ROOT = Path(__file__).resolve().parents[3]
LIDAR_RELEASE = Path("/opt/feedbot-ltr2/releases/20261122-ros2-v1")
OWNER_DIRECTORY = Path("/run/feedbot-mapping-static")
CONTAINER = "feedbot-base"
DOCKER = ["/opt/feedbot-nav2/docker/docker", "-H", "unix:///run/feedbot-nav2-docker.sock"]
DDS_ENVIRONMENT = [
    "-e", "ROS_DOMAIN_ID=89",
    "-e", "ROS_LOCALHOST_ONLY=0",
    "-e", "ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT",
    "-e", "RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
    "-e", "RMW_FASTRTPS_PUBLICATION_MODE=SYNCHRONOUS",
    "-e", "FASTRTPS_DEFAULT_PROFILES_FILE=/work/agv_workspace/integration/mapping/config/dds_loopback.xml",
]
FACTORY_SERIAL_EXECUTABLES = {
    "/dev/rikibase": "/opt/ros/melodic/lib/robot_driver/robot_driver",
    "/dev/rikiimu": "/opt/ros/melodic/lib/wit_ros_imu/imu_node",
}


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, str(path))
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load " + str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def socket_pids(lines):
    return sorted(set(int(value) for line in lines for value in re.findall(r"pid=(\d+)", line)))


def require_factory_serial_owner(device, executable):
    if executable != FACTORY_SERIAL_EXECUTABLES[device]:
        raise RuntimeError("unexpected factory serial owner for %s: %s" % (device, executable))


class Hooks:
    root = ROOT
    container = CONTAINER
    stack_script = "/work/agv_workspace/integration/bringup/stack.sh"

    def __init__(self, base, original, store, results):
        self.base = base
        self.original = original
        self.store = store
        self.results = results
        self.state = "unconfigured"
        self.started = False
        self.native_stop_error = None
        self.transition_index = 0

    def inhibit(self):
        self.original.inhibit()
        system_source = self.root / "agv_workspace/drivers/chassis_ros2/src/system.cpp"
        lines = system_source.read_text().splitlines()
        self.store.write_log("chassis_activation_zero_path.log", "\n".join(lines) + "\n")
        output = self.base.command(self.store, DOCKER + ["ps", "-a", "--filter",
                                   "name=^/%s$" % self.container, "--format", "{{.Names}}"], 10)
        if output.strip():
            raise RuntimeError("static container already exists")

    def factory_off(self):
        self.original.factory_off()
        processes = self.base.command(self.store, ["/bin/ps", "-eo", "pid,ppid,args"], 10)
        if "/lib/ltme_node/ltme_node" in processes or "mapserver_manager.py" in processes:
            raise RuntimeError("factory lidar process survived stop")

    def factory_on(self):
        self.original.factory_on()

    def wait_free(self):
        self.original.wait_free()

    def factory_healthy(self):
        self.original.factory_healthy()
        self.base.run_probe(self.store, "restored_chassis", False,
                            ["preflight", "--timeout", "12"], 20)
        for topic, minimum, maximum in (("/scan", 14.0, 16.0), ("/wheel_odom", 45.0, 55.0),
                                        ("/driver_info", 45.0, 55.0)):
            output = self.base.command(
                self.store,
                ["/bin/bash", "-c", "source /opt/ros/melodic/setup.bash; "
                 "exec timeout -s INT 8 rostopic hz " + topic],
                12, accepted=(0, 124), env=self.base.ros_environment(False))
            self.store.write_log("restored_%s_hz.log" % topic.strip("/").replace("/", "_"), output)
            rates = [float(value) for value in re.findall(r"average rate: ([0-9.]+)", output)]
            if not rates or not minimum <= rates[-1] <= maximum:
                raise RuntimeError("restored rate outside bounds for " + topic)
        serial = {}
        for device in ("/dev/rikibase", "/dev/rikiimu"):
            output = self.base.command(self.store, ["/bin/fuser", "-v", device], 10)
            owners = [int(value) for value in output.split() if value.isdigit()]
            if len(owners) != 1:
                raise RuntimeError("restored serial owner mismatch: " + device)
            serial[device] = {"pid": owners[0], "executable": os.readlink("/proc/%d/exe" % owners[0])}
            require_factory_serial_owner(device, serial[device]["executable"])
        self.store.write_json("restored_serial_owners.json", serial)
        lines = self.base.socket_owners(self.store)
        owners = socket_pids(lines)
        if len(owners) != 1 or len(lines) != 2:
            raise RuntimeError("factory lidar socket ownership not restored")
        executable = os.readlink("/proc/%d/exe" % owners[0])
        if executable != "/opt/ros/melodic/lib/ltme_node/ltme_node":
            raise RuntimeError("unexpected restored lidar owner: " + executable)
        self.store.write_json("restored_socket_owner.json", {"lines": lines, "executable": executable})

    def native_start(self, token):
        argv = DOCKER + [
            "run", "-d", "--name", self.container, "--network", "host", "--ipc", "host",
            "--pid", "host", "--cap-add", "SYS_PTRACE",
            "--ulimit", "rtprio=50",
            "--ulimit", "nice=30:30",
            "--security-opt", "apparmor=feedbot-serial-audit",
            "--device", "/dev/rikibase:/dev/rikibase", "--device", "/dev/rikiimu:/dev/rikiimu",
            *DDS_ENVIRONMENT,
            "-v", str(self.root) + ":/work:ro", "-v", str(LIDAR_RELEASE) + ":/ltr2:ro",
            "-v", str(self.results) + ":/results:rw",
            "-v", str(OWNER_DIRECTORY) + ":/run/feedbot-lidar:rw",
            "--entrypoint", "/bin/bash", "feedbot-navigation:jazzy-20261122",
            self.stack_script, token,
        ]
        logged = list(argv)
        logged[-1] = "<owner-token-redacted>"
        self.base.command(self.store, argv, 20, logged_argv=logged)
        self.started = True

    def execute(self, script, timeout):
        return self.base.command(self.store, DOCKER + ["exec", self.container, "/bin/bash", "-c",
                                                       "source /opt/ros/jazzy/setup.bash; " + script], timeout)

    def transition(self, name):
        self.transition_index += 1
        destination = "/results/%02d_lidar_%s.json" % (self.transition_index, name)
        self.execute("python3 /ltr2/source/ltr2/ros2/tests/probe.py %s %s" % (name, destination), 25)
        self.state = {"configure": "inactive", "activate": "active",
                      "deactivate": "inactive", "cleanup": "unconfigured"}[name]

    def desired_awake(self):
        return True

    def native_healthy(self, awake):
        if not awake or self.state != "active":
            raise RuntimeError("static session requires active lidar")
        state = json.loads(self.base.command(self.store, DOCKER + ["inspect", "--format",
                                                                  "{{json .State}}", self.container], 10))
        if not state.get("Running"):
            raise RuntimeError("static stack container is not running")
        lines = self.base.socket_owners(self.store)
        owners = socket_pids(lines)
        if len(owners) != 1 or len(lines) != 2:
            raise RuntimeError("native lidar socket ownership mismatch")
        executable = os.readlink("/proc/%d/exe" % owners[0])
        if executable != "/ltr2/build/ltr2_ros2":
            raise RuntimeError("unexpected native lidar owner: " + executable)
        self.store.write_json("native_socket_owner.json", {"lines": lines, "executable": executable})

    def shutdown_lifecycle(self):
        errors = []
        if not self.started:
            return errors
        if self.state == "active":
            try:
                self.transition("deactivate")
            except Exception as error:
                errors.append("deactivate: %s" % error)
        if self.state != "unconfigured":
            try:
                self.transition("cleanup")
            except Exception as error:
                errors.append("cleanup: %s" % error)
        return errors

    def native_stop(self):
        if not self.started:
            return
        self.base.command(self.store, DOCKER + ["stop", "-t", "20", self.container], 30)
        self.store.write_log("stack.log", self.base.command(self.store, DOCKER + ["logs", self.container], 10))
        state = json.loads(self.base.command(self.store, DOCKER + ["inspect", "--format",
                                                                  "{{json .State}}", self.container], 10))
        self.store.write_json("container_state.json", state)
        if state.get("Running"):
            raise RuntimeError("static container remained running")
        if state.get("ExitCode") != 0:
            self.native_stop_error = "static container exited with %s" % state.get("ExitCode")
        self.base.command(self.store, DOCKER + ["rm", self.container], 10)
        self.started = False

#!/usr/bin/env python3
"""Bounded real LT-R2-B isolated trial using the archived lidar supervisor."""

import argparse
import hashlib
import importlib.util
import json
import os
import pathlib
import signal
import subprocess
import sys
import time


RELEASE = pathlib.Path("/opt/feedbot-ltr2/releases/20261122-managed-v3")
BINARY = RELEASE / "build/ltr2_managed_ros1"
SUPERVISOR_PATH = pathlib.Path("/opt/feedbot-nav2/lidar/supervisor.py")
PROBE = pathlib.Path(__file__).with_name("ros_probe.py")
PYTHON2 = "/usr/bin/python2"
ROS_SETUP = "/opt/ros/melodic/setup.bash"
EVIDENCE = pathlib.Path("/var/tmp/feedbot-ltr2-real-20261122-01")
OWNER_DIRECTORY = pathlib.Path("/run/feedbot-ltr2-real-20261122-01")
SPIRIT_UNIT = "spirit.service"
TRIAL_UNIT = "feedbot-ltr2-real-trial.service"
NATIVE_UNIT = "feedbot-ltr2-real-native.service"
RESTORE_UNIT = "feedbot-ltr2-real-restore.service"
RESTORE_TIMER = "feedbot-ltr2-real-restore.timer"
PRODUCTION_MASTER = "http://10.7.5.88:11311"
PRIVATE_PORT = 11372
PRIVATE_MASTER = "http://127.0.0.1:%d" % PRIVATE_PORT
LIDAR_PORT = 2105


class TrialInterrupted(RuntimeError):
    pass


def utc_now():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def load_supervisor_module():
    spec = importlib.util.spec_from_file_location("feedbot_lidar_supervisor", str(SUPERVISOR_PATH))
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load archived lidar supervisor")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class EvidenceStore:
    def __init__(self, root):
        self.root = pathlib.Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.command_log = self.root / "commands.log"

    def claim_run(self):
        path = self.root / "run_claim.json"
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        descriptor = os.open(str(path), flags, 0o600)
        with os.fdopen(descriptor, "w") as output:
            json.dump({"claimed_at": utc_now(), "pid": os.getpid()}, output, sort_keys=True)
            output.write("\n")

    def write_json(self, name, value):
        path = self.root / name
        temporary = path.with_name(path.name + ".tmp.%d" % os.getpid())
        with temporary.open("w") as output:
            json.dump(value, output, indent=2, sort_keys=True, allow_nan=False)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        os.replace(str(temporary), str(path))

    def write_log(self, name, value):
        path = self.root / name
        with path.open("w") as output:
            output.write(value)

    def log_command(self, argv, returncode, stdout, stderr):
        record = {
            "argv": list(argv),
            "finished_at": utc_now(),
            "returncode": returncode,
            "stderr": stderr,
            "stdout": stdout,
        }
        with self.command_log.open("a") as output:
            output.write(json.dumps(record, sort_keys=True) + "\n")


def output_text(value):
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", "replace")
    return value


def command(store, argv, timeout, accepted=(0,), env=None, logged_argv=None):
    try:
        result = subprocess.run(
            argv,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            timeout=timeout,
            env=env,
        )
    except subprocess.TimeoutExpired as error:
        stdout = output_text(getattr(error, "stdout", None) or getattr(error, "output", None))
        stderr = output_text(getattr(error, "stderr", None))
        store.log_command(logged_argv or argv, "timeout", stdout, stderr)
        raise RuntimeError("command timed out after %ss: %s" % (timeout, argv[0]))
    store.log_command(logged_argv or argv, result.returncode, result.stdout, result.stderr)
    if result.returncode not in accepted:
        detail = (result.stderr + result.stdout)[-2000:]
        raise RuntimeError("command failed (%d): %s: %s" % (result.returncode, argv[0], detail))
    return result.stdout


def systemd_state(store, unit):
    output = command(
        store,
        ["/bin/systemctl", "show", unit, "--property=LoadState", "--property=ActiveState",
         "--property=SubState", "--property=MainPID", "--property=ControlPID"],
        10,
    )
    return dict(line.split("=", 1) for line in output.splitlines() if "=" in line)


def ros_environment(private):
    environment = os.environ.copy()
    environment["ROS_MASTER_URI"] = PRIVATE_MASTER if private else PRODUCTION_MASTER
    environment["ROS_IP"] = "127.0.0.1" if private else "10.7.5.88"
    environment.pop("ROS_HOSTNAME", None)
    return environment


def parse_probe_output(output):
    for line in reversed(output.splitlines()):
        try:
            value = json.loads(line)
        except ValueError:
            continue
        if isinstance(value, dict) and "passed" in value:
            return value
    raise RuntimeError("ROS probe produced no JSON result")


def run_probe(store, module_name, private, arguments, timeout):
    argv = [
        "/bin/bash",
        "-c",
        "source \"$1\"; shift; exec \"$@\"",
        "_",
        ROS_SETUP,
        PYTHON2,
        str(PROBE),
    ] + list(arguments)
    environment = ros_environment(private)
    try:
        result = subprocess.run(
            argv,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            timeout=timeout,
            env=environment,
        )
        raw = result.stdout + ("\nSTDERR\n" + result.stderr if result.stderr else "")
        store.write_log(module_name + ".log", raw)
        try:
            report = parse_probe_output(result.stdout)
        except Exception as error:
            report = {
                "error": "%s: %s" % (error.__class__.__name__, error),
                "mode": arguments[0] if arguments else None,
                "navigation_accepted": False,
                "passed": False,
                "real_hardware": True,
            }
            store.write_json(module_name + ".json", report)
            raise
        store.write_json(module_name + ".json", report)
        if result.returncode or not report.get("passed"):
            raise RuntimeError("ROS probe failed: %s" % module_name)
        return report
    except subprocess.TimeoutExpired as error:
        stdout = output_text(getattr(error, "stdout", None) or getattr(error, "output", None))
        stderr = output_text(getattr(error, "stderr", None))
        store.write_log(module_name + ".log", stdout + "\nTIMEOUT\n" + stderr)
        store.write_json(module_name + ".json", {
            "error": "probe subprocess timeout",
            "mode": arguments[0] if arguments else None,
            "navigation_accepted": False,
            "passed": False,
            "real_hardware": True,
        })
        raise RuntimeError("ROS probe timed out: %s" % module_name)


def socket_owners(store):
    owners = []
    for argv in (["/bin/ss", "-H", "-tanp"], ["/bin/ss", "-H", "-uanp"]):
        output = command(store, argv, 10)
        for line in output.splitlines():
            fields = line.split()
            if len(fields) < 5 or "users:((" not in line:
                continue
            local_port = fields[3].rsplit(":", 1)[-1]
            peer_port = fields[4].rsplit(":", 1)[-1]
            lidar_owner = str(LIDAR_PORT) in (local_port, peer_port)
            private_master_owner = local_port == str(PRIVATE_PORT)
            if lidar_owner or private_master_owner:
                owners.append(line)
    return owners


def require_ports_free(store, timeout=15.0):
    deadline = time.monotonic() + timeout
    last = []
    while time.monotonic() < deadline:
        last = socket_owners(store)
        if not last:
            return
        time.sleep(0.2)
    raise RuntimeError("lidar/private ROS ports still owned: %s" % " | ".join(last))


def sha256_file(path):
    digest = hashlib.sha256()
    with pathlib.Path(path).open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def build_hooks(supervisor_module, store):
    class TrialHooks(supervisor_module.Hooks):
        def __init__(self):
            supervisor_module.Hooks.__init__(self, {}, str(OWNER_DIRECTORY))
            self.transition_index = 0
            self.lifecycle_state = "unconfigured"

        def inhibit(self):
            run_probe(store, "production_stationary_preflight", False, ["preflight", "--timeout", "12"], 20)
            state = systemd_state(store, NATIVE_UNIT)
            if state.get("LoadState") != "not-found" and state.get("ActiveState") not in ("inactive", "failed"):
                raise RuntimeError("native unit already active before trial")
            owners = [line for line in socket_owners(store) if ":%d" % PRIVATE_PORT in line]
            if owners:
                raise RuntimeError("private ROS port already owned")

        def factory_off(self):
            command(store, ["/bin/systemctl", "stop", SPIRIT_UNIT], 105)
            state = systemd_state(store, SPIRIT_UNIT)
            if state.get("ActiveState") != "inactive":
                raise RuntimeError("spirit.service did not become inactive")

        def factory_on(self):
            command(store, ["/bin/systemctl", "start", SPIRIT_UNIT], 105)
            state = systemd_state(store, SPIRIT_UNIT)
            if state.get("ActiveState") != "active":
                raise RuntimeError("spirit.service did not become active")

        def factory_healthy(self):
            run_probe(store, "factory_restored", False, ["factory", "--timeout", "40", "--count", "3"], 50)

        def wait_free(self):
            require_ports_free(store)

        def native_start(self, token):
            if not BINARY.is_file() or not os.access(str(BINARY), os.X_OK):
                raise RuntimeError("fixed managed driver binary is absent or not executable")
            if not PROBE.is_file():
                raise RuntimeError("ROS probe is absent")
            state = systemd_state(store, NATIVE_UNIT)
            if state.get("LoadState") != "not-found":
                if state.get("ActiveState") not in ("inactive", "failed"):
                    raise RuntimeError("native unit is already active")
                command(store, ["/bin/systemctl", "reset-failed", NATIVE_UNIT], 10, accepted=(0, 1))
            script = (
                "set -e; source \"$1\"; unset ROS_HOSTNAME; shift; "
                "roscore -p \"$1\" >>\"$6\" 2>&1 & shift; "
                "i=0; until rosparam list >/dev/null 2>&1; do i=$((i+1)); "
                "test $i -lt 100; sleep 0.1; done; "
                "exec \"$1\" \"_owner_token:=$2\" \"_owner_directory:=$3\" >>\"$4\" 2>&1"
            )
            argv = [
                "/usr/bin/systemd-run",
                "--unit=" + NATIVE_UNIT,
                "--collect",
                "--property=Type=simple",
                "--property=Restart=no",
                "--property=KillMode=control-group",
                "--property=TimeoutStopSec=20",
                "--property=RuntimeMaxSec=180",
                "--property=NoNewPrivileges=yes",
                "--property=Environment=ROS_MASTER_URI=" + PRIVATE_MASTER,
                "--property=Environment=ROS_IP=127.0.0.1",
                "/bin/bash",
                "-c",
                script,
                "_",
                ROS_SETUP,
                str(PRIVATE_PORT),
                str(BINARY),
                token,
                str(OWNER_DIRECTORY),
                str(store.root / "managed_driver.log"),
                str(store.root / "private_roscore.log"),
            ]
            self.started = time.monotonic()
            logged_argv = list(argv)
            logged_argv[logged_argv.index(token)] = "<owner-token-redacted>"
            command(store, argv, 20, logged_argv=logged_argv)

        def native_stop(self):
            state = systemd_state(store, NATIVE_UNIT)
            if state.get("LoadState") != "not-found":
                command(store, ["/bin/systemctl", "stop", NATIVE_UNIT], 30, accepted=(0, 5))
                state = systemd_state(store, NATIVE_UNIT)
                if state.get("LoadState") != "not-found" and state.get("ActiveState") not in ("inactive", "failed"):
                    raise RuntimeError("native unit did not stop")
                if state.get("LoadState") != "not-found" and (
                        state.get("MainPID") not in (None, "0") or state.get("ControlPID") not in (None, "0")):
                    raise RuntimeError("native unit retained processes after stop")

        def transition(self, name):
            self.transition_index += 1
            module = "%02d_transition_%s" % (self.transition_index, name)
            run_probe(store, module, True, ["transition", name], 20)
            if name == "configure":
                self.lifecycle_state = "configured"
            elif name == "activate":
                self.lifecycle_state = "active"
            elif name == "deactivate":
                self.lifecycle_state = "configured"
            elif name == "cleanup":
                self.lifecycle_state = "unconfigured"

        def desired_awake(self):
            return False

        def native_healthy(self, awake):
            supervisor_module.Hooks.native_healthy(self, awake)
            store.write_json("native_configured_health.json", {
                "desired_awake": bool(awake),
                "navigation_accepted": False,
                "passed": not awake,
                "real_hardware": True,
                "status": json.loads((OWNER_DIRECTORY / "status.json").read_text()),
            })

        def silence(self, module):
            return run_probe(store, module, True, ["silence", "--duration", "1.0"], 8)

        def collect(self, module):
            return run_probe(store, module, True, ["collect", "--timeout", "15", "--count", "10"], 22)

        def shutdown_lifecycle(self):
            errors = []
            if self.lifecycle_state == "active":
                try:
                    self.transition("deactivate")
                except Exception as error:
                    errors.append("deactivate: %s" % error)
            if self.lifecycle_state != "unconfigured":
                try:
                    self.transition("cleanup")
                except Exception as error:
                    errors.append("cleanup: %s" % error)
            return errors

    return TrialHooks()


def exercise(supervisor, hooks):
    supervisor.start()
    hooks.silence("configured_no_scan")
    hooks.transition("activate")
    first = hooks.collect("first_activation_scans")
    hooks.transition("deactivate")
    hooks.silence("deactivated_no_scan")
    hooks.transition("activate")
    second = hooks.collect("second_activation_scans")
    hooks.transition("cleanup")
    return {"first_activation": first, "second_activation": second}


def signal_interrupted(signum, unused_frame):
    raise TrialInterrupted("received signal %d" % signum)


def run_trial():
    store = EvidenceStore(EVIDENCE)
    store.claim_run()
    for signum in (signal.SIGTERM, signal.SIGINT):
        signal.signal(signum, signal_interrupted)
    report = {
        "binary": str(BINARY),
        "evidence_directory": str(EVIDENCE),
        "finished_at": None,
        "navigation_accepted": False,
        "owner_directory": str(OWNER_DIRECTORY),
        "passed": False,
        "production_master": PRODUCTION_MASTER,
        "private_master": PRIVATE_MASTER,
        "real_hardware": True,
        "release": str(RELEASE),
        "scope": "real LT-R2-B lifecycle and data only; no motion commands or production /scan remap",
        "started_at": utc_now(),
        "units": {
            "native": NATIVE_UNIT,
            "restore": RESTORE_UNIT,
            "restore_timer": RESTORE_TIMER,
            "spirit": SPIRIT_UNIT,
            "trial": TRIAL_UNIT,
        },
    }
    supervisor = None
    hooks = None
    primary_error = None
    cleanup_errors = []
    restore_error = None
    try:
        for path in (BINARY, SUPERVISOR_PATH, PROBE):
            if not pathlib.Path(path).is_file():
                raise RuntimeError("required fixed file absent: %s" % path)
        report["sha256"] = {
            "binary": sha256_file(BINARY),
            "probe": sha256_file(PROBE),
            "supervisor": sha256_file(SUPERVISOR_PATH),
        }
        module = load_supervisor_module()
        hooks = build_hooks(module, store)
        supervisor = module.Supervisor(str(OWNER_DIRECTORY), hooks)
        report["modules"] = exercise(supervisor, hooks)
        report["passed"] = True
    except BaseException as error:
        primary_error = error
        report["error"] = "%s: %s" % (error.__class__.__name__, error)
    finally:
        if hooks is not None:
            cleanup_errors = hooks.shutdown_lifecycle()
        if supervisor is not None:
            try:
                if supervisor.needs_restore:
                    supervisor.restore()
            except BaseException as error:
                restore_error = error
            finally:
                supervisor.close()
        if cleanup_errors:
            report["cleanup_errors"] = cleanup_errors
            report["passed"] = False
        if restore_error is not None:
            report["restore_error"] = "%s: %s" % (restore_error.__class__.__name__, restore_error)
            report["passed"] = False
        report["factory_restored"] = supervisor is not None and not supervisor.needs_restore
        report["finished_at"] = utc_now()
        store.write_json("trial.json", report)
    if primary_error is not None or cleanup_errors or restore_error is not None:
        return 1
    return 0


def current_unit_contains(unit):
    try:
        return any(unit in line for line in pathlib.Path("/proc/self/cgroup").read_text().splitlines())
    except OSError:
        return False


def stop_trial_before_external_restore(store):
    if current_unit_contains(TRIAL_UNIT):
        return
    state = systemd_state(store, TRIAL_UNIT)
    if state.get("LoadState") != "not-found" and state.get("ActiveState") not in ("inactive", "failed"):
        command(store, ["/bin/systemctl", "stop", TRIAL_UNIT], 180, accepted=(0, 5))
    state = systemd_state(store, TRIAL_UNIT)
    if state.get("LoadState") != "not-found" and state.get("ActiveState") not in ("inactive", "failed"):
        raise RuntimeError("trial unit remained active during external restoration")


def restore_only():
    store = EvidenceStore(EVIDENCE)
    report = {
        "finished_at": None,
        "navigation_accepted": False,
        "passed": False,
        "real_hardware": True,
        "started_at": utc_now(),
    }
    supervisor = None
    try:
        stop_trial_before_external_restore(store)
        module = load_supervisor_module()
        hooks = build_hooks(module, store)
        supervisor = module.Supervisor(str(OWNER_DIRECTORY), hooks)
        transaction_path = OWNER_DIRECTORY / "transaction.json"
        transaction = json.loads(transaction_path.read_text()) if transaction_path.exists() else None
        native_state = systemd_state(store, NATIVE_UNIT)
        factory_state = systemd_state(store, SPIRIT_UNIT)
        already_factory = transaction is None or transaction.get("state") == "factory"
        native_stopped = native_state.get("LoadState") == "not-found" or (
            native_state.get("ActiveState") in ("inactive", "failed") and
            native_state.get("MainPID") in (None, "0") and
            native_state.get("ControlPID") in (None, "0")
        )
        if already_factory and native_stopped and factory_state.get("ActiveState") == "active":
            supervisor.revoke()
            hooks.factory_healthy()
            report["restoration_action"] = "verified_already_factory"
        else:
            supervisor.restore()
            report["restoration_action"] = "restored_factory"
        report["passed"] = True
        report["factory_restored"] = True
    except BaseException as error:
        report["error"] = "%s: %s" % (error.__class__.__name__, error)
    finally:
        if supervisor is not None:
            supervisor.close()
        report["finished_at"] = utc_now()
        name = "external_restore_%d_%d.json" % (int(time.time()), os.getpid())
        store.write_json(name, report)
    return 0 if report["passed"] else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("run", "restore"))
    arguments = parser.parse_args(argv)
    return run_trial() if arguments.command == "run" else restore_only()


if __name__ == "__main__":
    sys.exit(main())

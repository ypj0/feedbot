#!/usr/bin/env python3
import importlib.util
import json
from pathlib import Path
import signal
import sys


DIRECTORY = Path(__file__).resolve().parent


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, str(path))
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load " + str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


STATIC = load(DIRECTORY / "static_session.py", "manual_static_session")
ROOT = STATIC.ROOT
OWNER_DIRECTORY = STATIC.OWNER_DIRECTORY
CONTAINER = STATIC.CONTAINER
DOCKER = STATIC.DOCKER


LIFECYCLE = load(DIRECTORY.parent / "bringup/lifecycle.py", "manual_lifecycle")
StopRequest = LIFECYCLE.StopRequest


class Hooks(STATIC.Hooks):
    def native_start(self, token):
        argv = DOCKER + [
            "run", "-d", "--name", CONTAINER, "--network", "host", "--ipc", "host",
            "--pid", "host", "--cap-add", "SYS_PTRACE",
            "--ulimit", "rtprio=50",
            "--ulimit", "nice=30:30",
            "--security-opt", "apparmor=feedbot-serial-audit",
            "--device", "/dev/rikibase:/dev/rikibase", "--device", "/dev/rikiimu:/dev/rikiimu",
            *STATIC.DDS_ENVIRONMENT,
            "-e", "FEEDBOT_TEST_TOKEN_FILE=/key/token",
            "-e", "FEEDBOT_TEST_LOG=/results/manual_trace.jsonl",
            "-v", str(ROOT) + ":/work:ro", "-v", str(STATIC.LIDAR_RELEASE) + ":/ltr2:ro",
            "-v", str(self.results) + ":/results:rw",
            "-v", str(OWNER_DIRECTORY) + ":/run/feedbot-lidar:rw",
            "-v", "/run/feedbot-chassis-web:/key:ro",
            "--entrypoint", "/bin/bash", "feedbot-navigation:jazzy-20261122",
            "/work/agv_workspace/integration/mapping/manual_stack.sh", token,
        ]
        logged = list(argv)
        logged[-1] = "<owner-token-redacted>"
        self.base.command(self.store, argv, 20, logged_argv=logged)
        self.started = True

    def container_state(self):
        output = self.base.command(self.store, DOCKER + [
            "inspect", "--format", "{{json .State}}", CONTAINER], 10)
        return json.loads(output)

    def native_stop(self):
        if not self.started:
            return
        state = self.container_state()
        if state.get("Running"):
            self.base.command(self.store, DOCKER + ["stop", "-t", "20", CONTAINER], 30)
        self.store.write_log(
            "stack.log", self.base.command(self.store, DOCKER + ["logs", CONTAINER], 10))
        state = self.container_state()
        self.store.write_json("container_state.json", state)
        if state.get("Running"):
            raise RuntimeError("manual container remained running")
        if state.get("ExitCode") != 0:
            self.native_stop_error = "manual container exited with %s" % state.get("ExitCode")
        self.base.command(self.store, DOCKER + ["rm", CONTAINER], 10)
        self.started = False

    def ready(self):
        self.collect()
        probe = json.loads((self.results / "static_probe.json").read_text())
        if not probe.get("passed"):
            raise RuntimeError("initial static probe did not pass")
        counts = probe.get("command_nonzero_counts", [])
        if not counts or any(value != 0 for value in counts):
            raise RuntimeError("initial static probe lacks zero-command evidence")
        self.store.write_json("sessionready.json", {
            "ready": True,
            "container": CONTAINER,
            "command_nonzero_counts": counts,
            "ready_at": self.base.utc_now(),
        })

    def hold(self, stop_request):
        if stop_request.requested:
            return
        argv = DOCKER + ["wait", CONTAINER]
        process = self.base.subprocess.Popen(
            argv, stdout=self.base.subprocess.PIPE, stderr=self.base.subprocess.PIPE,
            universal_newlines=True)
        stop_request.wait_process = process
        if stop_request.requested:
            process.terminate()
        stdout, stderr = process.communicate()
        stop_request.wait_process = None
        self.store.log_command(argv, process.returncode, stdout, stderr)
        exit_codes = [int(line) for line in stdout.splitlines() if line.strip().isdigit()]
        if exit_codes:
            raise RuntimeError("manual stack exited unexpectedly with %s" % exit_codes[-1])
        if not stop_request.requested:
            raise RuntimeError("docker wait failed with %s: %s" %
                               (process.returncode, (stderr + stdout)[-2000:]))


def bridge_command_summary(results):
    path = results / "manual_trace.jsonl"
    if not path.exists():
        return {"human_command_count": None, "human_nonzero_command_count": None}
    commands = []
    for line in path.read_text().splitlines():
        record = json.loads(line)
        if record.get("event") == "command":
            commands.append(record)
    nonzero = 0
    for record in commands:
        velocity = record.get("value", {}).get("body_velocity")
        if velocity is not None and any(float(value) != 0.0 for value in velocity):
            nonzero += 1
    return {"human_command_count": len(commands), "human_nonzero_command_count": nonzero}


def execute_session(supervisor, hooks, store, stop_request, utc_now):
    status = LIFECYCLE.execute_session(supervisor, hooks, store, stop_request, utc_now)
    report = json.loads((store.root / "session.json").read_text())
    report.update(bridge_command_summary(store.root))
    store.write_json("session.json", report)
    return status


def run(results, stop_request):
    validation = ROOT / "agv_workspace/drivers/ltr2/validation"
    base = load(validation / "real_lidar_trial.py", "manual_session_base")
    supervisor_module = load(ROOT / "navigation_runtime/replacement/lidar/supervisor.py",
                             "manual_session_supervisor")
    base.OWNER_DIRECTORY = OWNER_DIRECTORY
    store = base.EvidenceStore(results)
    store.claim_run()
    hooks = Hooks(base, base.build_hooks(supervisor_module, store), store, results)
    supervisor = supervisor_module.Supervisor(str(OWNER_DIRECTORY), hooks)
    return execute_session(supervisor, hooks, store, stop_request, base.utc_now)


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: manual_session.py RESULTS_DIRECTORY")
    results = Path(sys.argv[1]).resolve()
    results.mkdir(parents=True, exist_ok=False)
    stop_request = StopRequest()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, stop_request.handle)
    return run(results, stop_request)


if __name__ == "__main__":
    sys.exit(main())

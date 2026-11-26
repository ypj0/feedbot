#!/usr/bin/env python3
import importlib.util
import json
from pathlib import Path
import signal
import sys


def load(path, name):
    specification = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


HERE = Path(__file__).resolve().parent
OWNERSHIP = load(HERE / "ownership.py", "base_ownership")
LIFECYCLE = load(HERE / "lifecycle.py", "base_lifecycle")
ROOT = OWNERSHIP.ROOT
OWNER_DIRECTORY = OWNERSHIP.OWNER_DIRECTORY
DOCKER = OWNERSHIP.DOCKER


class Hooks(OWNERSHIP.Hooks):
    def collect(self):
        self.execute("set -e; "
                     "/work/build/tf_probe/tf_edge_probe > /results/tf_edges.jsonl & edge=$!; "
                     "trap 'kill -INT $edge; wait $edge || test $? -eq 130' EXIT; "
                     "python3 /work/agv_workspace/integration/bringup/probe.py "
                     "/results/base_probe.json 15", 35)
        self.execute("set -e; "
                     "ros2 topic info /tf -v --no-daemon --spin-time 3 > /results/tf_topic_info.log; "
                     "ros2 topic info /tf_static -v --no-daemon --spin-time 3 > "
                     "/results/tf_static_topic_info.log; "
                     "cd /results; ros2 run tf2_tools view_frames", 15)
        edges = [json.loads(line) for line in (self.results / "tf_edges.jsonl").read_text().splitlines()]
        ownership = {}
        probe = json.loads((self.results / "base_probe.json").read_text())
        gid_nodes = {}
        for topic in ("/tf", "/tf_static"):
            for endpoint in probe["endpoints"].get(topic, []):
                node = (endpoint["node_namespace"].rstrip("/") + "/" +
                        endpoint["node_name"]).replace("//", "/")
                gid_nodes.setdefault(endpoint["gid"], set()).add(node)
        for parent, child, expected_node in (("odom", "base_link", "/ekf_filter_node"),):
            gids = sorted(set(item["gid"] for item in edges
                              if item["parent"] == parent and item["child"] == child))
            nodes = sorted(set(node for gid in gids for node in gid_nodes.get(gid, set())))
            ownership["%s -> %s" % (parent, child)] = {"gids": gids, "nodes": nodes}
            if len(gids) != 1:
                raise RuntimeError("TF edge does not have exactly one observed GID: %s -> %s" %
                                   (parent, child))
            if nodes != [expected_node]:
                raise RuntimeError("TF edge GID has unexpected endpoint: %s -> %s: %s" %
                                   (parent, child, nodes))
        self.store.write_json("tf_edge_ownership.json", ownership)

    def ready(self):
        self.collect()
        self.store.write_json("sessionready.json", {
            "ready": True, "container": self.container,
            "ready_at": self.base.utc_now(),
        })

    def hold(self, stop_request):
        if stop_request.requested:
            return
        argv = DOCKER + ["wait", self.container]
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
            raise RuntimeError("base stack exited unexpectedly with %s" % exit_codes[-1])
        if not stop_request.requested:
            raise RuntimeError("docker wait failed with %s: %s" %
                               (process.returncode, (stderr + stdout)[-2000:]))


def run(results, stop_request):
    validation = ROOT / "agv_workspace/drivers/ltr2/validation"
    base = load(validation / "real_lidar_trial.py", "base_trial_helpers")
    supervisor_module = load(ROOT / "navigation_runtime/replacement/lidar/supervisor.py",
                             "base_owner_supervisor")
    base.OWNER_DIRECTORY = OWNER_DIRECTORY
    store = base.EvidenceStore(results)
    store.claim_run()
    hooks = Hooks(base, base.build_hooks(supervisor_module, store), store, results)
    supervisor = supervisor_module.Supervisor(str(OWNER_DIRECTORY), hooks)
    return LIFECYCLE.execute_session(supervisor, hooks, store, stop_request, base.utc_now)


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: session.py RESULTS_DIRECTORY")
    results = Path(sys.argv[1]).resolve()
    results.mkdir(parents=True, exist_ok=False)
    stop_request = LIFECYCLE.StopRequest()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, stop_request.handle)
    return run(results, stop_request)


if __name__ == "__main__":
    sys.exit(main())

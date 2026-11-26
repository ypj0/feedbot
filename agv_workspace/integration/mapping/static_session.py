#!/usr/bin/env python3
import importlib.util
import json
import os
from pathlib import Path
import re
import signal
import sys


SHARED_PATH = Path(__file__).resolve().parents[1] / "bringup/ownership.py"
SPEC = importlib.util.spec_from_file_location("mapping_ownership", str(SHARED_PATH))
OWNERSHIP = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(OWNERSHIP)
ROOT = OWNERSHIP.ROOT
LIDAR_RELEASE = OWNERSHIP.LIDAR_RELEASE
OWNER_DIRECTORY = OWNERSHIP.OWNER_DIRECTORY
CONTAINER = "feedbot-mapping-static"
DOCKER = OWNERSHIP.DOCKER
DDS_ENVIRONMENT = OWNERSHIP.DDS_ENVIRONMENT
FACTORY_SERIAL_EXECUTABLES = OWNERSHIP.FACTORY_SERIAL_EXECUTABLES
load = OWNERSHIP.load
socket_pids = OWNERSHIP.socket_pids
require_factory_serial_owner = OWNERSHIP.require_factory_serial_owner


class Hooks(OWNERSHIP.Hooks):
    container = CONTAINER
    stack_script = "/work/agv_workspace/integration/mapping/static_stack.sh"

    def collect(self):
        self.execute("set -e; "
                     "/work/build/tf_probe/tf_edge_probe > /results/tf_edges.jsonl & edge=$!; "
                     "trap 'kill -INT $edge; wait $edge || test $? -eq 130' EXIT; "
                     "python3 /work/agv_workspace/integration/mapping/static_probe.py "
                     "/results/static_probe.json 15", 35)
        self.execute("set -e; "
                     "ros2 topic info /tf -v --no-daemon --spin-time 3 > /results/tf_topic_info.log; "
                     "ros2 topic info /tf_static -v --no-daemon --spin-time 3 > "
                     "/results/tf_static_topic_info.log; "
                     "cd /results; ros2 run tf2_tools view_frames", 15)
        edges = [json.loads(line) for line in (self.results / "tf_edges.jsonl").read_text().splitlines()]
        ownership = {}
        probe = json.loads((self.results / "static_probe.json").read_text())
        gid_nodes = {}
        for topic in ("/tf", "/tf_static"):
            for endpoint in probe["endpoints"].get(topic, []):
                node = (endpoint["node_namespace"].rstrip("/") + "/" +
                        endpoint["node_name"]).replace("//", "/")
                gid_nodes.setdefault(endpoint["gid"], set()).add(node)
        for parent, child, expected_node in (("map", "odom", "/slam_toolbox"),
                                             ("odom", "base_link", "/ekf_filter_node")):
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


def interrupted(signum, unused_frame):
    raise RuntimeError("received signal %d" % signum)


def run(results):
    validation = ROOT / "agv_workspace/drivers/ltr2/validation"
    base = load(validation / "real_lidar_trial.py", "static_session_base")
    supervisor_module = load(ROOT / "navigation_runtime/replacement/lidar/supervisor.py",
                             "static_session_supervisor")
    base.OWNER_DIRECTORY = OWNER_DIRECTORY
    store = base.EvidenceStore(results)
    store.claim_run()
    hooks = Hooks(base, base.build_hooks(supervisor_module, store), store, results)
    supervisor = supervisor_module.Supervisor(str(OWNER_DIRECTORY), hooks)
    report = {"passed": False, "motion_commands_sent": False, "factory_restored": False,
              "started_at": base.utc_now(), "candidate": str(ROOT)}
    primary_error = None
    cleanup_errors = []
    restore_error = None
    try:
        supervisor.start()
        hooks.collect()
        report["data_collected"] = True
    except BaseException as error:
        primary_error = error
        report["error"] = "%s: %s" % (type(error).__name__, error)
    finally:
        cleanup_errors = hooks.shutdown_lifecycle()
        try:
            if supervisor.needs_restore:
                transaction = json.loads((OWNER_DIRECTORY / "transaction.json").read_text())
                if transaction.get("state") == "rollback_failed":
                    raise RuntimeError("prior restoration failed: " + str(transaction.get("error")))
                supervisor.restore()
        except BaseException as error:
            restore_error = error
        finally:
            supervisor.close()
        if cleanup_errors:
            report["cleanup_errors"] = cleanup_errors
        if restore_error is not None:
            report["restore_error"] = "%s: %s" % (type(restore_error).__name__, restore_error)
        if hooks.native_stop_error is not None:
            report["native_stop_error"] = hooks.native_stop_error
        report["factory_restored"] = not supervisor.needs_restore
        report["passed"] = bool(report.get("data_collected") and not cleanup_errors and
                                restore_error is None and hooks.native_stop_error is None and
                                report["factory_restored"])
        report["finished_at"] = base.utc_now()
        store.write_json("session.json", report)
    return 0 if primary_error is None and report["passed"] else 1


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: static_session.py RESULTS_DIRECTORY")
    results = Path(sys.argv[1]).resolve()
    results.mkdir(parents=True, exist_ok=False)
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, interrupted)
    return run(results)


if __name__ == "__main__":
    sys.exit(main())

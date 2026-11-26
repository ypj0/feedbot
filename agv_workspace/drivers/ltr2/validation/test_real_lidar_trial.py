#!/usr/bin/env python3

import importlib.util
import pathlib
import tempfile
import types
import unittest
import json
import subprocess
from unittest.mock import patch


HERE = pathlib.Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("real_lidar_trial", str(HERE / "real_lidar_trial.py"))
trial = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trial)
probe_spec = importlib.util.spec_from_file_location("ros_probe", str(HERE / "ros_probe.py"))
probe = importlib.util.module_from_spec(probe_spec)
probe_spec.loader.exec_module(probe)


class SequenceHooks:
    def __init__(self):
        self.events = []

    def silence(self, module):
        self.events.append(("silence", module))
        return {"passed": True}

    def transition(self, name):
        self.events.append(("transition", name))

    def collect(self, module):
        self.events.append(("collect", module))
        return {"passed": True, "module": module}


class SequenceSupervisor:
    def __init__(self, hooks):
        self.hooks = hooks

    def start(self):
        self.hooks.events.append(("supervisor", "start"))


class SupervisorHooks:
    def __init__(self, fail=None):
        self.events = []
        self.fail = fail
        self.started = 0

    def step(self, name):
        self.events.append(name)
        if self.fail == name:
            raise RuntimeError(name)

    def inhibit(self):
        self.step("inhibit")

    def factory_off(self):
        self.step("factory_off")

    def wait_free(self):
        self.step("wait_free")

    def native_start(self, unused_token):
        self.step("native_start")

    def transition(self, name):
        self.step(name)

    def desired_awake(self):
        self.step("desired_awake")
        return False

    def native_healthy(self, awake):
        if awake:
            raise AssertionError("trial must configure asleep")
        self.step("native_healthy_false")

    def native_stop(self):
        self.step("native_stop")

    def factory_on(self):
        self.step("factory_on")

    def factory_healthy(self):
        self.step("factory_healthy")


def archived_supervisor():
    path = HERE.parents[3] / "navigation_runtime/replacement/lidar/supervisor.py"
    spec = importlib.util.spec_from_file_location("archived_supervisor", str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TrialTests(unittest.TestCase):
    def test_socket_owners_distinguishes_ros_clients_from_master_and_lidar(self):
        ros_client = 'CLOSE-WAIT 1 0 127.0.0.1:42178 127.0.0.1:11372 users:(("python2",pid=1,fd=7))'
        master = 'LISTEN 0 128 [::]:11372 [::]:* users:(("python2",pid=2,fd=8))'
        lidar = 'ESTAB 0 0 192.168.10.200:50000 192.168.10.160:2105 users:(("driver",pid=3,fd=9))'
        udp = 'UNCONN 0 0 0.0.0.0:2105 0.0.0.0:* users:(("driver",pid=3,fd=10))'
        closed = 'TIME-WAIT 0 0 192.168.10.200:50001 192.168.10.160:2105'
        with patch.object(trial, "command", side_effect=["\n".join([ros_client, master, lidar, closed]), udp]):
            self.assertEqual(trial.socket_owners(None), [master, lidar, udp])
        with patch.object(trial, "command", side_effect=[ros_client, ""]):
            trial.require_ports_free(None)

    def test_real_lidar_sequence_has_two_activations_and_stopped_checks(self):
        hooks = SequenceHooks()
        result = trial.exercise(SequenceSupervisor(hooks), hooks)
        self.assertEqual(hooks.events, [
            ("supervisor", "start"),
            ("silence", "configured_no_scan"),
            ("transition", "activate"),
            ("collect", "first_activation_scans"),
            ("transition", "deactivate"),
            ("silence", "deactivated_no_scan"),
            ("transition", "activate"),
            ("collect", "second_activation_scans"),
            ("transition", "cleanup"),
        ])
        self.assertTrue(result["first_activation"]["passed"])
        self.assertTrue(result["second_activation"]["passed"])

    def test_archived_supervisor_configures_asleep_then_restores_in_order(self):
        module = archived_supervisor()
        hooks = SupervisorHooks()
        with tempfile.TemporaryDirectory() as directory:
            supervisor = module.Supervisor(directory, hooks)
            supervisor.start()
            supervisor.restore()
            supervisor.close()
        self.assertEqual(hooks.events, [
            "inhibit", "factory_off", "wait_free", "native_start", "configure",
            "desired_awake", "native_healthy_false", "native_stop", "wait_free",
            "factory_on", "factory_healthy",
        ])

    def test_configure_failure_restores_before_factory_health_check(self):
        module = archived_supervisor()
        hooks = SupervisorHooks(fail="configure")
        with tempfile.TemporaryDirectory() as directory:
            supervisor = module.Supervisor(directory, hooks)
            with self.assertRaises(RuntimeError):
                supervisor.start()
            supervisor.close()
        self.assertEqual(hooks.events[-4:], ["native_stop", "wait_free", "factory_on", "factory_healthy"])

    def test_native_stop_failure_never_starts_factory(self):
        module = archived_supervisor()
        hooks = SupervisorHooks()
        with tempfile.TemporaryDirectory() as directory:
            supervisor = module.Supervisor(directory, hooks)
            supervisor.start()
            hooks.fail = "native_stop"
            with self.assertRaises(RuntimeError):
                supervisor.restore()
            supervisor.close()
        self.assertNotIn("factory_on", hooks.events)

    def test_external_restore_stops_trial_before_native_restore(self):
        events = []

        class FakeSupervisor:
            def __init__(self, unused_root, unused_hooks):
                events.append("supervisor_created")

            def restore(self):
                events.append("restore")

            def close(self):
                events.append("close")

        module = types.SimpleNamespace(Supervisor=FakeSupervisor, Hooks=object)
        with tempfile.TemporaryDirectory() as directory, \
                patch.object(trial, "EVIDENCE", pathlib.Path(directory)), \
                patch.object(trial, "stop_trial_before_external_restore", side_effect=lambda store: events.append("stop_trial")), \
                patch.object(trial, "OWNER_DIRECTORY", pathlib.Path(directory)), \
                patch.object(trial, "systemd_state", return_value={"LoadState": "not-found"}), \
                patch.object(trial, "load_supervisor_module", return_value=module), \
                patch.object(trial, "build_hooks", return_value=object()):
            self.assertEqual(trial.restore_only(), 0)
        self.assertEqual(events, ["stop_trial", "supervisor_created", "restore", "close"])

    def test_delayed_restore_verifies_factory_without_waiting_for_its_ports(self):
        for previous_state in (None, {"state": "factory"}):
            module = archived_supervisor()
            hooks = SupervisorHooks()
            with tempfile.TemporaryDirectory() as directory:
                root = pathlib.Path(directory)
                if previous_state is not None:
                    (root / "transaction.json").write_text(json.dumps(previous_state))
                (root / "permit").write_text("obsolete")
                def state(unused_store, unit):
                    return {"LoadState": "not-found"} if unit == trial.NATIVE_UNIT else {"ActiveState": "active"}
                with patch.object(trial, "EVIDENCE", root / "evidence"), \
                        patch.object(trial, "OWNER_DIRECTORY", root), \
                        patch.object(trial, "stop_trial_before_external_restore"), \
                        patch.object(trial, "systemd_state", side_effect=state), \
                        patch.object(trial, "load_supervisor_module", return_value=module), \
                        patch.object(trial, "build_hooks", return_value=hooks):
                    self.assertEqual(trial.restore_only(), 0)
                self.assertEqual(hooks.events, ["factory_healthy"])
                self.assertFalse((root / "permit").exists())
                if previous_state is not None:
                    self.assertEqual(json.loads((root / "transaction.json").read_text()), previous_state)

    def test_timeout_bytes_are_decoded_before_evidence_write(self):
        with tempfile.TemporaryDirectory() as directory:
            store = trial.EvidenceStore(directory)
            error = subprocess.TimeoutExpired(["command"], 1, output=b"stdout-bytes", stderr=b"stderr-bytes")
            with patch.object(trial.subprocess, "run", side_effect=error):
                with self.assertRaises(RuntimeError):
                    trial.command(store, ["command"], 1)
            record = json.loads((pathlib.Path(directory) / "commands.log").read_text())
        self.assertEqual(record["stdout"], "stdout-bytes")
        self.assertEqual(record["stderr"], "stderr-bytes")

    def test_factory_geometry_is_reported_without_relaxing_managed_geometry(self):
        stamp = types.SimpleNamespace(secs=int(trial.time.time()), nsecs=0)
        header = types.SimpleNamespace(seq=1, stamp=stamp, frame_id="scan")
        message = types.SimpleNamespace(
            header=header,
            angle_min=-1.65,
            angle_max=1.65,
            angle_increment=0.003067961661145091,
            time_increment=0.0,
            scan_time=0.1,
            range_min=0.05,
            range_max=40.0,
            ranges=[1.0] * 1074 + [41.0],
            intensities=[2.0] * 1075,
        )
        factory_metadata = probe.validate_scan(message, trial.time.time(), strict_geometry=False)
        self.assertFalse(factory_metadata["geometry_matches_1075"])
        self.assertEqual(factory_metadata["finite_out_of_declared_range_count"], 1)
        with self.assertRaises(ValueError):
            probe.validate_scan(message, trial.time.time(), strict_geometry=True)

    def test_native_unit_uses_separate_logs_and_redacts_owner_token(self):
        class BaseHooks:
            def __init__(self, unused_config, root):
                self.root = pathlib.Path(root)

        module = types.SimpleNamespace(Hooks=BaseHooks)
        calls = []
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            binary = root / "ltr2_managed_ros1"
            binary.write_text("binary")
            binary.chmod(0o700)
            helper = root / "ros_probe.py"
            helper.write_text("helper")
            store = trial.EvidenceStore(root / "evidence")
            with patch.object(trial, "BINARY", binary), \
                    patch.object(trial, "PROBE", helper), \
                    patch.object(trial, "OWNER_DIRECTORY", root / "owner"), \
                    patch.object(trial, "systemd_state", return_value={"LoadState": "not-found"}), \
                    patch.object(trial, "command", side_effect=lambda *args, **kwargs: calls.append((args, kwargs))):
                trial.build_hooks(module, store).native_start("a" * 48)
        argv = calls[0][0][1]
        logged_argv = calls[0][1]["logged_argv"]
        script = argv[argv.index("-c") + 1]
        self.assertIn('roscore -p "$1" >>"$6"', script)
        self.assertNotIn("a" * 48, logged_argv)
        self.assertIn("<owner-token-redacted>", logged_argv)


if __name__ == "__main__":
    unittest.main()

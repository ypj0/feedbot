import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import textwrap
import time
import unittest
from unittest import mock


SUPERVISOR = (Path(__file__).parents[3] / "acceptance" / "runs" /
              "20261122_chassis_read_timing_02" / "supervise_trial.py")
SPEC = importlib.util.spec_from_file_location("supervise_trial", SUPERVISOR)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class SupervisorTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.worker = self.root / "worker.py"
        self.worker.write_text(textwrap.dedent("""
            import os, signal, subprocess, sys, time
            mode, ready, pidfile = sys.argv[1:]
            child = None
            if mode.startswith('controller'):
                child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
                open(pidfile, 'w').write('%d %d' % (os.getpid(), child.pid))
            if mode.startswith('collector'):
                open(pidfile, 'w').write(str(os.getpid()))
            if mode.startswith('collector') and mode != 'collector-no-ready':
                with open(ready, 'w') as stream: stream.write('ready\\n')
            if mode == 'collector-ok': time.sleep(0.1); raise SystemExit(0)
            if mode == 'collector-fault': time.sleep(0.1); raise SystemExit(7)
            if mode == 'controller-early': time.sleep(0.1); raise SystemExit(0)
            time.sleep(60)
        """))

    def tearDown(self):
        self.temporary.cleanup()

    def argv(self, mode, ready, pidfile):
        return [sys.executable, str(self.worker), mode, str(ready), str(pidfile)]

    def assert_gone(self, pidfile):
        for pid in map(int, pidfile.read_text().split()):
            with self.assertRaises(ProcessLookupError):
                os.kill(pid, 0)

    def run_case(self, collector_mode, controller_mode):
        ready = self.root / (collector_mode + ".fifo")
        pidfile = self.root / (controller_mode + ".pids")
        collector_pids = self.root / "collector.pids"
        status = MODULE.supervise(self.argv(collector_mode, ready, collector_pids),
                                  self.argv(controller_mode, ready, pidfile), ready, 0.5, 0.5)
        self.assert_gone(pidfile)
        self.assert_gone(collector_pids)
        return status

    def test_normal_collector_exit_reaps_controller_tree(self):
        self.assertEqual(self.run_case("collector-ok", "controller-run"), 0)

    def test_collector_fault_is_preserved_and_reaps_controller_tree(self):
        self.assertEqual(self.run_case("collector-fault", "controller-run"), 7)

    def test_controller_early_exit_stops_collector(self):
        self.assertEqual(self.run_case("collector-run", "controller-early"), 1)

    def test_external_termination_reaps_both_groups(self):
        ready = self.root / "external.fifo"
        controller_pids = self.root / "external-controller.pids"
        collector_pids = self.root / "external-collector.pids"
        script = textwrap.dedent("""
            import importlib.util, pathlib, sys
            spec=importlib.util.spec_from_file_location('s', sys.argv[1]); m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
            root=pathlib.Path(sys.argv[2]); worker=root/'worker.py'; ready=root/'external.fifo'
            argv=lambda mode, pids: [sys.executable, str(worker), mode, str(ready), str(pids)]
            raise SystemExit(m.supervise(argv('collector-run', root/'external-collector.pids'), argv('controller-run', root/'external-controller.pids'), ready, .5, .5))
        """)
        process = subprocess.Popen([sys.executable, "-c", script, str(SUPERVISOR), str(self.root)])
        for _ in range(100):
            if controller_pids.exists() and collector_pids.exists(): break
            time.sleep(0.02)
        process.send_signal(signal.SIGTERM)
        self.assertEqual(process.wait(timeout=3), 128 + signal.SIGTERM)
        self.assert_gone(controller_pids)
        self.assert_gone(collector_pids)

    def test_terminated_before_ready_does_not_start_controller(self):
        ready = self.root / "before-ready.fifo"
        collector_pids = self.root / "before-ready-collector.pids"
        controller_pids = self.root / "before-ready-controller.pids"
        script = textwrap.dedent("""
            import importlib.util, pathlib, sys
            spec=importlib.util.spec_from_file_location('s', sys.argv[1]); m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
            root=pathlib.Path(sys.argv[2]); worker=root/'worker.py'; ready=root/'before-ready.fifo'
            argv=lambda mode, pids: [sys.executable, str(worker), mode, str(ready), str(pids)]
            raise SystemExit(m.supervise(argv('collector-no-ready', root/'before-ready-collector.pids'), argv('controller-run', root/'before-ready-controller.pids'), ready, .5, .5))
        """)
        process = subprocess.Popen([sys.executable, "-c", script, str(SUPERVISOR), str(self.root)])
        for _ in range(100):
            if collector_pids.exists(): break
            time.sleep(0.02)
        process.send_signal(signal.SIGTERM)
        self.assertEqual(process.wait(timeout=3), 128 + signal.SIGTERM)
        self.assert_gone(collector_pids)
        self.assertFalse(controller_pids.exists())

    def test_exited_leader_is_reaped_without_waiting_full_grace(self):
        process = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"],
                                   start_new_session=True)
        started = time.monotonic()
        MODULE.terminate_group(process, 2.0, 2.0)
        self.assertLess(time.monotonic() - started, 1.0)
        self.assertIsNotNone(process.returncode)

        class PersistentGroup:
            pid = 424242
            def poll(self): return None
            def wait(self): return None

        signals = []
        with mock.patch.object(MODULE.os, "killpg",
                               side_effect=lambda group, signum: signals.append((group, signum))), \
             mock.patch.object(MODULE.os, "waitpid", return_value=(0, 0)):
            with self.assertRaisesRegex(RuntimeError, "remained after SIGKILL"):
                MODULE.terminate_group(PersistentGroup(), 0.0, 0.0)
        self.assertEqual([entry[1] for entry in signals if entry[1] != 0],
                         [signal.SIGINT, signal.SIGTERM, signal.SIGKILL])


if __name__ == "__main__":
    unittest.main()

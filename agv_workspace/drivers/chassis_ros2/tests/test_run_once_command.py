import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


if "FEEDBOT_RUNNER_PATH" in os.environ:
    RUNNER = Path(os.environ["FEEDBOT_RUNNER_PATH"])
else:
    RUNNER = (Path(__file__).parents[3] / "acceptance" / "runs" /
              "20261122_chassis_read_timing_01" / "run_once.py")
SPEC = importlib.util.spec_from_file_location("run_once", RUNNER)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class CommandTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        MODULE.RESULTS = Path(self.temporary.name)

    def tearDown(self):
        self.temporary.cleanup()

    def test_normal_output_is_recorded(self):
        output = MODULE.command("normal", [sys.executable, "-c", "print('normal')"], 5)
        self.assertEqual(output, "normal\n")
        self.assertEqual((MODULE.RESULTS / "normal.log").read_text(), "normal\n")
        self.assertIn('"returncode": 0', (MODULE.RESULTS / "normal.json").read_text())

    def test_accepted_nonzero_output_is_recorded(self):
        output = MODULE.command(
            "nonzero", [sys.executable, "-c", "print('nonzero'); raise SystemExit(7)"],
            5, accepted=(7,))
        self.assertEqual(output, "nonzero\n")
        self.assertIn('"returncode": 7', (MODULE.RESULTS / "nonzero.json").read_text())

    def test_timeout_is_recorded_and_raised(self):
        with self.assertRaises(subprocess.TimeoutExpired):
            MODULE.command("timeout", [sys.executable, "-c", "import time; time.sleep(1)"], 0.01)
        self.assertIn("TIMEOUT after 0.01 seconds", (MODULE.RESULTS / "timeout.log").read_text())
        self.assertIn('"timed_out": true', (MODULE.RESULTS / "timeout.json").read_text())


if __name__ == "__main__":
    unittest.main()

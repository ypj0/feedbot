import os
from pathlib import Path
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from run_offline import execute


class ExecutionTests(unittest.TestCase):
    def test_returncode_and_log_digest(self):
        with tempfile.TemporaryDirectory() as directory:
            report = execute("failure", [sys.executable, "-c", "print('fixture'); raise SystemExit(7)"], Path(directory))
            self.assertEqual("fail", report["status"])
            self.assertEqual(7, report["returncode"])
            self.assertEqual(64, len(report["log_sha256"]))

    def test_timeout_kills_child_group(self):
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / "unexpected"
            child = "import time; from pathlib import Path; time.sleep(1); Path(%r).touch()" % str(marker)
            parent = "import subprocess,sys,time; subprocess.Popen([sys.executable,'-c',%r]); time.sleep(20)" % child
            report = execute("timeout", [sys.executable, "-c", parent], Path(directory), timeout=0.3)
            self.assertEqual("timeout", report["status"])
            time.sleep(1.1)
            self.assertFalse(marker.exists())


if __name__ == "__main__":
    unittest.main()

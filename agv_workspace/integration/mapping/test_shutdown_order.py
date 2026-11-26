from pathlib import Path
import subprocess
import unittest


class ShutdownOrderTest(unittest.TestCase):
    def test_all_processes_signalled_before_waiting(self):
        for filename in ("manual_stack.sh", "static_stack.sh"):
            with self.subTest(filename=filename):
                source = Path(__file__).with_name(filename).read_text()
                function = source.split("shutdown_stack() {", 1)[1].split("\ntrap ", 1)[0]
                script = """
pids=(101 102)
names=(first second)
kill() { if [[ $1 == -TERM ]]; then echo "signal $2"; fi; return 0; }
wait() { echo "wait $1"; return 0; }
shutdown_stack() {""" + function + "\nshutdown_stack 0\n"
                result = subprocess.run(["bash", "-c", script], capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stdout.splitlines(),
                                 ["signal 102", "signal 101", "wait 102", "wait 101"])


if __name__ == "__main__":
    unittest.main()

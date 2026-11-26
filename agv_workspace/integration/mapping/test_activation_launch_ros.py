import importlib.util
from pathlib import Path
import unittest

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit


SPEC = importlib.util.spec_from_file_location(
    "mapping_launch", Path(__file__).parent / "mapping.launch.py")
MAPPING = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MAPPING)


class ActivationExitTest(unittest.TestCase):
    def run_launch(self, code):
        process = ExecuteProcess(cmd=["/bin/sh", "-c", "exit %d" % code])
        service = LaunchService()
        service.include_launch_description(LaunchDescription([
            RegisterEventHandler(OnProcessExit(
                target_action=process, on_exit=MAPPING.require_activation_success)),
            process,
        ]))
        return service.run()

    def test_activation_failure_propagates_to_launch(self):
        self.assertNotEqual(self.run_launch(7), 0)

    def test_activation_success_does_not_fail_launch(self):
        self.assertEqual(self.run_launch(0), 0)


if __name__ == "__main__":
    unittest.main()

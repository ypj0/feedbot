import importlib.util
from pathlib import Path
import sys
from types import ModuleType
import unittest
from unittest.mock import patch


class PublicationTest(unittest.TestCase):
    def test_ekf_selects_asynchronous_publication_environment(self):
        launch = ModuleType("launch")
        launch.LaunchDescription = list
        actions = ModuleType("launch_ros.actions")
        actions.Node = lambda **arguments: arguments
        specification = importlib.util.spec_from_file_location(
            "ekf_launch", Path(__file__).with_name("ekf.launch.py"))
        module = importlib.util.module_from_spec(specification)
        with patch.dict(sys.modules, {"launch": launch, "launch_ros.actions": actions}):
            specification.loader.exec_module(module)
            nodes = module.generate_launch_description()
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0]["executable"], str(
            Path(__file__).resolve().parents[3] / "install/lib/feedbot_ekf/feedbot_ekf"))
        self.assertNotIn("prefix", nodes[0])
        self.assertEqual(nodes[0].get("additional_env"), {
            "RMW_FASTRTPS_PUBLICATION_MODE": "ASYNCHRONOUS"})


if __name__ == "__main__":
    unittest.main()

import ast
import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).parent
SHARED = ROOT.parent / "bringup"
SPEC = importlib.util.spec_from_file_location("static_session", ROOT / "static_session.py")
SESSION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SESSION)


class StaticSessionTest(unittest.TestCase):
    def test_cmake_install_and_host_tool_paths(self):
        stack = (SHARED / "stack.sh").read_text()
        session = (SHARED / "ownership.py").read_text()
        for package in ("feedbot_chassis_ros2", "feedbot_imu_ros2"):
            self.assertIn("source /work/install/share/" + package + "/local_setup.bash", stack)
        self.assertNotIn("/work/install/local_setup.bash", stack + session)
        self.assertIn('["/bin/fuser", "-v", device]', session)

    def test_factory_serial_identity(self):
        for device, executable in SESSION.FACTORY_SERIAL_EXECUTABLES.items():
            SESSION.require_factory_serial_owner(device, executable)
            with self.assertRaisesRegex(RuntimeError, "unexpected factory serial owner"):
                SESSION.require_factory_serial_owner(device, "/work/install/lib/candidate")

    def test_stack_scope_and_reverse_shutdown(self):
        stack = (ROOT / "static_stack.sh").read_text() + (SHARED / "stack.sh").read_text()
        for name in ("feedbot_chassis_ros2", "feedbot_imu_ros2", "ekf.launch.py",
                     "ltr2_ros2", "mapping.launch.py"):
            self.assertIn(name, stack)
        for forbidden in ("cmd_vel", "dashboard", "planner", "controller_server"):
            self.assertNotIn(forbidden, stack)
        self.assertIn("index=${#pids[@]}-1", stack)

    def test_session_restore_and_boundaries(self):
        source = (ROOT / "static_session.py").read_text() + (SHARED / "ownership.py").read_text()
        tree = ast.parse(source)
        for value in ("supervisor.restore()", "shutdown_lifecycle", "factory_healthy",
                      '"/dev/rikibase:/dev/rikibase"', '"/dev/rikiimu:/dev/rikiimu"',
                      "<owner-token-redacted>"):
            self.assertIn(value, source)
        self.assertNotIn("text=True", source)
        self.assertTrue(any(isinstance(node, ast.Try) and node.finalbody for node in ast.walk(tree)))

    def test_probe_contract_and_single_container(self):
        source = (ROOT / "static_session.py").read_text() + (SHARED / "ownership.py").read_text()
        self.assertIn("/results/static_probe.json 15", source)
        self.assertIn('CONTAINER = "feedbot-mapping-static"', source)
        self.assertIn('"ROS_DOMAIN_ID=89"', source)
        self.assertIn("tf_edge_probe", source)
        self.assertIn("tf_edge_ownership.json", source)
        self.assertIn('("map", "odom", "/slam_toolbox")', source)
        self.assertIn('("odom", "base_link", "/ekf_filter_node")', source)
        self.assertNotIn("retry", source.lower())


if __name__ == "__main__":
    unittest.main()

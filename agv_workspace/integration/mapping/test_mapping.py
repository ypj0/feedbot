import pathlib
import subprocess
import unittest
import xml.etree.ElementTree as ElementTree

ROOT = pathlib.Path(__file__).parent


class MappingConfigurationTest(unittest.TestCase):
    def test_localization_motion_update_cadence_is_explicit(self):
        localization = (ROOT / "config/localization.yaml").read_text()
        self.assertIn("    update_min_d: 0.05\n", localization)
        self.assertIn("    update_min_a: 0.05\n", localization)

    def test_system_time_and_frames(self):
        slam = (ROOT / "config/slam_toolbox.yaml").read_text()
        localization = (ROOT / "config/localization.yaml").read_text()
        self.assertEqual(slam.count("use_sim_time: false"), 1)
        self.assertIn("map_frame: map", slam)
        self.assertIn("odom_frame: odom", slam)
        self.assertIn("base_frame: base_link", slam)
        self.assertEqual(localization.count("use_sim_time: false"), 3)
        self.assertIn("scan_topic: /scan", localization)
        self.assertIn("attempt_respawn_reconnection: false", localization)
        chassis = (ROOT.parents[1] / "drivers/chassis_ros2/launch/chassis.launch.py").read_text()
        self.assertIn('"use_sim_time": False', chassis)

    def test_map_to_odom_owners_are_separated(self):
        mapping = (ROOT / "mapping.launch.py").read_text()
        localization = (ROOT / "localization.launch.py").read_text()
        self.assertIn("online_async_launch.py", mapping)
        self.assertNotIn("nav2_amcl", mapping)
        self.assertIn('package="nav2_amcl"', localization)
        self.assertNotIn("slam_toolbox", localization)
        self.assertNotIn('lidar_static_transforms', mapping)
        self.assertNotIn('lidar_static_transforms', localization)
        stack = (ROOT.parent / 'bringup/stack.sh').read_text()
        self.assertNotIn('start_process frames ', stack)
        self.assertFalse((ROOT / 'frames.launch.py').exists())

    def test_map_is_required_and_factory_transform_is_explicit(self):
        runner = str(ROOT / "run.sh")
        missing_map = subprocess.run([runner, "localization", "/does/not/exist"],
                                     capture_output=True, text=True)
        self.assertEqual(missing_map.returncode, 2)
        robot = ElementTree.parse(ROOT.parents[1] / "drivers/chassis_ros2/config/chassis.urdf").getroot()
        joint = robot.find("./joint[@name='laser_joint']")
        self.assertEqual(joint.find('origin').attrib['xyz'], '0.27 0 0.20')
        scan = robot.find("./joint[@name='scan_joint']")
        self.assertEqual(scan.find('parent').attrib['link'], 'laser_link')
        self.assertEqual(scan.find('child').attrib['link'], 'scan')

    def test_runner_has_single_attempt_and_explicit_map_mount(self):
        runner = (ROOT / "run.sh").read_text()
        self.assertEqual(runner.count("docker.sock run"), 1)
        self.assertNotIn("while ", runner)
        self.assertNotIn("docker.sock stop", runner)
        self.assertIn('--name feedbot-mapping-mode', runner)
        self.assertIn(':/maps:ro', runner)

    def test_managed_stop_targets_container_with_ros_signal(self):
        runner = (ROOT / "run.sh").read_text()
        managed = (ROOT / "start.sh").read_text()
        self.assertIn("--stop-signal SIGINT", runner)
        self.assertIn("ExecStop=/opt/feedbot-nav2/docker/docker", managed)
        self.assertIn("stop -t 20 feedbot-mapping-mode", managed)
        self.assertIn("BindsTo=feedbot-base.service", managed)
        self.assertIn("After=feedbot-base.service", managed)
        self.assertIn("Restart=no", managed)


if __name__ == "__main__":
    unittest.main()

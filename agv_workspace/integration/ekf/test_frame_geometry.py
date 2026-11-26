from pathlib import Path
import unittest
import xml.etree.ElementTree as ElementTree


EKF_ROOT = Path(__file__).resolve().parent
WORKSPACE = EKF_ROOT.parents[1]
URDF_PATH = WORKSPACE / "drivers/chassis_ros2/config/chassis.urdf"


class FrameGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = ElementTree.parse(URDF_PATH).getroot()
        cls.joints = {joint.attrib["name"]: joint for joint in cls.robot.findall("joint")}

    def test_base_link_is_single_root_and_footprint_is_zero_child(self):
        children = {
            joint.find("child").attrib["link"]: joint.find("parent").attrib["link"]
            for joint in self.robot.findall("joint")
        }
        self.assertNotIn("base_link", children)
        self.assertEqual(children["base_footprint"], "base_link")
        self.assertEqual(len(children), len(self.robot.findall("joint")))
        joint = self.joints["base_footprint_joint"]
        self.assertEqual(joint.attrib["type"], "fixed")
        self.assertEqual(joint.find("origin").attrib, {"xyz": "0 0 0", "rpy": "0 0 0"})

    def test_wheel_centres_and_axes_are_unchanged_except_height(self):
        expected = {
            "left_wheel_joint": "0 0.2275 0.085",
            "right_wheel_joint": "0 -0.2275 0.085",
        }
        for name, xyz in expected.items():
            joint = self.joints[name]
            self.assertEqual(joint.find("parent").attrib["link"], "base_link")
            self.assertEqual(joint.find("origin").attrib["xyz"], xyz)
            self.assertEqual(joint.find("axis").attrib["xyz"], "0 1 0")

    def test_imu_transform_is_zero_from_base_link(self):
        joint = self.joints["imu_joint"]
        self.assertEqual(joint.attrib["type"], "fixed")
        self.assertEqual(joint.find("parent").attrib["link"], "base_link")
        self.assertEqual(joint.find("child").attrib["link"], "imu_link")
        self.assertEqual(joint.find("origin").attrib, {"xyz": "0 0 0", "rpy": "0 0 0"})

    def test_lidar_fixed_geometry_has_one_urdf_definition(self):
        for name, parent, child, xyz in (
                ("laser_joint", "base_link", "laser_link", "0.27 0 0.20"),
                ("scan_joint", "laser_link", "scan", "0 0 0")):
            joint = self.joints[name]
            self.assertEqual(joint.attrib["type"], "fixed")
            self.assertEqual(joint.find("parent").attrib["link"], parent)
            self.assertEqual(joint.find("child").attrib["link"], child)
            self.assertEqual(joint.find("origin").attrib, {"xyz": xyz, "rpy": "0 0 0"})
        self.assertNotIn("static_transform_publisher", (EKF_ROOT / "ekf.launch.py").read_text())


if __name__ == "__main__":
    unittest.main()

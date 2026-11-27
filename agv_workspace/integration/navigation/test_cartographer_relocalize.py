import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch


ROOT = Path(__file__).parent


def load(name, modules):
    spec = importlib.util.spec_from_file_location(name, ROOT / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


def make_node(module, charge=0, docked=None, scan=None, map_msg=None):
    node = module.CartographerRelocalizer.__new__(module.CartographerRelocalizer)
    node.accept_score = 0.5
    node.laser_max_range = 3.5
    node.charge_status = charge
    node.docked_pose = docked
    node.scan = scan
    node.map = map_msg
    return node


class CartographerRelocalizerTest(unittest.TestCase):
    def setUp(self):
        ros_modules = {
            'rclpy': Mock(),
            'rclpy.node': types.SimpleNamespace(Node=object),
            'rclpy.qos': types.SimpleNamespace(
                DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL='transient'),
                QoSProfile=Mock(),
                ReliabilityPolicy=types.SimpleNamespace(RELIABLE='RELIABLE'),
                qos_profile_sensor_data=Mock()),
            'diagnostic_msgs.msg': types.SimpleNamespace(DiagnosticArray=object),
            'geometry_msgs.msg': types.SimpleNamespace(
                Point=object, PoseWithCovariance=object,
                PoseWithCovarianceStamped=object, Quaternion=object),
            'nav_msgs.msg': types.SimpleNamespace(OccupancyGrid=object),
            'sensor_msgs.msg': types.SimpleNamespace(LaserScan=object),
            'std_msgs.msg': types.SimpleNamespace(Header=object),
            'std_srvs.srv': types.SimpleNamespace(Trigger=object),
        }
        self.module = load('cartographer_relocalize', ros_modules)

    def test_docked_pose_wins_when_charging(self):
        node = make_node(self.module, charge=1, docked=[1.0, 2.0, 0.5])
        node.publish_pose = Mock()
        node.get_logger = Mock(return_value=Mock())
        resp = types.SimpleNamespace(success=False, message='')
        node.reloc_cb(None, resp)
        self.assertTrue(resp.success)
        self.assertIn('docked', resp.message)
        node.publish_pose.assert_called_once_with(1.0, 2.0, 0.5)

    def test_not_charging_returns_global(self):
        node = make_node(self.module, charge=0)
        node.publish_pose = Mock()
        node.get_logger = Mock(return_value=Mock())
        resp = types.SimpleNamespace(success=False, message='')
        node.reloc_cb(None, resp)
        self.assertTrue(resp.success)
        self.assertIn('global', resp.message)
        node.publish_pose.assert_not_called()

    def test_charge_detection_auto_seeds_docked_pose_once(self):
        node = make_node(self.module, charge=0, docked=[1.0, 2.0, 0.5])
        node.seeded_for_charge = False
        node.publish_pose = Mock()
        node.get_logger = Mock(return_value=Mock())
        kv = types.SimpleNamespace(key='charge', value='2')
        status = types.SimpleNamespace(name='chassis', values=[kv])
        msg = types.SimpleNamespace(status=[status])

        node.diag_cb(msg)
        self.assertEqual(node.charge_status, 2)
        node.publish_pose.assert_called_once_with(1.0, 2.0, 0.5)
        self.assertTrue(node.seeded_for_charge)

        node.publish_pose.reset_mock()
        node.diag_cb(msg)
        node.publish_pose.assert_not_called()


if __name__ == '__main__':
    unittest.main()

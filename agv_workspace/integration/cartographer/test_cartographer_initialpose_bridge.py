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


class CartographerInitialPoseBridgeTest(unittest.TestCase):
    def setUp(self):
        ros_modules = {
            'rclpy': Mock(),
            'rclpy.node': types.SimpleNamespace(Node=object),
            'rclpy.qos': types.SimpleNamespace(
                QoSProfile=Mock(),
                DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL='transient'),
                ReliabilityPolicy=types.SimpleNamespace(RELIABLE='RELIABLE')),
            'cartographer_ros_msgs.srv': types.SimpleNamespace(
                StartTrajectory=types.SimpleNamespace(Request=types.SimpleNamespace)),
            'geometry_msgs.msg': types.SimpleNamespace(
                PoseWithCovarianceStamped=object),
        }
        self.module = load('cartographer_initialpose_bridge', ros_modules)

    def test_apply_builds_relocation_request(self):
        bridge = self.module.CartographerInitialPoseBridge.__new__(
            self.module.CartographerInitialPoseBridge)
        bridge.config_dir = '/opt/carto/config'
        bridge.config_basename = 'feedbot_2d_localization.lua'
        bridge.get_logger = Mock(return_value=Mock())

        captured = {}

        def call_async(request):
            captured['request'] = request
            future = Mock()
            future.done.return_value = True
            future.result.return_value = types.SimpleNamespace(
                status=types.SimpleNamespace(code=0, message=''),
                trajectory_id=1)
            return future

        client = Mock()
        client.wait_for_service.return_value = True
        client.call_async.side_effect = call_async
        bridge.client = client

        pose = types.SimpleNamespace(position=types.SimpleNamespace(), orientation=types.SimpleNamespace())
        msg = types.SimpleNamespace(pose=types.SimpleNamespace(pose=pose))

        bridge.apply(msg)

        request = captured['request']
        self.assertTrue(request.use_initial_pose)
        self.assertEqual(request.relative_to_trajectory_id, 0)
        self.assertIs(request.initial_pose, pose)
        self.assertEqual(request.configuration_directory, '/opt/carto/config')
        self.assertEqual(request.configuration_basename, 'feedbot_2d_localization.lua')

    def test_apply_reports_service_unavailable(self):
        bridge = self.module.CartographerInitialPoseBridge.__new__(
            self.module.CartographerInitialPoseBridge)
        logger = Mock()
        bridge.get_logger = Mock(return_value=logger)
        client = Mock()
        client.wait_for_service.return_value = False
        bridge.client = client
        msg = types.SimpleNamespace(pose=types.SimpleNamespace(pose=types.SimpleNamespace()))
        bridge.apply(msg)
        client.call_async.assert_not_called()
        self.assertTrue(any('unavailable' in str(call) for call in logger.error.call_args_list))


if __name__ == '__main__':
    unittest.main()

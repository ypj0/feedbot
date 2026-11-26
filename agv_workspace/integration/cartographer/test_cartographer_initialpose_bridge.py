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
                FinishTrajectory=types.SimpleNamespace(Request=types.SimpleNamespace),
                GetTrajectoryStates=types.SimpleNamespace(Request=types.SimpleNamespace),
                StartTrajectory=types.SimpleNamespace(Request=types.SimpleNamespace)),
            'geometry_msgs.msg': types.SimpleNamespace(
                PoseWithCovarianceStamped=object),
        }
        self.module = load('cartographer_initialpose_bridge', ros_modules)

    def make_bridge(self):
        bridge = self.module.CartographerInitialPoseBridge.__new__(
            self.module.CartographerInitialPoseBridge)
        bridge.config_dir = '/opt/carto/config'
        bridge.config_basename = 'feedbot_2d_localization.lua'
        bridge.get_logger = Mock(return_value=Mock())
        bridge.active_trajectory_id = Mock(return_value=2)
        bridge.call = Mock()
        bridge.states_client = Mock()
        bridge.finish_client = Mock()
        bridge.start_client = Mock()
        return bridge

    def test_apply_finishes_active_then_starts_new(self):
        bridge = self.make_bridge()
        bridge.call.side_effect = [
            types.SimpleNamespace(trajectory_id=None),
            types.SimpleNamespace(trajectory_id=3),
        ]
        pose = types.SimpleNamespace(
            position=types.SimpleNamespace(x=1.0, y=2.0),
            orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
        msg = types.SimpleNamespace(pose=types.SimpleNamespace(pose=pose))

        bridge.apply(msg)

        self.assertEqual(bridge.call.call_count, 2)
        finish_request = bridge.call.call_args_list[0].args[1]
        start_request = bridge.call.call_args_list[1].args[1]
        self.assertEqual(finish_request.trajectory_id, 2)
        self.assertTrue(start_request.use_initial_pose)
        self.assertIs(start_request.initial_pose, pose)
        self.assertEqual(start_request.relative_to_trajectory_id, 0)
        self.assertEqual(start_request.configuration_directory, '/opt/carto/config')
        self.assertEqual(start_request.configuration_basename, 'feedbot_2d_localization.lua')

    def test_apply_skips_finish_without_active_trajectory(self):
        bridge = self.make_bridge()
        bridge.active_trajectory_id = Mock(return_value=None)
        bridge.call.return_value = types.SimpleNamespace(trajectory_id=3)
        pose = types.SimpleNamespace(position=types.SimpleNamespace(x=1.0, y=2.0),
                                     orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
        msg = types.SimpleNamespace(pose=types.SimpleNamespace(pose=pose))

        bridge.apply(msg)

        self.assertEqual(bridge.call.call_count, 1)
        start_request = bridge.call.call_args.args[1]
        self.assertTrue(start_request.use_initial_pose)

    def test_active_trajectory_id_finds_state_zero(self):
        bridge = self.make_bridge()
        bridge.call = Mock(return_value=types.SimpleNamespace(
            trajectory_states=types.SimpleNamespace(
                trajectory_id=[0, 1, 2], trajectory_state=[2, 2, 0])))
        self.assertEqual(
            self.module.CartographerInitialPoseBridge.active_trajectory_id(bridge), 2)

    def test_apply_reports_failure(self):
        bridge = self.make_bridge()
        bridge.call.side_effect = RuntimeError('finish failed')
        logger = bridge.get_logger()
        pose = types.SimpleNamespace(position=types.SimpleNamespace(x=1.0, y=2.0),
                                     orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
        msg = types.SimpleNamespace(pose=types.SimpleNamespace(pose=pose))
        bridge.apply(msg)
        self.assertTrue(any('finish failed' in str(call) for call in logger.error.call_args_list))


if __name__ == '__main__':
    unittest.main()

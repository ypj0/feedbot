import importlib.util
import json
from pathlib import Path
import sys
import tempfile
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


class LocalizationToolsTest(unittest.TestCase):
    def test_deactivation_order_and_stop_on_failure(self):
        activation = types.SimpleNamespace(change_state=Mock(return_value=0.1), require_state=Mock(return_value=2))
        module = load('deactivate_mapping', {'rclpy': Mock(), 'tf2_msgs.msg': Mock(),
                                            'activate_mapping': activation})
        report = {'passed': False, 'transitions': []}
        module.deactivate_mapping('node', report)
        self.assertEqual([call.args[1] for call in activation.change_state.call_args_list], [4, 2])
        self.assertEqual([call.args[1] for call in activation.require_state.call_args_list], [3, 2, 1])
        activation.change_state.reset_mock()
        activation.change_state.side_effect = RuntimeError('deactivate failed')
        with self.assertRaises(RuntimeError):
            module.deactivate_mapping('node', {'passed': False, 'transitions': []})
        self.assertEqual(activation.change_state.call_count, 1)

    def test_initial_pose_once_and_no_motion_publisher(self):
        def message():
            return types.SimpleNamespace(header=types.SimpleNamespace(), pose=types.SimpleNamespace(
                pose=types.SimpleNamespace(position=types.SimpleNamespace(), orientation=types.SimpleNamespace()),
                covariance=[0.0] * 36))
        publisher = Mock()
        publisher.get_subscription_count.return_value = 1
        publisher.wait_for_all_acked.return_value = True
        node = Mock()
        node.create_publisher.return_value = publisher
        ros = Mock()
        ros.create_node.return_value = node
        module = load('initialize_localization', {'rclpy': ros, 'rclpy.duration': Mock(),
                      'geometry_msgs.msg': types.SimpleNamespace(PoseWithCovarianceStamped=message)})
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'seed.json'
            path.write_text(json.dumps({'robot': {'x': 1, 'y': 2, 'yaw': 0}}))
            with patch.object(sys, 'argv', ['initialize_localization.py', str(path)]), patch('builtins.print') as output:
                module.main()
            self.assertEqual(json.loads(output.call_args.args[0])['source'], str(path.resolve()))
        self.assertEqual(node.create_publisher.call_args.args[1], '/initialpose')
        self.assertEqual(publisher.publish.call_count, 1)
        pose = publisher.publish.call_args.args[0]
        self.assertEqual(pose.pose.pose.position.x, 1)
        self.assertEqual(pose.pose.pose.orientation.w, 1)
        self.assertEqual(pose.pose.covariance[0], 0.25)

    def test_observer_never_initializes(self):
        text = (ROOT / 'localization_probe.py').read_text()
        self.assertNotIn('create_publisher', text)
        self.assertNotIn('publisher_gid', text)


if __name__ == '__main__':
    unittest.main()

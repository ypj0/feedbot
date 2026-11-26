import json
import unittest

import rclpy
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from lifecycle_node import ConvergenceNode


class LifecycleTrackingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = ConvergenceNode()
        self.stamp = self.node.get_clock().now().nanoseconds
        for name in ('scan', 'imu', 'wheel', 'filtered'):
            value = [0.] * 6 if name in ('wheel', 'filtered') else self.stamp
            self.node.store(name, value, source_ns=self.stamp)
        self.node.store('chassis', (0, 0, 0, 0))
        self.verification = dict(map_generation=1, map_digest='test-map', posterior_generation=1,
                                 posterior_source='test-amcl', observation_stamp_ns=self.stamp,
                                 phase='verified_pair', result={'accepted': True})
        self.node.verification(String(data=json.dumps(self.verification)))
        self.assertTrue(self.node.tracking_status()['localization_ready'])

    def tearDown(self):
        self.node.destroy_node()

    def test_invalid_then_valid_input_between_ticks_requires_new_pair(self):
        message = Imu()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.angular_velocity.z = float('nan')
        self.node.imu(message)
        message.angular_velocity.z = 0.
        self.node.imu(message)
        self.assertFalse(self.node.tracking_status()['localization_ready'])

    def test_invalid_source_timestamp_then_valid_input_requires_new_pair(self):
        self.node.store('imu', None, source_ns=self.stamp - 500000001)
        self.node.store('imu', None, source_ns=self.stamp)
        self.assertFalse(self.node.tracking_status()['localization_ready'])

    def test_long_input_receipt_gap_then_valid_input_requires_new_pair(self):
        self.node.inputs['imu']['received'] -= .6
        self.node.store('imu', None, source_ns=self.stamp)
        self.assertFalse(self.node.tracking_status()['localization_ready'])

    def test_rejection_then_old_verified_heartbeat_between_ticks_does_not_restore(self):
        rejected = dict(self.verification, phase='rejected', result={'accepted': False})
        self.node.verification(String(data=json.dumps(rejected)))
        self.node.verification(String(data=json.dumps(self.verification)))
        self.assertFalse(self.node.tracking_status()['localization_ready'])

    def test_pending_pair_can_become_verified_without_resetting_epoch(self):
        pending = dict(self.verification, phase='pending_convergence',
                       result={'accepted': False, 'pending_convergence': True})
        self.node.verification(String(data=json.dumps(pending)))
        self.assertFalse(self.node.tracking_status()['localization_ready'])
        self.node.verification(String(data=json.dumps(self.verification)))
        self.assertTrue(self.node.tracking_status()['localization_ready'])

    def test_tracking_only_imu_input_does_not_change_acquisition_contract(self):
        self.node.state.start(0., self.node.view())
        message = Imu()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.angular_velocity.z = float('nan')
        self.node.imu(message)
        self.assertEqual(self.node.state.phase, 'running')
        self.assertFalse(self.node.tracking_status()['localization_ready'])

    def test_map_or_source_roundtrip_between_ticks_requires_new_pair(self):
        for change in (dict(map_generation=2, map_digest='new-map'),
                       dict(posterior_generation=2, posterior_source='new-amcl')):
            self.verification['observation_stamp_ns'] = self.node.get_clock().now().nanoseconds
            self.node.verification(String(data=json.dumps(self.verification)))
            self.assertTrue(self.node.tracking_status()['localization_ready'])
            self.node.verification(String(data=json.dumps(dict(self.verification, **change))))
            self.node.verification(String(data=json.dumps(self.verification)))
            self.assertFalse(self.node.tracking_status()['localization_ready'])


if __name__ == '__main__':
    unittest.main()

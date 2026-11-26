import json
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty

from lifecycle_node import ConvergenceNode, stamp_ns


def main():
    rclpy.init()
    lifecycle = ConvergenceNode()
    fixture = rclpy.create_node('tracking_fixture')
    executor = SingleThreadedExecutor()
    executor.add_node(lifecycle)
    executor.add_node(fixture)
    publishers = {
        'scan': fixture.create_publisher(LaserScan, '/scan', qos_profile_sensor_data),
        'imu': fixture.create_publisher(Imu, '/imu', qos_profile_sensor_data),
        'wheel': fixture.create_publisher(Odometry, '/wheel/odom', qos_profile_sensor_data),
        'filtered': fixture.create_publisher(Odometry, '/odometry/filtered', qos_profile_sensor_data),
        'diagnostics': fixture.create_publisher(DiagnosticArray, '/diagnostics', 10),
        'verification': fixture.create_publisher(String, '/localization/verification', 10)}
    calls = []
    received = []
    state = dict(imu=True, new_pair=True, posterior_stamp=0, phase='verified_pair',
                 generation=1, moving=True)

    def request_nomotion(request, response):
        calls.append(time.monotonic())
        return response

    service = fixture.create_service(Empty, '/request_nomotion_update', request_nomotion)
    subscription = fixture.create_subscription(String, '/localization/convergence',
        lambda message: received.append(json.loads(message.data)), 10)

    def publish():
        stamp = fixture.get_clock().now().to_msg()
        scan = LaserScan()
        scan.header.stamp = stamp
        publishers['scan'].publish(scan)
        if state['imu']:
            imu = Imu()
            imu.header.stamp = stamp
            publishers['imu'].publish(imu)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.twist.twist.linear.x = .1 if state['moving'] else 0.
        publishers['wheel'].publish(odom)
        publishers['filtered'].publish(odom)
        fields = dict(initialized='1', shutdown='0', emergency='0', left_alarm='0', right_alarm='0',
                      command_nonzero_count='4', feedback_timeout_count='0',
                      runtime_field_error_count='0', sync_checksum_rejected_bytes='0')
        diagnostic = DiagnosticStatus(name='feedbot_chassis/MCU', level=bytes([0]))
        diagnostic.values = [KeyValue(key=key, value=value) for key, value in fields.items()]
        publishers['diagnostics'].publish(DiagnosticArray(status=[diagnostic]))
        if state['new_pair']:
            state['posterior_stamp'] = stamp_ns(stamp)
        publishers['verification'].publish(String(data=json.dumps(dict(
            map_generation=1, map_digest='fixture-map', posterior_generation=state['generation'],
            posterior_source='fixture-amcl-' + str(state['generation']), phase=state['phase'],
            observation_stamp_ns=state['posterior_stamp'],
            result=dict(accepted=state['phase'] == 'verified_pair')))))

    timer = fixture.create_timer(.05, publish)

    def wait_for(predicate, seconds=3):
        baseline = len(received)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
            if len(received) > baseline and predicate(received[-1]['tracking']):
                return received[-1]
        raise AssertionError(received[-1] if received else 'no tracking status received')

    try:
        current = wait_for(lambda tracking: tracking['localization_ready'])
        assert current['phase'] == 'idle'
        assert lifecycle.view()['error'] == 'nonstationary_wheel'
        assert lifecycle.view(stationary=False)['error'] is None
        assert not current['tracking']['motion_authorized']
        state.update(imu=False, new_pair=False)
        wait_for(lambda tracking: tracking['reason'] == 'missing_or_stale_imu')
        state['imu'] = True
        wait_for(lambda tracking: tracking['reason'] == 'post_invalidation_pair_required')
        state['new_pair'] = True
        wait_for(lambda tracking: tracking['localization_ready'])
        state.update(generation=2, new_pair=False)
        wait_for(lambda tracking: not tracking['localization_ready'])
        state['new_pair'] = True
        wait_for(lambda tracking: tracking['localization_ready'] and tracking['source_key'][0] == 2)
        state['phase'] = 'pending_convergence'
        wait_for(lambda tracking: tracking['reason'] == 'verification_pending_convergence')
        state['phase'] = 'verified_pair'
        wait_for(lambda tracking: tracking['localization_ready'])
        state.update(new_pair=False, moving=False)
        wait_for(lambda tracking: tracking['reason'] == 'posterior_source_time_invalid')
        assert not calls
        assert lifecycle.state.phase == 'idle'
        assert all(not status['tracking']['motion_authorized'] for status in received)
        print(json.dumps(dict(passed=True, status_messages=len(received), nomotion_requests=len(calls),
                              scenarios=['moving', 'imu_interruption', 'old_pair_rejected',
                                         'new_pair_recovery', 'source_replacement',
                                         'pending_convergence', 'source_age_not_heartbeat'])))
    finally:
        lifecycle.clear_future()
        executor.shutdown()
        lifecycle.destroy_node()
        fixture.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

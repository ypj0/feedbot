import json
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger

from lifecycle_node import ConvergenceNode, stamp_ns


def run_case(produce_updates, same_scan=False, acquisition=False, updates_to_verify=2):
    lifecycle = ConvergenceNode()
    fixture = rclpy.create_node('convergence_fixture')
    executor = SingleThreadedExecutor()
    executor.add_node(fixture)
    executor.add_node(lifecycle)
    publishers = {
        'scan': fixture.create_publisher(LaserScan, '/scan', qos_profile_sensor_data),
        'wheel': fixture.create_publisher(Odometry, '/wheel/odom', qos_profile_sensor_data),
        'filtered': fixture.create_publisher(Odometry, '/odometry/filtered', qos_profile_sensor_data),
        'diagnostics': fixture.create_publisher(DiagnosticArray, '/diagnostics', 10),
        'verification': fixture.create_publisher(String, '/localization/verification', 10)}
    calls = []
    state = dict(stamp=1, phase='waiting_pair', update=False)

    def update(request, response):
        calls.append(time.monotonic())
        state['update'] = produce_updates
        return response

    service = fixture.create_service(Empty, '/request_nomotion_update', update)
    starter = fixture.create_client(Trigger, '/localization/acquire' if acquisition else '/localization/converge')

    def publish():
        stamp = fixture.get_clock().now().to_msg()
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'laser'
        publishers['scan'].publish(scan)
        odom = Odometry()
        odom.header.stamp = stamp
        publishers['wheel'].publish(odom)
        publishers['filtered'].publish(odom)
        fields = dict(initialized='1', shutdown='0', emergency='0', left_alarm='0', right_alarm='0',
                      command_nonzero_count='0', feedback_timeout_count='0',
                      runtime_field_error_count='0', sync_checksum_rejected_bytes='0')
        diagnostic = DiagnosticStatus(name='feedbot_chassis/MCU', level=bytes([1]),
                                      message='isolated fixture; not hardware')
        diagnostic.values = [KeyValue(key=key, value=value) for key, value in fields.items()]
        publishers['diagnostics'].publish(DiagnosticArray(status=[diagnostic]))
        if state['update']:
            state['stamp'] = lifecycle.state.request_scan_stamp if same_scan else stamp_ns(stamp)
            state['phase'] = 'verified_pair' if len(calls) >= updates_to_verify else 'pending_convergence'
            state['update'] = False
        result = dict(accepted=state['phase'] == 'verified_pair', pending_convergence=True,
                      innovation={'compatible': True})
        publishers['verification'].publish(String(data=json.dumps(dict(
            map_generation=1, map_digest='fixture-map', posterior_generation=1,
            posterior_source='fixture-amcl', phase=state['phase'],
            observation_stamp_ns=state['stamp'], result=result))))

    timer = fixture.create_timer(.05, publish)
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
            if lifecycle.view()['error'] is None and lifecycle.client.service_is_ready() and starter.service_is_ready():
                break
        assert lifecycle.view()['error'] is None
        assert not calls
        start = starter.call_async(Trigger.Request())
        deadline = time.monotonic() + (updates_to_verify * 1.01 + 3 if produce_updates else 4)
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
            if start.done() and lifecycle.state.phase in ('succeeded', 'failed'):
                break
        assert start.done() and start.result().success
        if produce_updates:
            assert lifecycle.state.phase == 'succeeded', lifecycle.state.status()
            assert len(calls) == updates_to_verify
            assert all(second - first >= 1 for first, second in zip(calls, calls[1:]))
            assert lifecycle.state.confirmed == updates_to_verify
        else:
            assert lifecycle.state.reason == 'new_pair_timeout', lifecycle.state.status()
            assert len(calls) == 1
        deadline = time.monotonic() + 1.2
        call_count = len(calls)
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        assert len(calls) == call_count
        assert lifecycle.state.operation == ('acquire' if acquisition else 'refresh')
        assert not lifecycle.state.status()['localization_accepted']
        print(json.dumps(dict(updates=produce_updates, same_scan=same_scan, acquisition=acquisition,
                              calls=len(calls), status=lifecycle.state.status())), flush=True)
    finally:
        lifecycle.clear_future()
        executor.shutdown()
        lifecycle.destroy_node()
        fixture.destroy_node()


def main():
    rclpy.init()
    try:
        run_case(True)
        run_case(True, same_scan=True)
        run_case(False)
        run_case(True, acquisition=True, updates_to_verify=15)
        run_case(False, acquisition=True)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

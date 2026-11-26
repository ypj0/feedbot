import json
import math
import signal
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger

from authorization import NavigationAuthorization
from convergence import Convergence
from converge_gate import ConvergeGate
from tracking import Tracking


def stamp_ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


class ConvergenceNode(Node):
    def __init__(self):
        super().__init__('localization_lifecycle')
        self.state = Convergence()
        self.tracking = Tracking()
        self.converge_gate = ConvergeGate()
        self.authorization = NavigationAuthorization()
        self.inputs = {}
        self.future = None
        self.last_status = None
        self.client = self.create_client(Empty, '/request_nomotion_update')
        self.publisher = self.create_publisher(String, '/localization/convergence', 1)
        self.service = self.create_service(Trigger, '/localization/converge', self.start_request)
        self.acquire_service = self.create_service(Trigger, '/localization/acquire', self.start_acquisition)
        self.confirm_service = self.create_service(Trigger, '/localization/confirm', self.start_confirmation)
        self.authorize_service = self.create_service(Trigger, '/navigation/authorize', self.authorize_request)
        self.revoke_service = self.create_service(Trigger, '/navigation/revoke', self.revoke_request)
        self.subscriptions_owned = [
            self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data),
            self.create_subscription(Imu, '/imu', self.imu, qos_profile_sensor_data),
            self.create_subscription(Odometry, '/wheel/odom',
                                     lambda message: self.odometry('wheel', message), qos_profile_sensor_data),
            self.create_subscription(Odometry, '/odometry/filtered',
                                     lambda message: self.odometry('filtered', message), qos_profile_sensor_data),
            self.create_subscription(DiagnosticArray, '/diagnostics', self.diagnostics, 10),
            self.create_subscription(String, '/localization/verification', self.verification, 10)]
        self.timer = self.create_timer(.02, self.tick)
        self.status_timer = self.create_timer(.2, self.publish_status)

    def store(self, name, value, error=None, source_ns=None):
        received = time.monotonic()
        previous = self.inputs.get(name)
        ros_now_ns = self.get_clock().now().nanoseconds
        source_time_invalid = source_ns is not None and not 0 <= ros_now_ns - source_ns < 500000000
        receipt_gap = previous is not None and received - previous['received'] >= .5
        if source_time_invalid or receipt_gap:
            self.tracking.invalidate(ros_now_ns)
        self.inputs[name] = dict(value=value, error=error, received=received, source_ns=source_ns)
        if error:
            self.tracking.invalidate(self.get_clock().now().nanoseconds)
            if name != 'imu':
                self.state.fail(error)
        elif name in ('chassis', 'verification'):
            self.tracking_status()

    def scan(self, message):
        self.store('scan', stamp_ns(message.header.stamp), source_ns=stamp_ns(message.header.stamp))

    def imu(self, message):
        gyro = message.angular_velocity
        error = None if all(math.isfinite(value) for value in (gyro.x, gyro.y, gyro.z)) else 'invalid_imu'
        self.store('imu', None, error, stamp_ns(message.header.stamp))

    def odometry(self, name, message):
        twist = message.twist.twist
        values = [getattr(vector, axis) for vector in (twist.linear, twist.angular)
                  for axis in ('x', 'y', 'z')]
        error = 'invalid_' + name if any(not math.isfinite(value) for value in values) else None
        self.store(name, values, error, stamp_ns(message.header.stamp))
        if any(abs(value) > .01 for value in values):
            self.state.fail('nonstationary_' + name)

    def diagnostics(self, message):
        for status in message.status:
            if status.name != 'feedbot_chassis/MCU':
                continue
            try:
                fields = {value.key: value.value for value in status.values}
                counters = tuple(int(fields[key]) for key in (
                    'command_nonzero_count', 'feedback_timeout_count',
                    'runtime_field_error_count', 'sync_checksum_rejected_bytes'))
                error = None
                if (int.from_bytes(status.level, 'little') >= 2 or fields['initialized'] != '1' or
                        fields['shutdown'] != '0' or any(fields[key] != '0' for key in (
                            'emergency', 'left_alarm', 'right_alarm'))):
                    error = 'chassis_not_ready: ' + status.message
                if self.state.phase == 'running' and counters != self.state.counters:
                    self.state.fail('chassis_fault_or_command_counter_changed')
                self.store('chassis', counters, error)
            except (KeyError, ValueError, TypeError) as error:
                self.store('chassis', None, 'invalid_chassis_diagnostics: ' + str(error))

    def verification(self, message):
        try:
            value = json.loads(message.data)
            for key in ('phase', 'map_generation', 'map_digest', 'observation_stamp_ns', 'result',
                        'posterior_generation', 'posterior_source'):
                value[key]
            previous = self.inputs.get('verification', {}).get('value')
            if previous is not None:
                previous_identity = ((previous['map_generation'], previous['map_digest']),
                                     (previous['posterior_generation'], previous['posterior_source']))
                current_identity = ((value['map_generation'], value['map_digest']),
                                    (value['posterior_generation'], value['posterior_source']))
                if current_identity != previous_identity:
                    self.tracking.invalidate(self.get_clock().now().nanoseconds)
            if value['phase'] in ('error', 'rejected'):
                self.tracking.invalidate(self.get_clock().now().nanoseconds)
            self.store('verification', value)
            wheel = self.inputs.get('wheel', {}).get('value') or [0.] * 6
            filtered = self.inputs.get('filtered', {}).get('value') or [0.] * 6
            self.converge_gate.update(value, list(wheel) + list(filtered))
            if (self.state.phase == 'running' and
                    (value['map_generation'], value['map_digest']) != self.state.map_key):
                self.state.fail('map_changed')
            if (self.state.phase == 'running' and
                    (value['posterior_generation'], value['posterior_source']) != self.state.source_key):
                self.state.fail('posterior_source_changed')
        except (KeyError, TypeError, ValueError) as error:
            self.store('verification', None, 'invalid_verification: ' + str(error))

    def view(self, stationary=True):
        error = None
        now = time.monotonic()
        ros_now_ns = self.get_clock().now().nanoseconds
        names = ('scan', 'wheel', 'filtered', 'chassis', 'verification')
        for name in names + (() if stationary else ('imu',)):
            entry = self.inputs.get(name)
            if entry is None or now - entry['received'] >= .5:
                error = 'missing_or_stale_' + name
                break
            if entry['error']:
                error = entry['error']
                break
            if stationary and name in ('wheel', 'filtered') and any(abs(value) > .01 for value in entry['value']):
                error = 'nonstationary_' + name
                break
            if entry['source_ns'] is not None and not 0 <= ros_now_ns - entry['source_ns'] < 500000000:
                error = 'invalid_source_time_' + name
                break
        if error:
            return dict(error=error)
        verification = self.inputs['verification']['value']
        return dict(error=None, map_key=(verification['map_generation'], verification['map_digest']),
                    source_key=(verification['posterior_generation'], verification['posterior_source']),
                    phase=verification['phase'], result=verification['result'],
                    observation_stamp_ns=verification['observation_stamp_ns'],
                    scan_stamp_ns=self.inputs['scan']['value'], counters=self.inputs['chassis']['value'])

    def clear_future(self):
        if self.future is not None:
            self.client.remove_pending_request(self.future)
            self.future.cancel()
            self.future = None

    def start_confirmation(self, request, response):
        try:
            if self.converge_gate.phase in ('running', 'converged'):
                raise ValueError('confirmation already active or completed')
            self.converge_gate.start()
            response.success = True
            response.message = ('accepted; wait for covariance convergence and consecutive '
                                'dynamic-robust verification, not navigation authorization')
        except ValueError as error:
            response.success, response.message = False, str(error)
        self.publish_status()
        return response

    def authorize_request(self, request, response):
        try:
            ready = self.converge_gate.status()['localization_ready']
            self.authorization.request(self.get_clock().now().nanoseconds, ready)
            response.success = True
            response.message = ('accepted; navigation authorization is separate from '
                                'localization correctness and remains revocable')
        except ValueError as error:
            response.success, response.message = False, str(error)
        self.publish_status()
        return response

    def revoke_request(self, request, response):
        self.authorization.revoke('operator_revoke')
        response.success = True
        response.message = 'authorization revoked'
        self.publish_status()
        return response

    def start_acquisition(self, request, response):
        return self.start_request(request, response, operation='acquire')

    def start_request(self, request, response, operation='refresh'):
        try:
            if self.state.phase == 'running':
                raise ValueError('convergence already running')
            if not self.client.service_is_ready():
                raise ValueError('nomotion service unavailable')
            self.clear_future()
            self.state.start(time.monotonic(), self.view(), operation=operation)
            response.success = True
            response.message = 'accepted; follow /localization/convergence, not navigation authorization'
        except ValueError as error:
            response.success, response.message = False, str(error)
        self.publish_status()
        return response

    def tick(self):
        self.tracking_status()
        ready = self.converge_gate.status()['localization_ready']
        view = self.view(stationary=False)
        self.authorization.update(ready, fault=view.get('error') is not None)
        if self.state.phase != 'running':
            self.clear_future()
            return
        if self.future is not None and self.future.done():
            self.state.response(not self.future.cancelled() and self.future.exception() is None)
            self.future = None
        if self.state.step(time.monotonic(), self.view()):
            if not self.client.service_is_ready():
                self.state.fail('nomotion service unavailable')
            elif time.monotonic() >= self.state.deadline:
                self.state.fail('budget_exhausted')
            else:
                try:
                    self.future = self.client.call_async(Empty.Request())
                except Exception as error:
                    self.state.fail('nomotion_send_failed: ' + str(error))
        if self.state.phase != 'running':
            self.clear_future()
            self.publish_status()

    def tracking_status(self):
        return self.tracking.update(self.get_clock().now().nanoseconds, self.view(stationary=False),
                                    acquiring=self.state.phase == 'running')

    def publish_status(self):
        status = self.state.status()
        status['tracking'] = self.tracking_status()
        status['confirm'] = self.converge_gate.status()
        status['authorization'] = self.authorization.status()
        view = self.view()
        status['input_error'] = view['error']
        status['observed_pair_stamp_ns'] = view.get('observation_stamp_ns')
        status['observed_scan_stamp_ns'] = view.get('scan_stamp_ns')
        status['observed_verification_phase'] = view.get('phase')
        status['service_ready'] = self.client.service_is_ready()
        payload = json.dumps(status, ensure_ascii=False, allow_nan=False)
        self.publisher.publish(String(data=payload))
        transition = json.dumps({key: value for key, value in status.items()
                                 if not key.startswith('observed_') and key != 'tracking'},
                                ensure_ascii=False, allow_nan=False)
        if transition != self.last_status:
            self.get_logger().info(payload)
            self.last_status = transition


def main():
    rclpy.init()
    node = ConvergenceNode()
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        node.state.fail('lifecycle_stopped')
        node.clear_future()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

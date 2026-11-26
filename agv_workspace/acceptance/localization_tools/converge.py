import json
from pathlib import Path
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    boundary = json.loads(Path('initialization.json').read_text())['before_initialization_ns']
    rclpy.init()
    node = rclpy.create_node('convergence_lifecycle_acceptance_53')
    output = Path('messages.jsonl').open('x')
    latest = {}

    def record(topic, message):
        payload = message_to_ordereddict(message)
        latest[topic] = payload
        output.write(json.dumps(dict(topic=topic, monotonic_s=time.monotonic(),
                                     ros_now_ns=node.get_clock().now().nanoseconds,
                                     message=payload)) + '\n')

    subscriptions = []
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for topic, message_type, qos in (
            ('/scan', LaserScan, qos_profile_sensor_data),
            ('/amcl_pose', PoseWithCovarianceStamped, retained),
            ('/wheel/odom', Odometry, qos_profile_sensor_data),
            ('/odometry/filtered', Odometry, qos_profile_sensor_data),
            ('/diagnostics', DiagnosticArray, 10),
            ('/localization/verification', String, 10),
            ('/localization/convergence', String, 10)):
        subscriptions.append(node.create_subscription(
            message_type, topic, lambda message, topic=topic: record(topic, message), qos))
    client = node.create_client(Trigger, '/localization/acquire')
    try:
        deadline = time.monotonic() + 10
        status = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            message = latest.get('/localization/convergence')
            verification_message = latest.get('/localization/verification')
            if message and verification_message:
                status = json.loads(message['data'])
                verification = json.loads(verification_message['data'])
                result = verification.get('result') or {}
                compatible = (result.get('accepted') or
                              (result.get('pending_convergence') and
                               (result.get('innovation') or {}).get('compatible')))
                posterior = latest.get('/amcl_pose')
                posterior_stamp = posterior['header']['stamp'] if posterior else {}
                posterior_ns = posterior_stamp.get('sec', 0) * 1000000000 + posterior_stamp.get('nanosec', 0)
                if (compatible and verification.get('posterior_source') and
                        verification['observation_stamp_ns'] == posterior_ns and posterior_ns >= boundary and
                        verification['phase'] in ('waiting_pair', 'pending_convergence', 'verified_pair') and
                        status['phase'] == 'idle' and status['input_error'] is None and
                        status['service_ready'] and client.service_is_ready() and
                        '/amcl_pose' in latest and '/scan' in latest):
                    break
        else:
            raise RuntimeError('lifecycle preflight not ready: ' + repr(status))
        print(json.dumps(dict(event='single_start_request', monotonic_s=time.monotonic(),
                              baseline_pose=latest['/amcl_pose'])), flush=True)
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 32
        terminal = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            status = json.loads(latest['/localization/convergence']['data'])
            if future.done():
                if future.exception() or not future.result().success:
                    raise RuntimeError('start rejected: ' + str(future.exception() or future.result().message))
                if status['phase'] in ('succeeded', 'failed'):
                    terminal = status
                    break
        if terminal is None:
            raise RuntimeError('no terminal lifecycle result received')
        print(json.dumps(dict(event='terminal', monotonic_s=time.monotonic(), status=terminal)), flush=True)
        deadline = time.monotonic() + 1.2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        final = json.loads(latest['/localization/convergence']['data'])
        assert final['requests'] == terminal['requests']
        assert not final['localization_accepted']
        assert final['operation'] == 'acquire' and final['budget_s'] == 30.
        if terminal['phase'] != 'succeeded':
            raise RuntimeError('lifecycle failed: ' + terminal['reason'])
        print(json.dumps(dict(event='complete', status=final)), flush=True)
    finally:
        output.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

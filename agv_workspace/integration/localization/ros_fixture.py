import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from test_global_localization import fixture

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'dashboard/localization'))
from car_bridge import State


def main():
    rclpy.init()
    node = rclpy.create_node('verification_fixture')
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    map_publisher = node.create_publisher(OccupancyGrid, '/map', retained)
    pose_publisher = node.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', retained)
    scan_publisher = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    status = {}
    web = State()

    def received(message):
        value = json.loads(message.data)
        status.clear()
        status.update(value)
        if value['map_digest']:
            web.set_map(dict(digest=value['map_digest']), node.get_clock().now().nanoseconds)
        web.put('verification', value)
        web.put('scan', {})
        web.put('robot_tf', {})

    subscriber = node.create_subscription(String, '/localization/verification',
                                          received, 1)
    broadcaster = StaticTransformBroadcaster(node)
    grid, points, pose = fixture()
    map_message = OccupancyGrid()
    map_message.header.frame_id = 'map'
    map_message.info.width, map_message.info.height = grid['width'], grid['height']
    map_message.info.resolution = grid['resolution']
    map_message.info.origin.position.x, map_message.info.origin.position.y = grid['origin'][:2]
    map_message.info.origin.orientation.z = math.sin(grid['origin'][2] / 2)
    map_message.info.origin.orientation.w = math.cos(grid['origin'][2] / 2)
    map_message.data = grid['data']
    scan = LaserScan()
    scan.header.frame_id = 'scan'
    scan.angle_min, scan.angle_increment = -math.pi, 2 * math.pi / len(points)
    scan.range_min, scan.range_max = .01, 10.
    scan.ranges = [math.hypot(point[0] - .27, point[1]) for point in points]
    posterior = PoseWithCovarianceStamped()
    posterior.header.frame_id = 'map'
    posterior.pose.pose.position.x, posterior.pose.pose.position.y = pose[:2]
    posterior.pose.pose.orientation.z = math.sin(pose[2] / 2)
    posterior.pose.pose.orientation.w = math.cos(pose[2] / 2)
    for index in (0, 7, 35):
        posterior.pose.covariance[index] = .001
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'base_link', 'scan'
    transform.transform.translation.x = .27
    transform.transform.rotation.w = 1.
    broadcaster.sendTransform(transform)
    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen([sys.executable, '-B', str(Path(__file__).with_name('observer.py'))],
                                   stdout=log, stderr=log)
        try:
            deadline = time.monotonic() + 6
            while map_publisher.get_subscription_count() != 1 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            assert map_publisher.get_subscription_count() == 1, 'observer did not subscribe'
            map_publisher.publish(map_message)
            deadline = time.monotonic() + 6
            while time.monotonic() < deadline and not status.get('pair_verified'):
                scan.header.stamp = node.get_clock().now().to_msg()
                posterior.header.stamp = scan.header.stamp
                scan_publisher.publish(scan)
                pose_publisher.publish(posterior)
                rclpy.spin_once(node, timeout_sec=.1)
                time.sleep(.05)
            assert status.get('pair_verified'), status
            assert not status['localization_accepted']
            displayed = web.snapshot()
            assert displayed['tracking_valid'] and not displayed['motion_authorized'], displayed
            assert displayed['amcl']['stamp_ns'] == status['observation_stamp_ns']
            assert displayed['amcl']['covariance'] == list(posterior.pose.covariance)
            assert displayed['amcl']['x'] == posterior.pose.pose.position.x
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                scan.header.stamp = node.get_clock().now().to_msg()
                scan_publisher.publish(scan)
                rclpy.spin_once(node, timeout_sec=.1)
                time.sleep(.05)
            assert status['phase'] == 'waiting_pair', status
            assert not status['pair_verified']
            assert not web.snapshot()['tracking_valid']
            assert web.snapshot()['amcl']['observation_age_s'] >= 2
            deadline = time.monotonic() + 6
            while time.monotonic() < deadline and not status.get('pair_verified'):
                scan.header.stamp = node.get_clock().now().to_msg()
                posterior.header.stamp = scan.header.stamp
                scan_publisher.publish(scan)
                pose_publisher.publish(posterior)
                rclpy.spin_once(node, timeout_sec=.05)
            assert status.get('pair_verified'), status
            old_source = status['posterior_source']
            old_generation = status['posterior_generation']
            node.destroy_publisher(pose_publisher)
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline and status.get('posterior_source') is not None:
                scan.header.stamp = node.get_clock().now().to_msg()
                scan_publisher.publish(scan)
                rclpy.spin_once(node, timeout_sec=.05)
            assert status.get('posterior_source') is None, status
            assert status['result'] is None and not status['pair_verified'], status
            assert status['posterior'] is None and 'amcl' not in web.snapshot()
            assert status['posterior_generation'] > old_generation, status
            pose_publisher = node.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', retained)
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline and status.get('posterior_source') is None:
                scan.header.stamp = node.get_clock().now().to_msg()
                scan_publisher.publish(scan)
                pose_publisher.publish(posterior)
                rclpy.spin_once(node, timeout_sec=.05)
            assert status.get('posterior_source') not in (None, old_source), status
            assert not status['pair_verified'] and status['result'] is None, status
            assert 'amcl' not in web.snapshot()
            deadline = time.monotonic() + 6
            while time.monotonic() < deadline and not status.get('pair_verified'):
                scan.header.stamp = node.get_clock().now().to_msg()
                posterior.header.stamp = scan.header.stamp
                scan_publisher.publish(scan)
                pose_publisher.publish(posterior)
                rclpy.spin_once(node, timeout_sec=.05)
            assert status.get('pair_verified'), status
            assert not status['localization_accepted']
            assert web.snapshot()['tracking_valid']
        finally:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=10)
            log.seek(0)
            print(log.read())
            node.destroy_node()
            rclpy.shutdown()
        assert process.returncode == 0, process.returncode
    print('PASS: real ROS pairing, publisher replacement invalidation, new-pair recovery, clean shutdown')


if __name__ == '__main__':
    main()

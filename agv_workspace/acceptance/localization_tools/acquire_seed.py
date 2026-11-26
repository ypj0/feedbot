import json
import math
from pathlib import Path
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import LaserScan

sys.path.insert(0, '/localization')
from global_localization import search


def stamp_ns(message):
    return message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec


def points(scan):
    if scan.header.frame_id != 'scan' or len(scan.ranges) != 1075:
        raise RuntimeError('unexpected lidar frame or beam count')
    return [[.27 + distance * math.cos(scan.angle_min + index * scan.angle_increment),
             distance * math.sin(scan.angle_min + index * scan.angle_increment)]
            for index, distance in enumerate(scan.ranges)
            if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max]


def main():
    rclpy.init()
    node = rclpy.create_node('fresh_localization_seed_64')
    latest = {}
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = []
    for name, topic, message_type, qos in (
            ('map', '/map', OccupancyGrid, retained),
            ('scan', '/scan', LaserScan, qos_profile_sensor_data),
            ('wheel', '/wheel/odom', Odometry, qos_profile_sensor_data),
            ('filtered', '/odometry/filtered', Odometry, qos_profile_sensor_data)):
        subscriptions.append(node.create_subscription(
            message_type, topic, lambda message, name=name: latest.update({name: message}), qos))

    def ready(after=0):
        if len(latest) != 4 or stamp_ns(latest['scan']) <= after:
            return False
        now = node.get_clock().now().nanoseconds
        return all(0 <= now - stamp_ns(latest[name]) < 500000000
                   for name in ('scan', 'wheel', 'filtered'))

    def wait_inputs(after=0):
        deadline = time.monotonic() + 8
        while not ready(after) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        if not ready(after):
            raise RuntimeError('fresh mapping inputs unavailable')

    try:
        wait_inputs()
        message = latest['map']
        origin = message.info.origin
        if message.header.frame_id != 'map' or origin.orientation.w != 1.0:
            raise RuntimeError('unexpected saved map frame or orientation')
        grid = dict(width=message.info.width, height=message.info.height,
                    resolution=message.info.resolution,
                    origin=[origin.position.x, origin.position.y, 0.], data=list(message.data))
        first = latest['scan']
        candidate = search(grid, points(first))
        Path('candidate.json').write_text(json.dumps(dict(
            map=message_to_ordereddict(message), scan=message_to_ordereddict(first),
            result=candidate), ensure_ascii=False) + '\n')
        position = candidate['pose'] if candidate.get('pose') else None
        seed = dict(robot=None if position is None else
                    dict(x=position[0], y=position[1], yaw=position[2]),
                    scan_stamp_ns=stamp_ns(first), created_monotonic=time.monotonic(),
                    candidate_accepted=bool(candidate.get('accepted')),
                    candidate_score=candidate.get('score'),
                    candidate_reason=candidate.get('reason'),
                    covariance_source='AMCL global search prior, large initializer covariance; '
                                      'not calibrated, converges with motion')
        Path('seed.json').write_text(json.dumps(seed) + '\n')
        print(json.dumps(seed), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

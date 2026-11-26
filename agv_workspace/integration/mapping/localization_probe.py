import json
import math
from pathlib import Path
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


def main():
    seed = json.loads(Path(sys.argv[1]).read_text())
    output = Path(sys.argv[2])
    rclpy.init()
    node = rclpy.create_node('feedbot_localization_probe')
    report = {'initial_pose_source': 'last live SLAM map pose; not independent ground truth',
              'initial_pose': seed['robot'], 'poses': [], 'maps': [], 'scan_count': 0, 'edges': []}

    def pose(message):
        position = message.pose.pose.position
        rotation = message.pose.pose.orientation
        report['poses'].append({'stamp_s': message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                               'x': position.x, 'y': position.y,
                               'yaw': math.atan2(2 * rotation.w * rotation.z, 1 - 2 * rotation.z**2),
                               'covariance': list(message.pose.covariance)})

    def occupancy(message):
        report['maps'].append({'width': message.info.width, 'height': message.info.height,
                               'resolution': message.info.resolution,
                               'origin': [message.info.origin.position.x, message.info.origin.position.y],
                               'occupied': sum(value == 100 for value in message.data),
                               'free': sum(value == 0 for value in message.data),
                               'unknown': sum(value == -1 for value in message.data)})

    def scan(message):
        report['scan_count'] += 1

    def transforms(message):
        for transform in message.transforms:
            if (transform.header.frame_id, transform.child_frame_id) in (
                    ('map', 'odom'), ('odom', 'base_link')):
                report['edges'].append({'parent': transform.header.frame_id,
                                        'child': transform.child_frame_id,
                                        'x': transform.transform.translation.x,
                                        'y': transform.transform.translation.y,
                                        'stamp_s': transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9})

    subscriptions = [
        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', pose,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(OccupancyGrid, '/map', occupancy,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data),
        node.create_subscription(TFMessage, '/tf', transforms, 100)]
    try:
        report['observer_only'] = True
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        report['endpoints'] = {
            topic: [{'node': item.node_name, 'gid': bytes(item.endpoint_gid).hex()}
                    for item in node.get_publishers_info_by_topic(topic)]
            for topic in ('/tf', '/map', '/amcl_pose')}
        report['tf_provenance'] = 'Verify existing C++ tf_edge_probe GIDs against endpoints; not rclpy callback metadata'
        if not report['poses'] or not report['maps'] or report['scan_count'] == 0:
            raise RuntimeError('missing AMCL pose, loaded map or scans')
        if [item['node'] for item in report['endpoints']['/map']] != ['map_server']:
            raise RuntimeError('map_server is not the sole map publisher')
        if not all(math.isfinite(value) for pose_value in report['poses']
                   for value in (pose_value['x'], pose_value['y'], pose_value['yaw'], *pose_value['covariance'])):
            raise RuntimeError('nonfinite AMCL pose')
        report['passed'] = True
    except Exception as error:
        report.update(passed=False, error=str(error))
        raise
    finally:
        output.write_text(json.dumps(report, indent=2))
        print(json.dumps({key: value for key, value in report.items() if key not in ('edges', 'poses')}))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

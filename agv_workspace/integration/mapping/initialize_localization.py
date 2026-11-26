import json
import math
from pathlib import Path
import sys
import time

import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped


def main():
    seed = json.loads(Path(sys.argv[1]).read_text())['robot']
    rclpy.init()
    node = rclpy.create_node('feedbot_localization_initial_pose')
    publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    try:
        deadline = time.monotonic() + 8
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if publisher.get_subscription_count() != 1:
            raise RuntimeError('expected exactly one initialpose subscriber')
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = node.get_clock().now().to_msg()
        message.pose.pose.position.x = seed['x']
        message.pose.pose.position.y = seed['y']
        message.pose.pose.orientation.z = math.sin(seed['yaw'] / 2)
        message.pose.pose.orientation.w = math.cos(seed['yaw'] / 2)
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = (math.pi / 12)**2
        publisher.publish(message)
        if not publisher.wait_for_all_acked(Duration(seconds=3)):
            raise RuntimeError('initial pose delivery not acknowledged')
        print(json.dumps({'published_once': True, 'source': str(Path(sys.argv[1]).resolve()), 'seed': seed}))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

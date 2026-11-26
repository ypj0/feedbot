import json
import time

import rclpy
from tf2_msgs.msg import TFMessage

from activate_mapping import change_state, require_state


def deactivate_mapping(node, report):
    require_state(node, 3, 'before deactivate')
    for identifier, name, expected in ((4, 'deactivate', 2), (2, 'cleanup', 1)):
        elapsed = change_state(node, identifier, name)
        state = require_state(node, expected, name)
        report['transitions'].append({'transition': name, 'elapsed_s': elapsed, 'state': state})
    report['passed'] = True


def main():
    rclpy.init()
    node = rclpy.create_node('feedbot_mapping_deactivation')
    report = {'passed': False, 'transitions': []}
    try:
        deactivate_mapping(node, report)
        report['passed'] = False
        cutoff = node.get_clock().now().nanoseconds
        report['map_odom_after_cleanup'] = []

        def observe(message):
            for transform in message.transforms:
                stamp = transform.header.stamp.sec * 1000000000 + transform.header.stamp.nanosec
                if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom' and stamp >= cutoff:
                    report['map_odom_after_cleanup'].append(stamp)

        subscription = node.create_subscription(TFMessage, '/tf', observe, 100)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        if report['map_odom_after_cleanup']:
            raise RuntimeError('map->odom still published after SLAM cleanup')
        report['passed'] = True
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        print(json.dumps(report))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

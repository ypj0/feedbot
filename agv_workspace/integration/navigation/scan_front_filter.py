#!/usr/bin/env python3
"""Front-only LaserScan filter for Nav2 costmaps.

Nav2 should not mark obstacles beside the robot as lethal during forward
planning; side returns should only be ignored by the costmap, not the planner.
This node publishes a valid front arc so the obstacle layer still receives
normal LaserScan geometry with only the forward beams present.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


class FrontScanFilter(Node):
    def __init__(self):
        super().__init__('scan_front_filter')
        self.declare_parameter('front_half_angle', 0.7)
        self.front_half_angle = float(self.get_parameter('front_half_angle').value)
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(LaserScan, '/scan_nav', 10)

    def scan_cb(self, msg):
        increment = msg.angle_increment
        if increment <= 0.0 or not msg.ranges:
            return
        start_index = max(0, int(math.floor((-self.front_half_angle - msg.angle_min) / increment)))
        end_index = min(
            len(msg.ranges) - 1,
            int(math.floor((self.front_half_angle - msg.angle_min) / increment)),
        )
        if end_index < start_index:
            return
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min + start_index * increment
        out.angle_max = msg.angle_min + end_index * increment
        out.angle_increment = increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = [
            float('inf') if value < 0.35 else value
            for value in msg.ranges[start_index:end_index + 1]
        ]
        if len(msg.intensities) == len(msg.ranges):
            out.intensities = msg.intensities[start_index:end_index + 1]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = FrontScanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

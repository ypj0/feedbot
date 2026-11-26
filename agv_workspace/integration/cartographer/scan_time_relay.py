"""Republish /scan with first-beam timestamps for Cartographer.

The ROS2 lidar driver stamps a completed frame with the time the last block was
received. Cartographer matches each scan to TF at that later time, which adds
motion-induced yaw/position drift while mapping. Subtract scan_time to recover
the first-beam estimate and publish it as /cartographer/scan.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanTimeRelay(Node):
    def __init__(self):
        super().__init__('cartographer_scan_time_relay')
        self.publisher = self.create_publisher(LaserScan, '/cartographer/scan', qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.callback, qos_profile_sensor_data)

    def callback(self, message):
        output = LaserScan()
        output.header.frame_id = message.header.frame_id
        shift_ns = int(message.scan_time * 1e9)
        stamp_ns = int(message.header.stamp.sec) * 1000000000 + int(message.header.stamp.nanosec) - shift_ns
        if stamp_ns < 0:
            stamp_ns = 0
        output.header.stamp.sec = stamp_ns // 1000000000
        output.header.stamp.nanosec = stamp_ns % 1000000000
        output.angle_min = message.angle_min
        output.angle_max = message.angle_max
        output.angle_increment = message.angle_increment
        output.time_increment = message.time_increment
        output.scan_time = message.scan_time
        output.range_min = message.range_min
        output.range_max = message.range_max
        output.ranges = message.ranges
        output.intensities = message.intensities
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = ScanTimeRelay()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

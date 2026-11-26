#!/usr/bin/env python3
"""Broadcast odom -> base_link for AMCL from the ROS1 wheel odometry relay.

The original relay forwards odometry as a topic but does not own a reliable
dynamic TF broadcaster.  AMCL requires odom -> base_link at laser timestamps,
so this node publishes that transform at a fixed rate with current time.
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.qos import qos_profile_sensor_data


class OdomTfBridge(Node):
    def __init__(self):
        super().__init__('odom_tf_bridge')
        self.broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_chain()
        self.latest = None
        self.sub = self.create_subscription(Odometry, '/odom', self.cb, 10)
        self.create_timer(0.02, self.tick)

    def publish_static_chain(self):
        now = self.get_clock().now().to_msg()
        base_footprint_to_base_link = TransformStamped()
        base_footprint_to_base_link.header.stamp = now
        base_footprint_to_base_link.header.frame_id = 'base_footprint'
        base_footprint_to_base_link.child_frame_id = 'base_link'
        base_footprint_to_base_link.transform.translation.z = 0.5
        base_footprint_to_base_link.transform.rotation.w = 1.0

        base_link_to_scan = TransformStamped()
        base_link_to_scan.header.stamp = now
        base_link_to_scan.header.frame_id = 'base_link'
        base_link_to_scan.child_frame_id = 'scan'
        base_link_to_scan.transform.translation.x = 0.27
        base_link_to_scan.transform.translation.z = -0.3
        base_link_to_scan.transform.rotation.w = 1.0

        self.static_broadcaster.sendTransform([
            base_footprint_to_base_link,
            base_link_to_scan,
        ])

    def cb(self, msg):
        self.latest = msg

    def tick(self):
        if self.latest is None:
            return
        msg = self.latest
        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = msg.child_frame_id or 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(t)

        scan_tf = TransformStamped()
        scan_tf.header.stamp = now
        scan_tf.header.frame_id = 'base_link'
        scan_tf.child_frame_id = 'scan'
        scan_tf.transform.translation.x = 0.27
        scan_tf.transform.translation.z = -0.3
        scan_tf.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(scan_tf)


def main():
    rclpy.init()
    node = OdomTfBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

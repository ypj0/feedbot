"""Convert Cartographer /tracked_pose to /amcl_pose for charge dock and web."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class TrackedPoseRelay(Node):
    def __init__(self):
        super().__init__("tracked_pose_relay")
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, "/amcl_pose", 10)
        self.subscription = self.create_subscription(
            PoseStamped, "/tracked_pose", self.callback, qos_profile_sensor_data)

    def callback(self, message):
        output = PoseWithCovarianceStamped()
        output.header = message.header
        output.pose.pose = message.pose
        output.pose.covariance = [0.01] * 36
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = TrackedPoseRelay()
    rclpy.spin(node)


if __name__ == "__main__":
    main()

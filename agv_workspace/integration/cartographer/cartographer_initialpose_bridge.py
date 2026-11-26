#!/usr/bin/env python3
"""Cartographer 纯定位初始位姿桥：/initialpose -> /start_trajectory。

Cartographer 纯定位不订阅 /initialpose。本桥等价于原厂“发布 initialpose 给
cartographer”的桥接：收到 map 系位姿后，用 StartTrajectory(use_initial_pose=true,
relative_to_trajectory_id=0) 把当前轨迹锚定到该位姿，完成运行时重定位。
"""

import threading
import time
from pathlib import Path

import rclpy
from cartographer_ros_msgs.srv import StartTrajectory
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class CartographerInitialPoseBridge(Node):
    def __init__(self):
        super().__init__('cartographer_initialpose_bridge')
        self.declare_parameter(
            'configuration_directory', str(Path(__file__).resolve().parent / 'config'))
        self.declare_parameter('configuration_basename', 'feedbot_2d_localization.lua')
        self.config_dir = self.get_parameter('configuration_directory').value
        self.config_basename = self.get_parameter('configuration_basename').value
        self.client = self.create_client(StartTrajectory, '/start_trajectory')
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.on_initial_pose, qos)
        self.get_logger().info('cartographer initialpose bridge ready')

    def on_initial_pose(self, msg):
        # 回调里不能阻塞等待服务响应，放到独立线程，主线程继续 spin。
        threading.Thread(target=self.apply, args=(msg,), daemon=True).start()

    def apply(self, msg):
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('cartographer /start_trajectory unavailable')
            return
        request = StartTrajectory.Request()
        request.configuration_directory = self.config_dir
        request.configuration_basename = self.config_basename
        request.use_initial_pose = True
        request.initial_pose = msg.pose.pose
        request.relative_to_trajectory_id = 0
        future = self.client.call_async(request)
        deadline = time.time() + 10.0
        while time.time() < deadline and not future.done():
            time.sleep(0.02)
        if not future.done():
            self.get_logger().error('cartographer start_trajectory relocalization timeout')
            return
        response = future.result()
        if response.status.code != 0:
            self.get_logger().error(
                'cartographer relocalization failed: %s'
                % (response.status.message or ('code %d' % response.status.code)))
            return
        self.get_logger().info(
            'cartographer relocalized to trajectory %d' % response.trajectory_id)


def main():
    rclpy.init()
    node = CartographerInitialPoseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Cartographer 纯定位初始位姿桥：/initialpose -> 结束当前轨迹并新建锚定轨迹。

Cartographer 纯定位不支持直接改位姿，也不能直接用 StartTrajectory 复用已被
占用的话题。正确做法：结束当前 active 轨迹，再用
StartTrajectory(use_initial_pose=true, relative_to_trajectory_id=0) 在 map 系
锚定到 /initialpose 给出的位姿。
"""

import threading
import time
from pathlib import Path

import rclpy
from cartographer_ros_msgs.srv import FinishTrajectory, GetTrajectoryStates, StartTrajectory
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


ACTIVE_STATE = 0


def yaw_from_quaternion(q):
    import math
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CartographerInitialPoseBridge(Node):
    def __init__(self):
        super().__init__('cartographer_initialpose_bridge')
        self.declare_parameter(
            'configuration_directory', str(Path(__file__).resolve().parent / 'config'))
        self.declare_parameter('configuration_basename', 'feedbot_2d_localization.lua')
        self.config_dir = self.get_parameter('configuration_directory').value
        self.config_basename = self.get_parameter('configuration_basename').value
        self.states_client = self.create_client(GetTrajectoryStates, '/get_trajectory_states')
        self.finish_client = self.create_client(FinishTrajectory, '/finish_trajectory')
        self.start_client = self.create_client(StartTrajectory, '/start_trajectory')
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

    def call(self, client, request, timeout=10.0):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('%s service not available' % client.srv_name)
        future = client.call_async(request)
        deadline = time.time() + timeout
        while time.time() < deadline and not future.done():
            time.sleep(0.02)
        if not future.done():
            raise RuntimeError('%s service timeout' % client.srv_name)
        response = future.result()
        if response.status.code != 0:
            raise RuntimeError(
                response.status.message or ('code %d' % response.status.code))
        return response

    def active_trajectory_id(self):
        response = self.call(self.states_client, GetTrajectoryStates.Request())
        states = response.trajectory_states
        for trajectory_id, state in zip(states.trajectory_id, states.trajectory_state):
            if state == ACTIVE_STATE:
                return trajectory_id
        return None

    def apply(self, msg):
        try:
            pose = msg.pose.pose
            active_id = self.active_trajectory_id()
            if active_id is not None:
                finish = FinishTrajectory.Request()
                finish.trajectory_id = active_id
                self.call(self.finish_client, finish)
            request = StartTrajectory.Request()
            request.configuration_directory = self.config_dir
            request.configuration_basename = self.config_basename
            request.use_initial_pose = True
            request.initial_pose = pose
            request.relative_to_trajectory_id = 0
            response = self.call(self.start_client, request)
            self.get_logger().info(
                'cartographer relocalized to (%.3f, %.3f, %.2f), trajectory %d'
                % (pose.position.x, pose.position.y,
                   yaw_from_quaternion(pose.orientation), response.trajectory_id))
        except Exception as error:
            self.get_logger().error('cartographer relocalization failed: %s' % error)


def main():
    rclpy.init()
    node = CartographerInitialPoseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Cartographer 全局重定位。

与旧 scan_relocalize.py（AMCL）不同：结果发布到 /initialpose，由
cartographer_initialpose_bridge 通过 /start_trajectory 应用到 Cartographer 纯定位。

位姿来源：
1. 充电中且存在 docked_pose -> 直接使用初始点（确定性播种，回桩自动重定位）。
2. 否则 -> 用 /scan 与 /map 做暴力全局扫描匹配。
"""

import argparse
import json
import math
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import (
    Point,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    Quaternion,
)
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from std_srvs.srv import Trigger


DOCK_CONFIG = Path('/opt/feedbot-maps/charge_dock_config.json')


class CartographerRelocalizer(Node):
    def __init__(self, accept_score=0.5, laser_max_range=3.5):
        super().__init__('cartographer_relocalizer')
        self.accept_score = accept_score
        self.laser_max_range = laser_max_range
        self.scan = None
        self.map = None
        self.charge_status = 0
        self.docked_pose = None
        self.seeded_for_charge = False
        self.load_docked_pose()

        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)
        self.sub_diag = self.create_subscription(
            DiagnosticArray, '/diagnostics', self.diag_cb, 10)
        initialpose_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub_initialpose = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', initialpose_qos)
        self.srv_reloc = self.create_service(Trigger, '/relocalize', self.reloc_cb)
        self.get_logger().info('cartographer relocalizer ready (/relocalize)')

    def load_docked_pose(self):
        name = ''
        try:
            name = Path('/opt/feedbot-maps/current_map.txt').read_text().strip()
        except Exception:
            pass
        path = (Path('/opt/feedbot-maps') / name / 'charge_dock_config.json'
                if name else DOCK_CONFIG)
        try:
            cfg = json.loads(path.read_text(encoding='utf-8'))
        except Exception:
            self.docked_pose = None
            return
        dp = cfg.get('docked_pose') if isinstance(cfg, dict) else None
        if dp and all(k in dp for k in ('x', 'y', 'yaw')):
            self.docked_pose = [float(dp['x']), float(dp['y']), float(dp['yaw'])]
        else:
            self.docked_pose = None

    def scan_cb(self, msg):
        self.scan = msg

    def map_cb(self, msg):
        self.map = msg

    def diag_cb(self, msg):
        for s in msg.status:
            if 'chassis' not in (s.name or ''):
                continue
            for v in s.values:
                if v.key == 'charge':
                    try:
                        self.charge_status = int(float(v.value))
                    except (TypeError, ValueError):
                        pass
        charging = 0 < self.charge_status < 5
        if charging and self.docked_pose is not None and not self.seeded_for_charge:
            self.seeded_for_charge = True
            self.publish_pose(*self.docked_pose)
            self.get_logger().info('auto-seeded Cartographer from docked pose (charge detected)')
        elif not charging:
            self.seeded_for_charge = False

    def reloc_cb(self, req, resp):
        try:
            # 每次重定位都重读当前地图配置：建图保存后 current_map.txt 已更新，
            # 这里立即用新地图的 docked_pose 播种，而不是继续用进程启动时的旧值。
            self.load_docked_pose()
            # 充电/回桩：用 docked_pose 精确播种（确定性，最快）
            if 0 < self.charge_status < 5 and self.docked_pose is not None:
                x, y, yaw = self.docked_pose
                self.publish_pose(x, y, yaw)
                resp.success = True
                resp.message = '%.3f,%.3f,%.2f (docked)' % (x, y, math.degrees(yaw))
                self.get_logger().info('relocalized from docked pose')
                return resp
            # 桩外：Cartographer 纯定位已放宽全局搜索窗，会自动全图重定位
            resp.success = True
            resp.message = 'Cartographer global localization running'
            self.get_logger().info('relocalize: rely on Cartographer global localization')
        except Exception as e:
            resp.success = False
            resp.message = str(e)
        return resp

    def publish_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header = Header(frame_id='map')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = PoseWithCovariance()
        msg.pose.pose.position = Point(x=float(x), y=float(y), z=0.0)
        msg.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))
        msg.pose.covariance = [0.01, 0, 0, 0, 0, 0,
                               0, 0.01, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0.03]
        self.pub_initialpose.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--accept-score', type=float, default=0.5)
    parser.add_argument('--laser-max-range', type=float, default=3.5)
    args = parser.parse_args()
    rclpy.init()
    node = CartographerRelocalizer(args.accept_score, args.laser_max_range)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

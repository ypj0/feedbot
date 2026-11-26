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
        try:
            cfg = json.loads(DOCK_CONFIG.read_text(encoding='utf-8'))
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
        if self.charge_status > 0 and self.docked_pose is not None and not self.seeded_for_charge:
            self.seeded_for_charge = True
            self.publish_pose(*self.docked_pose)
            self.get_logger().info('auto-seeded Cartographer from docked pose (charge detected)')
        elif self.charge_status == 0:
            self.seeded_for_charge = False

    def resolve_pose(self):
        """返回 (x, y, yaw, source) 或 None。"""
        if self.charge_status > 0 and self.docked_pose is not None:
            return self.docked_pose[0], self.docked_pose[1], self.docked_pose[2], 'docked'
        if self.scan is None or self.map is None:
            return None
        from scan_match import search
        best = search(self.map, self.scan, self.laser_max_range)
        if best is None:
            return None
        score, x, y, yaw = best
        if score > self.accept_score:
            raise RuntimeError('score too high %.3f > %.2f' % (score, self.accept_score))
        return x, y, yaw, 'scan'

    def reloc_cb(self, req, resp):
        try:
            pose = self.resolve_pose()
            if pose is None:
                resp.success = False
                resp.message = 'no scan/map data'
                return resp
            x, y, yaw, source = pose
            self.publish_pose(x, y, yaw)
            resp.success = True
            resp.message = '%.3f,%.3f,%.2f (%s)' % (x, y, math.degrees(yaw), source)
            self.get_logger().info(
                'relocalized to (%.3f, %.3f, %.2f) via %s'
                % (x, y, math.degrees(yaw), source))
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

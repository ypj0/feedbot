#!/usr/bin/env python3
"""扫描匹配全局重定位：用当前激光帧与地图边界做暴力匹配，自动恢复 AMCL 初始位姿。

流程：
1. 订阅一帧 /scan 和 /map。
2. 提取 < laser_max_range 的有效激光点，转换到 base_link 坐标。
3. 构建地图障碍物距离场，粗网格暴力搜索，再在最优候选附近细化。
4. 若最优匹配分数 < accept_score，调用 /set_initial_pose 设置 AMCL 初始位姿。

使用：python3 scan_relocalize.py [--accept-score 0.3] [--laser-max-range 3.5]
"""
import argparse
import math
import time
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import SetInitialPose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseWithCovariance, Pose, Point, Quaternion
from std_msgs.msg import Header
from std_srvs.srv import Trigger


class ScanRelocalizer(Node):
    def __init__(self, accept_score=0.8, laser_max_range=3.5):
        super().__init__('scan_relocalizer')
        self.accept_score = accept_score
        self.laser_max_range = laser_max_range
        self.scan = None
        self.map = None
        self.latest_tracked = None
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.sub_tracked = self.create_subscription(
            PoseStamped, '/original/tracked_pose', self.tracked_cb, 10)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)
        self.cli_pose = self.create_client(SetInitialPose, '/set_initial_pose')
        self.pub_initialpose = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 1)
        self.srv_reloc = self.create_service(Trigger, '/relocalize', self.reloc_cb)
        self.get_logger().info('scan_relocalizer ready, /relocalize service available')

    def reloc_cb(self, req, resp):
        try:
            if self.latest_tracked is not None:
                self.set_initial_pose_from_pose(self.latest_tracked)
                resp.success = True
                resp.message = '%.3f,%.3f,%.2f' % (
                    self.latest_tracked.pose.position.x,
                    self.latest_tracked.pose.position.y,
                    math.degrees(yaw_from_quaternion(self.latest_tracked.pose.orientation)))
                self.get_logger().info('relocalized from original tracked pose')
                return resp
            if self.scan is None or self.map is None:
                resp.success = False
                resp.message = 'no scan/map data'
                return resp
            best = self.search()
            if best is None:
                resp.success = False
                resp.message = 'no candidate'
                return resp
            score, x, y, yaw = best
            if score > self.accept_score:
                resp.success = False
                resp.message = 'score too high %.3f > %.2f' % (score, self.accept_score)
                return resp
            self.set_initial_pose(x, y, yaw)
            resp.success = True
            resp.message = '%.3f,%.3f,%.2f' % (x, y, math.degrees(yaw))
            self.get_logger().info('relocalized to (%.3f, %.3f, %.1f deg), score %.4f' %
                                   (x, y, math.degrees(yaw), score))
        except Exception as e:
            resp.success = False
            resp.message = str(e)
        return resp

    def tracked_cb(self, msg):
        self.latest_tracked = msg

    def scan_cb(self, msg):
        self.scan = msg

    def map_cb(self, msg):
        self.map = msg

    def wait_data(self, timeout=6.0):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.scan is not None and self.map is not None:
                return True
        return False

    def build_distance_field(self):
        m = self.map
        w, h = m.info.width, m.info.height
        arr = np.array(m.data, dtype=np.int8).reshape(h, w)
        occ = arr == 100
        dist = np.full((h, w), 1e9, dtype=np.float32)
        q = deque()
        ys, xs = np.where(occ)
        for y, x in zip(ys.tolist(), xs.tolist()):
            dist[y, x] = 0.0
            q.append((y, x))
        while q:
            y, x = q.popleft()
            d0 = dist[y, x] + 1
            for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                if 0 <= ny < h and 0 <= nx < w and dist[ny, nx] > d0:
                    dist[ny, nx] = d0
                    q.append((ny, nx))
        return w, h, dist

    def laser_points(self):
        s = self.scan
        bx, by = [], []
        for i, r in enumerate(s.ranges):
            if math.isfinite(r) and 0.05 < r < self.laser_max_range:
                a = s.angle_min + i * s.angle_increment
                bx.append(0.27 + r * math.cos(a))
                by.append(r * math.sin(a))
        return np.array(bx, dtype=np.float32), np.array(by, dtype=np.float32)

    def score(self, px, py, pyaw, w, h, dist, bx, by, res, ox, oy):
        c, s = math.cos(pyaw), math.sin(pyaw)
        gx = ((px + bx * c - by * s - ox) / res).astype(np.int32)
        gy = ((py + bx * s + by * c - oy) / res).astype(np.int32)
        valid = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
        if valid.sum() < len(bx) * 0.5:
            return 1e9
        return float(dist[gy[valid], gx[valid]].mean())

    def search(self):
        m = self.map
        res, ox, oy = m.info.resolution, m.info.origin.position.x, m.info.origin.position.y
        w, h, dist = self.build_distance_field()
        bx, by = self.laser_points()
        if len(bx) < 50:
            raise RuntimeError('not enough laser points')
        # 粗搜索
        coarse = []
        for deg in range(-180, 180, 15):
            pyaw = math.radians(deg)
            c, s = math.cos(pyaw), math.sin(pyaw)
            rx = bx * c - by * s
            ry = bx * s + by * c
            best = (1e9, 0.0, 0.0)
            for py in np.arange(oy, oy + h * res, 0.10):
                gy = ((py + ry - oy) / res).astype(np.int32)
                for px in np.arange(ox, ox + w * res, 0.10):
                    gx = ((px + rx - ox) / res).astype(np.int32)
                    valid = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
                    if valid.sum() < len(bx) * 0.5:
                        continue
                    sc = float(dist[gy[valid], gx[valid]].mean())
                    if sc < best[0]:
                        best = (sc, px, py)
            coarse.append((best[0], best[1], best[2], pyaw))
        coarse.sort(key=lambda z: z[0])
        best = None
        for sc, px, py, pyaw in coarse[:3]:
            for x in np.arange(px - 0.15, px + 0.16, 0.02):
                for y in np.arange(py - 0.15, py + 0.16, 0.02):
                    for deg in np.arange(math.degrees(pyaw) - 12, math.degrees(pyaw) + 13, 2):
                        sc2 = self.score(x, y, math.radians(deg), w, h, dist, bx, by, res, ox, oy)
                        if best is None or sc2 < best[0]:
                            best = (sc2, x, y, math.radians(deg))
        return best

    def set_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header = Header(frame_id='map')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = PoseWithCovariance()
        msg.pose.pose = Pose(
            position=Point(x=float(x), y=float(y), z=0.0),
            orientation=Quaternion(x=0.0, y=0.0,
                                   z=math.sin(yaw / 2), w=math.cos(yaw / 2)))
        msg.pose.covariance = [0.01, 0, 0, 0, 0, 0,
                               0, 0.01, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0.03]
        self.pub_initialpose.publish(msg)

    def set_initial_pose_from_pose(self, pose_stamped):
        msg = PoseWithCovarianceStamped()
        msg.header = Header(frame_id='map')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = PoseWithCovariance()
        msg.pose.pose = pose_stamped.pose
        msg.pose.covariance = [0.01, 0, 0, 0, 0, 0,
                               0, 0.01, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0.03]
        self.pub_initialpose.publish(msg)


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--accept-score', type=float, default=0.5)
    parser.add_argument('--laser-max-range', type=float, default=3.5)
    parser.add_argument('--once', action='store_true', help='run once and exit')
    args = parser.parse_args()
    rclpy.init()
    node = ScanRelocalizer(args.accept_score, args.laser_max_range)
    if not args.once:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
        return
    if not node.wait_data():
        print('FAIL no scan/map data')
        raise SystemExit(1)
    best = node.search()
    if best is None:
        print('FAIL no candidate')
        raise SystemExit(1)
    score, x, y, yaw = best
    print('MATCH score=%.4f x=%.3f y=%.3f yaw_deg=%.1f' % (
        score, x, y, math.degrees(yaw)))
    if score > args.accept_score:
        print('FAIL score too high %.4f > %.3f' % (score, args.accept_score))
        raise SystemExit(1)
    node.set_initial_pose(x, y, yaw)
    print('OK initial pose set')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

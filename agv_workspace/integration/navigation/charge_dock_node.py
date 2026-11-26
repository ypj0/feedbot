#!/usr/bin/env python3
"""Feedbot 回充对接节点：复用原厂 ChargeController 控制律（离线逆向模型）。

输入：
- /amcl_pose：机器人当前位姿（Cartographer /tracked_pose 的兼容别名）
- /scan：前向激光，用于桩模板 ICP 搜索
- /diagnostics：底盘反馈，提取 ChargeStatus（charge 字段）
输出：
- /wheel/cmd_vel：TwistStamped 速度命令（回充期间独占）

原厂逻辑来源：navigation_runtime/reverse/recovered/charge_core.c
控制参数：lidarcharge.launch（abort_angle 2deg, max_rotate 0.3,
forward 0.08, backward -0.07, chargedis_threshold 0.4, abort_threshold 0.07）
"""
import json
import math
import os
import threading

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

from dock_icp import (
    compose_pose, icp_align, measure_plate_distance, refine_pose,
    relative_pose, scan_front_points,
)

TAU = 6.283185307179586


def norm_angle(a):
    a = math.fmod(math.fmod(a, TAU) + TAU, TAU)
    return a - TAU if a > math.pi else a


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ChargeDockNode(Node):
    def __init__(self):
        super().__init__('feedbot_charge_dock')
        self.declare_parameter('config', '/opt/feedbot-maps/charge_dock_config.json')
        self.declare_parameter('dock_x', 0.0)
        self.declare_parameter('dock_y', 0.0)
        self.declare_parameter('dock_yaw', 0.0)
        self.declare_parameter('abort_angle', 0.0349)
        self.declare_parameter('max_rotate_speed', 0.3)
        self.declare_parameter('forward_speed', 0.08)
        self.declare_parameter('backward_speed', -0.07)
        self.declare_parameter('chargedis_threshold', 0.4)
        self.declare_parameter('abort_threshold', 0.07)
        self.declare_parameter('abort_distance', 0.5)
        self.declare_parameter('final_stall_ticks', 120)
        self.declare_parameter('charge_timeout', 180.0)
        self.declare_parameter('controller_frequency', 20.0)
        self.declare_parameter('undock_distance', 0.2)
        self.declare_parameter('undock_speed', 0.15)
        self.declare_parameter('undock_max_distance', 0.6)
        self.declare_parameter('search_timeout', 30.0)
        self.declare_parameter('search_rotate_speed', 0.3)
        self.declare_parameter('icp_rms_threshold', 0.05)
        self.declare_parameter('icp_inlier_ratio', 0.5)

        self.cfg_path = self.get_parameter('config').value
        self.abort_angle = self.get_parameter('abort_angle').value
        self.max_rotate_speed = self.get_parameter('max_rotate_speed').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.backward_speed = self.get_parameter('backward_speed').value
        self.chargedis_threshold = self.get_parameter('chargedis_threshold').value
        self.abort_threshold = self.get_parameter('abort_threshold').value
        self.abort_distance = self.get_parameter('abort_distance').value
        self.final_stall_ticks = self.get_parameter('final_stall_ticks').value
        self.charge_timeout = self.get_parameter('charge_timeout').value
        self.freq = self.get_parameter('controller_frequency').value
        self.undock_distance = self.get_parameter('undock_distance').value
        self.undock_speed = self.get_parameter('undock_speed').value
        self.undock_max_distance = self.get_parameter('undock_max_distance').value
        self.search_timeout = self.get_parameter('search_timeout').value
        self.search_rotate_speed = self.get_parameter('search_rotate_speed').value
        self.icp_rms_threshold = self.get_parameter('icp_rms_threshold').value
        self.icp_inlier_ratio = self.get_parameter('icp_inlier_ratio').value

        self.dock = None          # [x, y, yaw]
        self.pre_dock = None      # [x, y, yaw]
        self.docked_pose = None   # 初始点：车实际停靠桩上的地图位姿 [x, y, yaw]
        self.robot = None         # [x, y, yaw]
        self.charge_status = 0
        self.scan = None          # 最新 LaserScan
        self.odom = None          # 最新里程计位姿 [x, y, yaw]（odom 帧）
        self.template = None      # Nx2 桩模板点（scan 帧）
        self.template_pose = None # 标定时机器人地图位姿 [x, y, yaw]

        # 原厂跨周期命令状态
        self.twist_lin = 0.0
        self.twist_ang = 0.0

        # 原厂状态标志
        self.xy_reached = False
        self.yaw_reached = False
        self.notpermit_back = False
        self.approach_done = False
        self.progress_count = 0
        self.distance_baseline = 0.0
        self.distance_increase_count = 0
        self.last_distance = 0.0

        self.active = False
        self.undocking = False
        self.charging = False
        self.finished = False
        self.failed = False
        self.failure_reason = ''
        self.searching = False
        self.search_count = 0
        self.search_started_at = None
        self.icp_attempted = False
        self.pose_correction = None
        self.final_start_pose = None
        self.final_start_odom = None
        self.started_at = None
        self.searching = False
        self.search_count = 0
        self.search_started_at = None
        self.icp_attempted = False
        self.pose_correction = None  # [dx, dy, dyaw] ICP 锁定后相对 amcl 的修正
        self.final_start_pose = None  # 进入最终入桩时的地图位姿
        self.final_start_odom = None  # 进入最终入桩时的里程计位姿
        self.lock = threading.Lock()

        self.pub_cmd = self.create_publisher(TwistStamped, '/wheel/cmd_vel', qos_profile_sensor_data)
        self.sub_pose = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_cb, 10)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.sub_odom = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_cb, qos_profile_sensor_data)
        self.sub_diag = self.create_subscription(
            DiagnosticArray, '/diagnostics', self.diag_cb, 10)
        self.srv_start = self.create_service(Trigger, '/charge_dock/start', self.start_cb)
        self.srv_cancel = self.create_service(Trigger, '/charge_dock/cancel', self.cancel_cb)
        self.srv_undock = self.create_service(Trigger, '/charge_dock/undock', self.undock_cb)
        self.srv_calibrate = self.create_service(Trigger, '/charge_dock/calibrate', self.calibrate_cb)
        self.srv_calibrate_docked = self.create_service(
            Trigger, '/charge_dock/calibrate_docked', self.calibrate_docked_cb)
        self.srv_reload = self.create_service(Trigger, '/charge_dock/reload', self.reload_cb)
        self.srv_get_dock = self.create_service(Trigger, '/charge_dock/get_dock', self.get_dock_cb)
        self.create_timer(1.0 / self.freq, self.control_tick)

        self.load_dock()

    def load_dock(self):
        try:
            with open(self.cfg_path, 'r', encoding='utf-8') as f:
                cfg = json.load(f)
            dock = cfg.get('dock')
            if dock and all(k in dock for k in ('x', 'y', 'yaw')):
                self.dock = [float(dock['x']), float(dock['y']), float(dock['yaw'])]
                # 预对接点 = 桩沿桩背对方向（远离桩）0.5m，车头朝桩
                self.pre_dock = [
                    self.dock[0] - 0.5 * math.cos(self.dock[2]),
                    self.dock[1] - 0.5 * math.sin(self.dock[2]),
                    self.dock[2],
                ]
                self.get_logger().info(
                    'dock pose loaded: %.3f %.3f %.3f' % tuple(self.dock))
            else:
                self.get_logger().warn('charge dock config has no dock pose')
            tmpl = cfg.get('template')
            if tmpl and tmpl.get('points') and tmpl.get('pose'):
                self.template = np.asarray(tmpl['points'], dtype=float).reshape(-1, 2)
                self.template_pose = [float(v) for v in tmpl['pose']]
                self.get_logger().info(
                    'dock template loaded: %d points' % self.template.shape[0])
            else:
                self.template = None
                self.template_pose = None
                self.get_logger().warn('charge dock config has no laser template')
            dp = cfg.get('docked_pose')
            if dp and all(k in dp for k in ('x', 'y', 'yaw')):
                self.docked_pose = [float(dp['x']), float(dp['y']), float(dp['yaw'])]
                self.get_logger().info(
                    'docked pose loaded: %.3f %.3f %.3f' % tuple(self.docked_pose))
            else:
                self.docked_pose = None
                self.get_logger().warn('charge dock config has no docked pose')
        except FileNotFoundError:
            self.get_logger().warn('charge dock config not found: %s' % self.cfg_path)
        except Exception as e:
            self.get_logger().error('load dock config failed: %s' % e)

    def pose_cb(self, msg):
        p = msg.pose.pose
        self.robot = [p.position.x, p.position.y, quat_to_yaw(p.orientation)]

    def scan_cb(self, msg):
        self.scan = msg

    def odom_cb(self, msg):
        p = msg.pose.pose
        self.odom = [p.position.x, p.position.y, quat_to_yaw(p.orientation)]

    def diag_cb(self, msg):
        for s in msg.status:
            if 'chassis' in s.name.lower():
                for kv in s.values:
                    if kv.key == 'charge':
                        try:
                            self.charge_status = int(float(kv.value))
                        except Exception:
                            pass
                break

    def start_cb(self, req, resp):
        with self.lock:
            if self.dock is None or self.robot is None:
                resp.success = False
                resp.message = 'dock pose or robot pose not ready'
                return resp
            if self.active:
                resp.success = False
                resp.message = 'already docking'
                return resp
            self.reset_state()
            self.active = True
            self.started_at = self.get_clock().now()
            resp.success = True
            resp.message = 'started'
            self.get_logger().info('docking started')
            return resp

    def cancel_cb(self, req, resp):
        with self.lock:
            self.stop_motion()
            self.active = False
            self.undocking = False
            self.failed = True
            self.failure_reason = 'cancelled'
            resp.success = True
            resp.message = 'cancelled'
            self.get_logger().info('docking cancelled')
            return resp

    def undock_cb(self, req, resp):
        with self.lock:
            if self.active or self.undocking:
                resp.success = False
                resp.message = 'already moving'
                return resp
            self.undocking = True
            self.undock_start = list(self.robot) if self.robot is not None else None
            self.undock_started_at = self.get_clock().now()
            self.undock_lin = 0.0
            resp.success = True
            resp.message = 'undocking started'
            self.get_logger().info('undocking started')
            return resp

    def calibrate_cb(self, req, resp):
        """标定：约定车辆停在桩前 0.5m、车头朝桩，记录桩位姿和激光模板。"""
        with self.lock:
            if self.robot is None:
                resp.success = False
                resp.message = 'robot pose not ready'
                return resp
            if self.scan is None:
                resp.success = False
                resp.message = 'scan not ready'
                return resp
            pts = scan_front_points(self.scan)
            if pts.shape[0] < 20:
                resp.success = False
                resp.message = 'too few front scan points: %d' % pts.shape[0]
                return resp
            th = self.robot[2]
            plate = measure_plate_distance(pts)
            # 桩位姿 = 标定位姿 + (激光到桩面距离 - 0.055m) * 朝向
            # 0.055 = 激光在中心前 0.27m - 车尾距中心 0.325m
            offset = (plate - 0.055) if plate is not None else 0.5
            dock = [self.robot[0] + offset * math.cos(th),
                    self.robot[1] + offset * math.sin(th), th]
            pre_dock = [self.robot[0], self.robot[1], th]
            cfg = {
                'dock': {'x': dock[0], 'y': dock[1], 'yaw': dock[2]},
                'pre_dock': {'x': pre_dock[0], 'y': pre_dock[1], 'yaw': pre_dock[2]},
                'template': {'points': pts.tolist(), 'pose': pre_dock},
            }
            self.dock = dock
            self.pre_dock = pre_dock
            self.template = pts
            self.template_pose = pre_dock
            self.stop_motion()
            self.active = False
            self.undocking = False
            resp.success = True
            resp.message = json.dumps(cfg, ensure_ascii=False)
            self.get_logger().info(
                'dock calibrated: plate=%.3f offset=%.3f' % (
                    plate if plate is not None else float('nan'), offset))
            return resp

    def calibrate_docked_cb(self, req, resp):
        """初始点：车辆已上桩并充电，记录当前实际停靠位姿。"""
        with self.lock:
            if self.robot is None:
                resp.success = False
                resp.message = 'robot pose not ready'
                return resp
            if not (0 < self.charge_status < 5):
                resp.success = False
                resp.message = '未检测到充电状态，请确认已上桩并充电'
                return resp
            self.docked_pose = [self.robot[0], self.robot[1], self.robot[2]]
            resp.success = True
            resp.message = json.dumps(
                {'docked_pose': {'x': self.docked_pose[0], 'y': self.docked_pose[1],
                                 'yaw': self.docked_pose[2]}}, ensure_ascii=False)
            self.get_logger().info('docked pose calibrated: %s' % resp.message)
            return resp

    def reload_cb(self, req, resp):
        # 标定通过 set_parameters 设置 dock_x/y/yaw；优先参数，其次配置文件
        dx = self.get_parameter('dock_x').value
        dy = self.get_parameter('dock_y').value
        dyaw = self.get_parameter('dock_yaw').value
        if dx != 0.0 or dy != 0.0:
            self.dock = [dx, dy, dyaw]
            self.pre_dock = [
                self.dock[0] - 0.5 * math.cos(self.dock[2]),
                self.dock[1] - 0.5 * math.sin(self.dock[2]),
                self.dock[2],
            ]
            self.get_logger().info(
                'dock pose from parameters: %.3f %.3f %.3f' % tuple(self.dock))
            # 持久化到配置文件（/navcfg 需可写）
            try:
                cfg = {
                    'dock': {'x': self.dock[0], 'y': self.dock[1], 'yaw': self.dock[2]},
                    'pre_dock': {'x': self.pre_dock[0], 'y': self.pre_dock[1],
                                 'yaw': self.pre_dock[2]},
                }
                with open(self.cfg_path, 'w', encoding='utf-8') as f:
                    json.dump(cfg, f, ensure_ascii=False)
                self.get_logger().info('dock config persisted to %s' % self.cfg_path)
            except Exception as e:
                self.get_logger().error('persist dock config failed: %s' % e)
        else:
            self.load_dock()
        with self.lock:
            self.stop_motion()
            self.active = False
        resp.success = self.dock is not None
        resp.message = 'dock reloaded' if self.dock is not None else 'dock pose still missing'
        return resp

    def get_dock_cb(self, req, resp):
        resp.success = self.dock is not None
        resp.message = json.dumps({'dock': self.dock, 'pre_dock': self.pre_dock,
                                   'active': self.active,
                                   'charging': self.charging, 'failed': self.failed,
                                   'undocking': self.undocking,
                                   'failure_reason': self.failure_reason},
                                  ensure_ascii=False)
        return resp

    def reset_state(self):
        self.twist_lin = 0.0
        self.twist_ang = 0.0
        self.xy_reached = False
        self.yaw_reached = False
        self.notpermit_back = False
        self.approach_done = False
        self.progress_count = 0
        self.distance_baseline = 0.0
        self.distance_increase_count = 0
        self.last_distance = 0.0
        self.charging = False
        self.finished = False
        self.failed = False
        self.failure_reason = ''

    def stop_motion(self):
        self.publish_cmd(0.0, 0.0)

    def publish_cmd(self, lin, ang):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist.linear.x = float(lin)
        out.twist.angular.z = float(ang)
        self.pub_cmd.publish(out)

    def control_tick(self):
        with self.lock:
            if self.undocking:
                self.undock_tick()
                return
            if not self.active or self.robot is None or self.dock is None:
                return
            # 充电成功检测：0 < charge < 5（原厂语义）
            if 0 < self.charge_status < 5:
                self.charging = True
                self.finished = True
                self.active = False
                self.stop_motion()
                self.get_logger().info('charging detected, docking finished')
                return
            # 超时
            if self.started_at is not None:
                dt = (self.get_clock().now() - self.started_at).nanoseconds / 1e9
                if dt > self.charge_timeout:
                    self.failed = True
                    self.failure_reason = 'timeout'
                    self.active = False
                    self.stop_motion()
                    self.get_logger().info('docking timeout')
                    return
            # 有激光模板：尝试一次 ICP 精定位；失败则退回 Cartographer
            if self.template is not None and self.pose_correction is None \
                    and not self.icp_attempted:
                self.try_icp_once()
            robot = self.current_robot()
            if robot is None:
                return
            if not self.approach_done:
                r = self.approach(robot, self.dock)
                if r == 0:
                    self.approach_done = True
                    self.final_start_pose = robot
                    self.final_start_odom = list(self.odom) if self.odom is not None else None
                    self.stop_motion()
                    self.get_logger().info('approach done, entering final')
            else:
                if not self.final_step(robot, self.docked_target()):
                    self.failed = True
                    self.failure_reason = 'final progress stopped'
                    self.active = False
                    self.stop_motion()
                    self.get_logger().info('docking final failed')

    def corrected_robot(self):
        if self.robot is None:
            return None
        if self.pose_correction is None:
            return list(self.robot)
        dx, dy, dyaw = self.pose_correction
        return [self.robot[0] + dx, self.robot[1] + dy,
                norm_angle(self.robot[2] + dyaw)]

    def current_robot(self):
        """接近段用 Cartographer+ICP 修正；最终入桩段改用里程计死推。"""
        if self.approach_done and self.final_start_pose is not None \
                and self.final_start_odom is not None and self.odom is not None:
            rel = relative_pose(self.final_start_odom, self.odom)
            return compose_pose(self.final_start_pose, rel)
        return self.corrected_robot()

    def docked_target(self):
        """最终倒车目标：初始点（上桩实测位姿）；无初始点时退回桩位姿。"""
        if self.docked_pose is None:
            return self.dock
        return [self.docked_pose[0], self.docked_pose[1],
                norm_angle(self.docked_pose[2] - math.pi)]

    def try_icp_once(self):
        """导航到 pre_dock 后只做一次模板 ICP 精定位，不旋转找桩。"""
        self.icp_attempted = True
        if self.scan is None or self.robot is None:
            return
        current = scan_front_points(self.scan)
        if current.shape[0] < 20:
            return
        theta, tx, ty, rms, inliers = icp_align(self.template, current)
        if rms < self.icp_rms_threshold and inliers > self.icp_inlier_ratio:
            refined = refine_pose(self.template_pose, theta, tx, ty)
            self.pose_correction = [
                refined[0] - self.robot[0],
                refined[1] - self.robot[1],
                norm_angle(refined[2] - self.robot[2]),
            ]
            self.get_logger().info(
                'dock template locked: rms=%.4f inliers=%.2f correction=%s'
                % (rms, inliers, self.pose_correction))
        else:
            self.get_logger().info(
                'dock template ICP not locked (rms=%.4f inliers=%.2f), '
                'fall back to cartographer pose' % (rms, inliers))

    def undock_tick(self):
        """脱桩：前进离开桩，直到充电解除且离开足够距离，或达到最大距离。"""
        if self.robot is None:
            self.stop_motion()
            return
        if self.undock_started_at is not None:
            dt = (self.get_clock().now() - self.undock_started_at).nanoseconds / 1e9
            if dt > 15.0:
                self.undocking = False
                self.stop_motion()
                self.get_logger().warn('undocking timeout, charge still locked')
                return
        if self.undock_start is None:
            self.undock_start = list(self.robot)
        moved = math.hypot(self.robot[0] - self.undock_start[0],
                           self.robot[1] - self.undock_start[1])
        if self.charge_status == 0 and moved >= self.undock_distance:
            self.undocking = False
            self.stop_motion()
            self.get_logger().info(
                'undocking done, charge cleared, moved %.3f m' % moved)
            return
        if moved >= self.undock_max_distance:
            self.undocking = False
            self.stop_motion()
            self.get_logger().warn(
                'undocking stopped at max distance %.3f m, charge=%d'
                % (moved, self.charge_status))
            return
        # 低速前进（正线速度）
        self.undock_lin = min(self.undock_lin + 0.02, self.undock_speed)
        self.publish_cmd(self.undock_lin, 0.0)

    def approach(self, r, d):
        """原厂 recovered_approach_planar。返回 0=完成, 1=进行中。"""
        heading = d[2] + 3.14
        diff = norm_angle(heading - math.atan2(r[1] - d[1], r[0] - d[0]))
        distance = math.hypot(r[0] - d[0], r[1] - d[1])
        lateral = math.sin(diff) * distance
        along = abs(distance * math.cos(diff))
        if heading < 0:
            heading += 6.28
        cos_h, sin_h = math.cos(heading), math.sin(heading)
        ax = d[0] - along * cos_h
        ay = d[1] - along * sin_h
        bx = d[0] + along * cos_h
        by = d[1] + along * sin_h
        da = math.hypot(ax - r[0], ay - r[1])
        db = math.hypot(bx - r[0], by - r[1])
        if db <= da:
            ax, ay = bx, by
        error = norm_angle(math.atan2(ay - r[1], ax - r[0]) - r[2])

        if abs(lateral) < self.abort_threshold or self.xy_reached:
            self.xy_reached = True
            if self.abort_threshold < along - self.abort_distance:
                error = norm_angle(math.atan2(d[1] - r[1], d[0] - r[0]) - r[2])
                self.approach_slice(error, second=False)
                return 1
            self.publish_cmd(0.0, 0.0)
            return 0
        if self.abort_distance <= along or self.notpermit_back:
            self.approach_slice(error, second=True)
            return 1
        # 后退拉开距离
        self.twist_lin = max(self.twist_lin - 0.03, self.backward_speed)
        self.twist_ang = 0.0
        self.publish_cmd(self.twist_lin, self.twist_ang)
        return 1

    def approach_slice(self, error, second):
        if second:
            self.notpermit_back = True
        a = abs(error)
        if a <= self.abort_angle:
            self.twist_lin = min(self.twist_lin + 0.03, self.forward_speed)
            self.twist_ang = 0.0
        else:
            self.twist_lin = 0.0
            w = abs(self.twist_ang)
            if a > self.max_rotate_speed:
                w = min(w + 0.06, self.max_rotate_speed)
            else:
                target = 0.6 if a > 0.5 else 0.15
                step = 0.06 if a > 0.5 else 0.09
                w = max(w - step, target) if w > target else min(w + 0.06, target)
            self.twist_ang = (-1.0 if error < 0 else 1.0) * w
        self.publish_cmd(self.twist_lin, self.twist_ang)

    def final_step(self, r, d):
        """原厂 recovered_final_planar。返回 True=进行中, False=完成(进度停止)。"""
        error = norm_angle(d[2] + 3.1415926 - r[2])
        dx = d[0] - r[0]
        dy = d[1] - r[1]
        distance = math.hypot(dx, dy)
        rotate = False
        if not self.yaw_reached:
            self.distance_increase_count = 0
            self.distance_baseline = distance
            rotate = abs(error) > self.abort_angle
        self.final_slice(error, distance, reverse=not rotate)
        if self.chargedis_threshold <= distance:
            return True
        if not self.yaw_reached:
            # 掉头对准阶段不检测入桩进度（原地旋转距离本就不变）
            return True
        if abs(self.distance_baseline - distance) >= 0.025:
            self.progress_count = 0
            self.distance_baseline = distance
            return True
        self.progress_count += 1
        return self.progress_count <= self.final_stall_ticks

    def final_slice(self, error, distance, reverse):
        if reverse:
            self.yaw_reached = True
            self.twist_lin = max(self.twist_lin - 0.01, self.backward_speed)
            self.twist_ang = 0.0
            if distance > self.last_distance:
                self.distance_increase_count += 1
            self.last_distance = distance
        else:
            self.twist_lin = 0.0
            a = abs(error)
            w = abs(self.twist_ang)
            if a > self.max_rotate_speed:
                w = min(w + 0.06, self.max_rotate_speed)
            else:
                target = 0.6 if a > 0.5 else 0.05
                step = 0.06 if a > 0.5 else 0.04
                w = max(w - step, target) if w > target else min(w + step, target)
            self.twist_ang = (-1.0 if error < 0 else 1.0) * w
        self.publish_cmd(self.twist_lin, self.twist_ang)


def main():
    rclpy.init()
    node = ChargeDockNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

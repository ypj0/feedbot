"""导航测试网页后端：发布地图/位姿/规划路径/footprint，接收目标点。
- GET /api/state: 地图、当前位姿、规划路径、碰撞体积(footprint)、AMCL协方差
- POST /api/preview: 只规划不运动(compute_path_to_pose)，返回路径供确认
- POST /api/goal: 发送 navigate_to_pose，真正执行
- POST /api/cancel: 取消导航
"""
import json
import math
import os
import subprocess
import os
import threading
import time
import re
import shutil
import datetime
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from std_srvs.srv import Trigger
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data


FOOTPRINT = [[-0.325, -0.275], [0.325, -0.275], [0.325, 0.275], [-0.325, 0.275]]
MAPS_ROOT = '/opt/feedbot-maps'
CART_API = 'http://127.0.0.1:8099'
MGR_API = 'http://127.0.0.1:8097'
NAV_POINTS_FILE = os.path.join(MAPS_ROOT, 'nav_points.json')

ERROR_TEXT = {
    200: '未知错误',
    201: '规划器无效',
    202: 'TF 错误',
    203: '起点超出地图',
    204: '目标点超出地图',
    205: '起点被占用',
    206: '目标点被占用',
    207: '规划超时',
    208: '无有效路径（目标点不可达或太靠近障碍物）',
}


def err_text(result):
    msg = getattr(result, 'error_msg', '') or ''
    text = ERROR_TEXT.get(getattr(result, 'error_code', 0), '')
    if msg:
        return '%s（%s）' % (text or '错误', msg)
    return text or '错误码 %s' % getattr(result, 'error_code', 0)


def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


class State:
    def __init__(self):
        self.lock = threading.RLock()
        self.listeners = []
        self.map = None
        self.raw_map = None
        self.airwall = None
        self.pose = None
        self.plan = None
        self.local_plan = None
        self.goal = None
        self.nav_error = None
        self.diag = None
        self.scan = None
        self.status = 'idle'  # idle / previewing / navigating / succeeded / aborted / cancelled / docking_nav / docking
        self.dock = None
        self.pre_dock = None
        self.docked_pose = None
        self.mapping = False
        self.mapping_saved = None
        self.current_map = None

    def add_listener(self, fn):
        with self.lock:
            self.listeners.append(fn)

    def notify(self, key, value):
        with self.lock:
            listeners = list(self.listeners)
        for fn in listeners:
            try:
                fn(key, value)
            except Exception:
                pass

    def set(self, key, value):
        with self.lock:
            setattr(self, key, value)
            self.notify(key, value)

    def snapshot(self):
        with self.lock:
            return dict(map=self.map, raw_map=self.raw_map, airwall=self.airwall,
                        pose=self.pose, plan=self.plan, local_plan=self.local_plan,
                        goal=self.goal, status=self.status, nav_error=self.nav_error,
                        dock=self.dock, pre_dock=self.pre_dock,
                        docked_pose=self.docked_pose,
                        diag=self.diag, scan=self.scan, footprint=FOOTPRINT,
                        mapping=self.mapping, mapping_saved=self.mapping_saved,
                        current_map=self.current_map)


class NavNode(Node):
    def __init__(self, state):
        super().__init__('nav_web_bridge')
        self.state = state
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, retained)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_cb, retained)
        self.create_subscription(PoseStamped, '/tracked_pose', self.tracked_pose_cb, qos_profile_sensor_data)
        self.create_subscription(NavPath, '/plan', self.plan_cb, qos_profile_sensor_data)
        self.create_subscription(NavPath, '/local_plan', self.local_plan_cb, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.diag_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.manual_vel_pub = self.create_publisher(TwistStamped, '/wheel/cmd_vel', qos_profile_sensor_data)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.path_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.dock_start_client = self.create_client(Trigger, '/charge_dock/start')
        self.dock_cancel_client = self.create_client(Trigger, '/charge_dock/cancel')
        self.dock_undock_client = self.create_client(Trigger, '/charge_dock/undock')
        self.dock_calibrate_client = self.create_client(Trigger, '/charge_dock/calibrate')
        self.dock_calibrate_docked_client = self.create_client(
            Trigger, '/charge_dock/calibrate_docked')
        self.dock_get_client = self.create_client(Trigger, '/charge_dock/get_dock')
        self.dock_reload_client = self.create_client(Trigger, '/charge_dock/reload')
        self.load_dock_state()
        self.relocalize_client = self.create_client(Trigger, '/relocalize')
        self.load_map_client = self.create_client(LoadMap, '/map_server/load_map')
        self.dock_nav_active = False
        self.dock_start_future = None
        self.charge_status = 0
        self.power = None
        self.auto_charge = True
        self.auto_charge_triggered = False
        self.auto_charge_retry_at = 0.0
        self.dock_status_at = 0
        self.route_active = False
        self.route_points = []
        self.route_index = 0
        self.create_timer(0.05, self.update)


    def scan_cb(self, msg):
        try:
            t = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
            tx, ty = t.transform.translation.x, t.transform.translation.y
            q = t.transform.rotation
            theta = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        except Exception:
            return
        pts = []
        step = 4
        a = msg.angle_min
        ranges = msg.ranges
        for i in range(0, len(ranges), step):
            r = ranges[i]
            if math.isfinite(r) and 0.0 < r < msg.range_max:
                px = r * math.cos(a)
                py = r * math.sin(a)
                mx = tx + px*math.cos(theta) - py*math.sin(theta)
                my = ty + px*math.sin(theta) + py*math.cos(theta)
                pts.append([round(mx, 3), round(my, 3)])
            a += msg.angle_increment * step
        self.state.set('scan', pts)

    def diag_cb(self, msg):
        diag = dict(getattr(self.state, 'diag', None) or {})
        for s in msg.status:
            name = s.name
            if 'chassis' in name or 'ltr2' in name or 'imu' in name.lower():
                level = s.level[0] if isinstance(s.level, (bytes, bytearray)) else int(s.level)
                values = {v.key: v.value for v in s.values}
                diag[name] = dict(level=level, message=s.message,
                                  values=values)
                if 'chassis' in name:
                    if 'charge' in values:
                        try:
                            self.charge_status = int(float(values['charge']))
                        except Exception:
                            pass
                    if 'power' in values:
                        try:
                            self.power = int(float(values['power']))
                        except Exception:
                            pass
        self.state.set('diag', diag)

    def map_cb(self, msg):
        m = msg
        self.state.set('map', dict(
            width=m.info.width, height=m.info.height, resolution=m.info.resolution,
            origin=[m.info.origin.position.x, m.info.origin.position.y],
            data=[int(v) for v in m.data]))

    def pose_cb(self, msg):
        p = msg.pose.pose
        cov = msg.pose.covariance
        import math as _m
        self.pose_xy_std = _m.sqrt(max(cov[0], cov[7], 0.0))
        self.state.set('pose', dict(
            x=p.position.x, y=p.position.y, yaw=yaw(p.orientation),
            yaw_std=math.degrees(math.sqrt(max(cov[35], 0)))))

    def tracked_pose_cb(self, msg):
        p = msg.pose
        self.pose_xy_std = 0.0
        self.state.set('pose', dict(
            x=p.position.x, y=p.position.y, yaw=yaw(p.orientation),
            yaw_std=0.0))

    def local_plan_cb(self, msg):
        self.state.set('local_plan', self._path_to_map(msg))

    def plan_cb(self, msg):
        self.state.set('plan', self._path_to_map(msg))

    def _path_to_map(self, msg):
        frame = msg.header.frame_id or 'map'
        tx, ty, theta = 0.0, 0.0, 0.0
        if frame != 'map':
            try:
                t = self.tf_buffer.lookup_transform('map', frame, rclpy.time.Time())
                tx, ty = t.transform.translation.x, t.transform.translation.y
                q = t.transform.rotation
                theta = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            except Exception:
                pass
        out = []
        for pp in msg.poses:
            x, y, a = pp.pose.position.x, pp.pose.position.y, yaw(pp.pose.orientation)
            if frame != 'map':
                nx = tx + x*math.cos(theta) - y*math.sin(theta)
                ny = ty + x*math.sin(theta) + y*math.cos(theta)
                a = a + theta
            else:
                nx, ny = x, y
            out.append(dict(x=nx, y=ny, yaw=a))
        return out

    def call_trigger(self, client, timeout=5.0, service_name='service'):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(service_name + ' service not available')
        fut = client.call_async(Trigger.Request())
        deadline = time.time() + timeout
        while time.time() < deadline:
            if fut.done():
                r = fut.result()
                return r.success, r.message
            time.sleep(0.02)
        raise RuntimeError(service_name + ' service timeout')

    def relocalize(self):
        # 全局扫描匹配可能需要数秒；服务不存在仍要快速失败。
        if not self.relocalize_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('relocalize service not available')
        fut = self.relocalize_client.call_async(Trigger.Request())
        deadline = time.time() + 30.0
        while time.time() < deadline:
            if fut.done():
                ok, msg = fut.result().success, fut.result().message
                if not ok:
                    raise RuntimeError(msg)
                return {'ok': True, 'pose': msg}
            time.sleep(0.02)
        raise RuntimeError('relocalize service timeout')

    def navpoints_load(self):
        if not os.path.exists(NAV_POINTS_FILE):
            return []
        try:
            data = json.load(open(NAV_POINTS_FILE, 'r'))
            return data if isinstance(data, list) else []
        except Exception:
            return []

    def navpoints_save(self, points):
        tmp = NAV_POINTS_FILE + '.tmp'
        with open(tmp, 'w') as handle:
            json.dump(points, handle)
        os.replace(tmp, NAV_POINTS_FILE)

    def navpoint_add(self, name):
        name = re.sub(r'[^A-Za-z0-9._\-\u4e00-\u9fff]+', '_', (name or '').strip())
        if not name:
            raise RuntimeError('导航点名称不能为空')
        pose = self.state.pose
        if not pose:
            raise RuntimeError('当前位姿不可用')
        points = self.navpoints_load()
        if any(point.get('name') == name for point in points):
            raise RuntimeError('导航点已存在: ' + name)
        points.append({
            'name': name,
            'x': round(pose['x'], 4),
            'y': round(pose['y'], 4),
            'yaw': round(pose['yaw'], 4),
        })
        self.navpoints_save(points)
        return {'ok': True, 'points': points}

    def navpoint_delete(self, name):
        points = [point for point in self.navpoints_load() if point.get('name') != name]
        self.navpoints_save(points)
        return {'ok': True, 'points': points}

    def route_run(self, names):
        if not isinstance(names, list) or not names:
            raise RuntimeError('路线为空')
        points = self.navpoints_load()
        ordered = []
        for name in names:
            match = next((point for point in points if point.get('name') == name), None)
            if match is None:
                raise RuntimeError('导航点不存在: ' + name)
            ordered.append(match)
        self.route_points = ordered
        self.route_index = 0
        self.route_active = True
        self.route_step()
        return {'ok': True, 'total': len(ordered)}

    def route_step(self):
        if not self.route_active or self.route_index >= len(self.route_points):
            self.route_active = False
            self.route_points = []
            self.state.set('status', 'succeeded')
            self.state.set('goal', None)
            return
        point = self.route_points[self.route_index]
        self.route_index += 1
        self.go(point['x'], point['y'], point.get('yaw', 0.0))

    def route_cancel(self):
        self.route_active = False
        self.route_points = []
        self.cancel()
        return {'ok': True}

    def cart_request(self, method, path, body=None):
        data = json.dumps(body or {}).encode() if body is not None else None
        request = urllib.request.Request(CART_API + path, data=data, method=method)
        if data is not None:
            request.add_header('Content-Type', 'application/json')
        try:
            with urllib.request.urlopen(request, timeout=30) as response:
                raw = response.read().decode() or '{}'
                return json.loads(raw)
        except urllib.error.HTTPError as error:
            raw = error.read().decode() or error.reason
            try:
                message = json.loads(raw).get('error', raw)
            except Exception:
                message = raw
            raise RuntimeError(message)
        except urllib.error.URLError as error:
            raise RuntimeError('Cartographer web api unreachable: %s' % error.reason)

    def mgr_request(self, method, path):
        request = urllib.request.Request(MGR_API + path, method=method)
        try:
            with urllib.request.urlopen(request, timeout=60) as response:
                raw = response.read().decode() or '{}'
                return json.loads(raw)
        except urllib.error.HTTPError as error:
            raw = error.read().decode() or error.reason
            try:
                message = json.loads(raw).get('error', raw)
            except Exception:
                message = raw
            raise RuntimeError(message)
        except urllib.error.URLError as error:
            raise RuntimeError('Cartographer manager unreachable: %s' % error.reason)

    def mapping_start(self):
        # 先切到建图容器（管理器会自动停掉定位容器）
        self.mgr_request('POST', '/mapping')
        status = None
        deadline = time.time() + 25.0
        while time.time() < deadline:
            try:
                status = self.cart_request('GET', '/status')
                break
            except Exception:
                time.sleep(0.5)
        if status is None:
            raise RuntimeError('Cartographer web api unreachable')
        if not status.get('mapping'):
            self.cart_request('POST', '/start')
        self.state.set('mapping', True)
        return {'ok': True, 'mapping': True}

    def mapping_save(self, name=None):
        name = re.sub(r'[^A-Za-z0-9._-]+', '_', (name or '').strip())
        if not name:
            name = datetime.datetime.utcnow().strftime('%Y%m%dT%H%M%SZ')
        result = self.cart_request('POST', '/save?' + urllib.parse.urlencode({'name': name}))
        self.state.set('mapping_saved', result.get('dir'))
        # 保存后自动切回定位并加载新图 → 建完立即能导航
        self.mgr_request('POST', '/localize?' + urllib.parse.urlencode({'name': name}))
        self.state.set('mapping', False)
        self.state.set('current_map', name)
        return {'ok': True, 'name': result.get('name', name), 'dir': result.get('dir')}

    def maps_list(self):
        maps = []
        if os.path.isdir(MAPS_ROOT):
            for entry in sorted(os.listdir(MAPS_ROOT)):
                if os.path.exists(os.path.join(MAPS_ROOT, entry, 'feedbot.yaml')):
                    maps.append({'name': entry})
        return {'maps': maps}

    def load_static_map(self, name):
        name = re.sub(r'[^A-Za-z0-9._-]+', '_', (name or '').strip())
        map_dir = os.path.join(MAPS_ROOT, name)
        yaml_path = os.path.join(map_dir, 'feedbot.yaml')
        if not name or not os.path.exists(yaml_path):
            raise RuntimeError('地图不存在: ' + name)
        yaml_text = open(yaml_path, 'r').read()
        resolution = 0.05
        origin = [0.0, 0.0]
        image = 'feedbot.pgm'
        for line in yaml_text.splitlines():
            line = line.split('#', 1)[0].strip()
            if line.startswith('image:'):
                image = line.split(':', 1)[1].strip()
            elif line.startswith('resolution:'):
                try:
                    resolution = float(line.split(':', 1)[1].strip())
                except ValueError:
                    pass
            elif line.startswith('origin:'):
                try:
                    body = line.split(':', 1)[1].strip().strip('[]')
                    parts = [float(part) for part in body.split(',')[:2]]
                    origin = parts
                except Exception:
                    pass
        image_path = os.path.join(map_dir, image)
        if not os.path.isabs(image_path):
            image_path = os.path.abspath(image_path)
        if not os.path.exists(image_path):
            raise RuntimeError('地图图片不存在: ' + image)
        raw = open(image_path, 'rb').read()
        tokens = []
        index = 0
        while len(tokens) < 4:
            while index < len(raw) and raw[index:index + 1] in b' \t\r\n':
                index += 1
            if raw[index:index + 1] == b'#':
                while raw[index] != 10:
                    index += 1
                continue
            start = index
            while index < len(raw) and raw[index:index + 1] not in b' \t\r\n':
                index += 1
            tokens.append(raw[start:index].decode())
        magic, width, height, max_value = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
        index += 1
        pixels = raw[index:index + width * height]

        def pixel_to_occ(value):
            if value >= 205:
                return 0
            if value <= 89:
                return 100
            return -1

        row_major = [pixel_to_occ(value) for value in pixels]
        data = []
        for row in range(height - 1, -1, -1):
            data.extend(row_major[row * width:(row + 1) * width])
        self.state.set('map', dict(
            width=width, height=height, resolution=resolution,
            origin=[origin[0], origin[1]], data=data))
        self.state.set('current_map', name)
        return {'ok': True, 'name': name}

    def maps_switch(self, name):
        # 切换 Cartographer 定位到该地图，再更新网页显示
        self.mgr_request('POST', '/localize?' + urllib.parse.urlencode({'name': name}))
        self.state.set('mapping', False)
        return self.load_static_map(name)

    def maps_delete(self, name):
        name = re.sub(r'[^A-Za-z0-9._-]+', '_', (name or '').strip())
        if not name:
            raise RuntimeError('地图名不能为空')
        map_dir = os.path.join(MAPS_ROOT, name)
        if not os.path.isdir(map_dir):
            raise RuntimeError('地图不存在: ' + name)
        shutil.rmtree(map_dir)
        return {'ok': True, 'name': name}

    def mapping_stop(self):
        self.cart_request('POST', '/finish')
        self.state.set('mapping', False)
        return {'ok': True, 'mapping': False}

    def load_dock_state(self):
        path = os.path.join(MAPS_ROOT, 'charge_dock_config.json')
        try:
            with open(path, 'r', encoding='utf-8') as f:
                cfg = json.load(f)
        except Exception:
            return
        for key in ('dock', 'pre_dock', 'docked_pose'):
            value = cfg.get(key)
            if value and all(k in value for k in ('x', 'y', 'yaw')):
                self.state.set(key, {'x': float(value['x']),
                                     'y': float(value['y']),
                                     'yaw': float(value['yaw'])})

    def dock_get(self):
        ok, msg = self.call_trigger(self.dock_get_client)
        return json.loads(msg)

    def dock_calibrate(self):
        ok, msg = self.call_trigger(self.dock_calibrate_client, timeout=15.0,
                                    service_name='dock calibrate')
        if not ok:
            raise RuntimeError(msg)
        info = json.loads(msg)
        cfg = {
            'dock': info['dock'],
            'pre_dock': info['pre_dock'],
            'template': info['template'],
        }
        path = os.path.join(MAPS_ROOT, 'charge_dock_config.json')
        tmp = path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(cfg, f, ensure_ascii=False)
        os.replace(tmp, path)
        self.call_trigger(self.dock_reload_client, timeout=5.0,
                          service_name='dock reload')
        self.state.set('dock', info['dock'])
        self.state.set('pre_dock', info['pre_dock'])
        self.state.set('docked_pose', None)
        return {'dock': info['dock'], 'pre_dock': info['pre_dock'],
                'template_points': len(info.get('template', {}).get('points', []))}

    def dock_calibrate_docked(self):
        ok, msg = self.call_trigger(self.dock_calibrate_docked_client,
                                    timeout=10.0, service_name='dock docked calibrate')
        if not ok:
            raise RuntimeError(msg)
        info = json.loads(msg)
        path = os.path.join(MAPS_ROOT, 'charge_dock_config.json')
        cfg = {}
        try:
            with open(path, 'r', encoding='utf-8') as f:
                cfg = json.load(f)
        except Exception:
            cfg = {}
        cfg['docked_pose'] = info['docked_pose']
        tmp = path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(cfg, f, ensure_ascii=False)
        os.replace(tmp, path)
        self.call_trigger(self.dock_reload_client, timeout=5.0,
                          service_name='dock reload')
        self.state.set('docked_pose', info['docked_pose'])
        return {'docked_pose': info['docked_pose']}

    def dock_undock(self):
        # 触发脱桩，并等待 charge 恢复 0 且脱桩状态机真正结束。
        # 定位无需在此重扫：回桩时已用 docked_pose 自动播种，脱桩过程由
        # Cartographer 连续跟踪。
        self.manual_stop()
        ok, msg = self.call_trigger(self.dock_undock_client, timeout=15.0)
        if not ok:
            raise RuntimeError(msg)
        deadline = time.time() + 20.0
        while time.time() < deadline:
            try:
                info = self.dock_get()
                still_undocking = bool(info.get('undocking', False))
            except Exception:
                still_undocking = True
            if getattr(self, 'charge_status', 0) == 0 and not still_undocking:
                self.state.set('status', 'idle')
                return {'ok': True}
            time.sleep(0.3)
        raise RuntimeError('脱桩超时')

    def dock_start(self):
        self.manual_stop()
        info = self.dock_get()
        pre = info.get('pre_dock')
        if not pre:
            raise RuntimeError('dock not calibrated')
        # 1) 导航到预对接点
        if not self.nav_client.wait_for_server(timeout_sec=5):
            raise RuntimeError('navigator not available')
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = pre[0]
        goal.pose.pose.position.y = pre[1]
        yaw_goal = pre[2]
        goal.pose.pose.orientation.z = math.sin(yaw_goal/2)
        goal.pose.pose.orientation.w = math.cos(yaw_goal/2)
        self.state.set('status', 'docking_nav')
        self.state.set('goal', dict(x=pre[0], y=pre[1], yaw=yaw_goal))
        self.dock_nav_active = True
        self.dock_nav_result_future = None
        self.dock_start_future = None
        self.dock_nav_future = self.nav_client.send_goal_async(goal)
        return dict(accepted='pending')

    def dock_cancel(self):
        if self.dock_nav_active and hasattr(self, 'dock_nav_future') and self.dock_nav_future.done():
            try:
                self.dock_nav_future.result().cancel_goal_async()
            except Exception:
                pass
        self.dock_nav_active = False
        self.dock_nav_result_future = None
        self.dock_start_future = None
        self.call_trigger(self.dock_cancel_client)
        self.state.set('status', 'cancelled')
        return dict(ok=True)

    def dock_tick(self):
        # 导航到预对接点完成后触发回充
        if not self.dock_nav_active or not hasattr(self, 'dock_nav_future'):
            return
        if not self.dock_nav_future.done():
            return
        try:
            handle = self.dock_nav_future.result()
        except Exception:
            self.dock_nav_active = False
            self.state.set('status', 'aborted')
            self.state.set('nav_error', 'dock nav send failed')
            return
        if not handle.accepted:
            self.dock_nav_active = False
            self.state.set('status', 'aborted')
            self.state.set('nav_error', 'dock nav rejected')
            return
        rf = getattr(self, 'dock_nav_result_future', None)
        if rf is None:
            self.dock_nav_result_future = handle.get_result_async()
            return
        if not rf.done():
            return
        result = rf.result()
        r = result.result if hasattr(result, 'result') else result
        self.dock_nav_active = False
        self.dock_nav_result_future = None
        if r.error_code != 0:
            self.state.set('status', 'aborted')
            self.state.set('nav_error', 'dock nav failed: %s' % err_text(r))
            return
        # 异步触发回充节点（timer 回调内不能阻塞等待 service）
        if not hasattr(self, 'dock_start_future') or self.dock_start_future is None:
            self.dock_start_future = self.dock_start_client.call_async(Trigger.Request())
            return
        if not self.dock_start_future.done():
            return
        try:
            r = self.dock_start_future.result()
            ok, msg = r.success, r.message
        except Exception as e:
            ok, msg = False, str(e)
        self.dock_start_future = None
        self.state.set('status', 'docking' if ok else 'aborted')
        if not ok:
            self.state.set('nav_error', 'dock start failed: %s' % msg)

    def auto_reloc_check(self):
        return
    def auto_charge_tick(self):
        if not self.auto_charge:
            return
        power = getattr(self, 'power', None)
        status = self.state.status

        # 回充恢复（电量 ≥ 90）→ 重新武装，允许下次低电再触发
        if self.auto_charge_triggered and power is not None and power >= 90:
            self.auto_charge_triggered = False
            self.auto_charge_retry_at = 0.0
            self.get_logger().info('auto charge recovered at %d%%, re-armed' % power)
            return

        if self.auto_charge_triggered:
            # 回充失败/中止后允许重试，带 60s 冷却避免死循环
            if status in ('aborted', 'cancelled'):
                now = time.time()
                if self.auto_charge_retry_at == 0.0:
                    self.auto_charge_retry_at = now + 60.0
                elif now >= self.auto_charge_retry_at:
                    self.auto_charge_triggered = False
                    self.auto_charge_retry_at = 0.0
                    self.get_logger().info('auto charge retry re-armed after failure')
            return

        # 低电量 20 触发；回充/导航中不重复触发。
        # 放到后台线程执行，避免在 update 定时器里阻塞执行器等待 service 响应。
        if power is not None and power < 20 and status not in (
                'docking_nav', 'docking', 'navigating', 'charging'):
            self.auto_charge_triggered = True
            self.get_logger().info('low power %d%%, auto charge triggered' % power)
            threading.Thread(target=self._auto_dock_start, daemon=True).start()

    def _auto_dock_start(self):
        try:
            self.dock_start()
        except Exception as e:
            self.auto_charge_triggered = False
            self.state.set('nav_error', 'auto charge failed: %s' % e)

    def dock_status_tick(self):
        if self.state.status != 'docking':
            return
        now = time.time()
        if now - self.dock_status_at < 1.0:
            return
        self.dock_status_at = now
        try:
            info = self.dock_get()
        except Exception:
            return
        if info.get('charging'):
            self.state.set('status', 'charging')
            self.state.set('nav_error', None)
        elif info.get('failed'):
            self.state.set('status', 'aborted')
            self.state.set('nav_error', '回充失败: %s' % info.get('failure_reason', ''))

    def charge_status_tick(self):
        """以 MCU 充电状态为准刷新 UI 状态（重启后也能正确显示充电中）。"""
        charge = getattr(self, 'charge_status', 0)
        status = self.state.status
        if charge > 0:
            if status in ('navigating', 'undocking'):
                return
            if status != 'charging':
                self.state.set('status', 'charging')
                self.state.set('nav_error', None)
        else:
            if status == 'charging':
                self.state.set('status', 'idle')
                self.state.set('nav_error', None)

    def preview(self, x, y, yaw_goal):
        if not self.path_client.wait_for_server(timeout_sec=5):
            raise RuntimeError('planner not available')
        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = 'map'
        goal.goal.pose.position.x = x
        goal.goal.pose.position.y = y
        goal.goal.pose.orientation.z = math.sin(yaw_goal/2)
        goal.goal.pose.orientation.w = math.cos(yaw_goal/2)
        goal.planner_id = 'GridBased'
        future = self.path_client.send_goal_async(goal)
        deadline = time.time() + 10
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        if not future.done():
            raise RuntimeError('preview timeout')
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError('preview rejected')
        result_future = handle.get_result_async()
        deadline = time.time() + 15
        while not result_future.done() and time.time() < deadline:
            time.sleep(0.02)
        if not result_future.done():
            raise RuntimeError('preview result timeout')
        result = result_future.result()
        r = result.result if hasattr(result, 'result') else result
        if r.error_code != 0:
            raise RuntimeError('预览失败：%s' % err_text(r))
        path = [dict(x=pp.pose.position.x, y=pp.pose.position.y) for pp in r.path.poses]
        self.state.set('plan', path)
        self.state.set('status', 'previewing')
        return dict(path=path, length=len(path))

    def go(self, x, y, yaw_goal):
        self.manual_stop()
        if getattr(self, 'charge_status', 0) > 0:
            self.state.set('status', 'undocking')
            self.dock_undock()
        if not self.nav_client.wait_for_server(timeout_sec=5):
            raise RuntimeError('navigator not available')
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw_goal/2)
        goal.pose.pose.orientation.w = math.cos(yaw_goal/2)
        self.state.set('goal', dict(x=x, y=y, yaw=yaw_goal))
        self.state.set('status', 'navigating')
        self.state.set('nav_error', None)
        self.result_future = None
        self.nav_future = self.nav_client.send_goal_async(goal)
        self.nav_sent = time.time()
        return dict(accepted='pending')

    def manual_vel(self, vx, wz):
        """网页遥杆直控：向 /wheel/cmd_vel 发布 TwistStamped。"""
        vx = max(-0.4, min(0.4, float(vx)))
        wz = max(-0.8, min(0.8, float(wz)))
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist.linear.x = vx
        out.twist.angular.z = wz
        self.manual_vel_pub.publish(out)
        return {'ok': True, 'vx': vx, 'wz': wz}

    def manual_stop(self):
        if getattr(self, 'manual_vel_pub', None) is not None:
            self.manual_vel(0.0, 0.0)

    def update(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            tr = t.transform
            q = tr.rotation
            yawv = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            self.state.set('pose', dict(x=tr.translation.x, y=tr.translation.y, yaw=yawv))
        except TransformException:
            pass
        self.tick_nav()
        self.dock_tick()
        self.charge_status_tick()
        self.dock_status_tick()
        self.auto_charge_tick()
        self.auto_reloc_check()

    def tick_nav(self):
        nav_future = getattr(self, 'nav_future', None)
        if nav_future is None or not nav_future.done():
            return
        try:
            handle = nav_future.result()
        except Exception:
            self.state.set('status', 'aborted')
            self.state.set('nav_error', '发送导航目标失败')
            self.nav_future = None
            self.result_future = None
            return
        if not handle.accepted:
            self.state.set('status', 'aborted')
            self.state.set('nav_error', '导航目标被拒绝')
            self.nav_future = None
            self.result_future = None
            return
        result_future = getattr(self, 'result_future', None)
        if result_future is None:
            self.result_future = handle.get_result_async()
            return
        if not result_future.done():
            return
        result = result_future.result()
        r = result.result if hasattr(result, 'result') else result
        if r.error_code == 0:
            self.state.set('nav_error', None)
            if self.route_active and self.route_index < len(self.route_points):
                self.nav_future = None
                self.result_future = None
                self.route_step()
                return
            self.state.set('status', 'succeeded')
        else:
            self.route_active = False
            self.route_points = []
            self.state.set('status', 'aborted')
            self.state.set('nav_error', err_text(r))
        self.nav_future = None
        self.result_future = None

    def cancel(self):
        if hasattr(self, 'nav_future') and self.nav_future.done():
            try:
                self.nav_future.result().cancel_goal_async()
            except Exception:
                pass
        self.nav_future = None
        self.result_future = None
        self.state.set('status', 'cancelled')


class Handler(BaseHTTPRequestHandler):
    node = None
    state = None

    def log_message(self, *_):
        pass

    def reply(self, status, body):
        if isinstance(body, (dict, list)):
            body = json.dumps(body).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == '/api/state':
            sn = self.state.snapshot()
            return self.reply(200, {k: v for k, v in sn.items() if k not in ('map', 'raw_map', 'airwall')})
        if self.path == '/api/map':
            sn = self.state.snapshot()
            return self.reply(200, {'map': sn.get('map'), 'footprint': sn.get('footprint')})
        if self.path == '/api/maps':
            return self.reply(200, self.node.maps_list())
        if self.path == '/api/navpoints':
            return self.reply(200, {'points': self.node.navpoints_load()})
        if self.path == '/api/stream':
            return self.stream()
        self.reply(404, {'error': 'unknown'})

    def stream(self):
        import queue
        q = queue.Queue(maxsize=100)
        def notify(key, value):
            if key in ('pose', 'plan', 'local_plan', 'status', 'goal'):
                try:
                    q.put_nowait((key, value))
                except queue.Full:
                    pass
        self.state.add_listener(notify)
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.end_headers()
        # 初始快照
        sn = self.state.snapshot()
        for key in ('pose', 'plan', 'local_plan', 'status', 'goal'):
            self.wfile.write(('data: ' + json.dumps({'k': key, 'v': sn.get(key)}) + '\n\n').encode())
            self.wfile.flush()
        try:
            while True:
                try:
                    key, value = q.get(timeout=15)
                except queue.Empty:
                    self.wfile.write(b': ping\n\n')
                    self.wfile.flush()
                    continue
                self.wfile.write(('data: ' + json.dumps({'k': key, 'v': value}) + '\n\n').encode())
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):
        size = int(self.headers.get('Content-Length', '0'))
        if not 0 < size <= 4096:
            return self.reply(400, {'error': 'bad size'})
        body = json.loads(self.rfile.read(size))
        try:
            if self.path == '/api/preview':
                return self.reply(200, self.node.preview(body['x'], body['y'], body.get('yaw', 0)))
            if self.path == '/api/goal':
                return self.reply(200, self.node.go(body['x'], body['y'], body.get('yaw', 0)))
            if self.path == '/api/cancel':
                self.node.cancel()
                return self.reply(200, {'ok': True})
            if self.path == '/api/relocalize':
                return self.reply(200, self.node.relocalize())
            if self.path == '/api/manual_vel':
                return self.reply(200, self.node.manual_vel(
                    body.get('vx', 0), body.get('wz', 0)))
            if self.path == '/api/mapping/start':
                return self.reply(200, self.node.mapping_start())
            if self.path == '/api/mapping/save':
                return self.reply(200, self.node.mapping_save(body.get('name')))
            if self.path == '/api/mapping/stop':
                return self.reply(200, self.node.mapping_stop())
            if self.path == '/api/maps/switch':
                return self.reply(200, self.node.maps_switch(body.get('name')))
            if self.path == '/api/maps/delete':
                return self.reply(200, self.node.maps_delete(body.get('name')))
            if self.path == '/api/navpoints/add':
                return self.reply(200, self.node.navpoint_add(body.get('name')))
            if self.path == '/api/navpoints/delete':
                return self.reply(200, self.node.navpoint_delete(body.get('name')))
            if self.path == '/api/route/run':
                return self.reply(200, self.node.route_run(body.get('names')))
            if self.path == '/api/route/cancel':
                return self.reply(200, self.node.route_cancel())
            if self.path == '/api/dock/get':
                return self.reply(200, self.node.dock_get())
            if self.path == '/api/dock/calibrate':
                return self.reply(200, self.node.dock_calibrate())
            if self.path == '/api/dock/calibrate_docked':
                return self.reply(200, self.node.dock_calibrate_docked())
            if self.path == '/api/dock/start':
                return self.reply(200, self.node.dock_start())
            if self.path == '/api/dock/cancel':
                return self.reply(200, self.node.dock_cancel())
            if self.path == '/api/dock/undock':
                return self.reply(200, self.node.dock_undock())
        except Exception as error:
            return self.reply(500, {'error': str(error)})
        self.reply(404, {'error': 'unknown'})


def main():
    rclpy.init()
    state = State()
    node = NavNode(state)
    Handler.node = node
    Handler.state = state
    server = ThreadingHTTPServer(('0.0.0.0', 8098), Handler)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    print('nav bridge at 0.0.0.0:8098')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

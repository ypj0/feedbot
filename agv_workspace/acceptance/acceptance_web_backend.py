#!/usr/bin/env python3
"""实车验证网页后端 - HTTP + WebSocket + MotionAuthority 集成

运行方式：
  python3 acceptance_web_backend.py

访问：http://10.7.5.88:8765
"""
import asyncio
import json
import math
import sys
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from diagnostic_msgs.msg import DiagnosticArray
from feedbot_motion_interfaces.msg import MotionRequest
from feedbot_motion_interfaces.srv import AcquireLease, ReleaseLease

try:
    from aiohttp import web
except ImportError:
    print("❌ 需要安装 aiohttp")
    print("   pip install aiohttp")
    sys.exit(1)


class AcceptanceBackend:
    def __init__(self, port=8765):
        self.port = port
        self.clients = set()
        self.lease_id = None
        self.sensor_data = {
            "scan_rate_hz": 0,
            "imu_rate_hz": 0,
            "odom_rate_hz": 0,
            "filtered_rate_hz": 0,
            "scan_count": 0,
            "imu_count": 0,
            "odom_count": 0,
            "filtered_count": 0,
            "odom_x": 0,
            "odom_y": 0,
            "odom_yaw": 0,
            "odom_vx": 0,
            "odom_wz": 0,
            "diagnostics": {},
        }
        self.timestamps = {
            "scan": 0, "imu": 0, "odom": 0, "filtered": 0
        }

        # ROS 2 初始化
        rclpy.init()
        self.node = rclpy.create_node("acceptance_web_backend")

        # 获取租约
        self._acquire_lease()

        # 发布者和订阅者
        self.motion_pub = self.node.create_publisher(MotionRequest, "/motion/request", 1)
        self.node.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.node.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.node.create_subscription(Odometry, "/wheel/odom", self._on_odom, 10)
        self.node.create_subscription(Odometry, "/odometry/filtered", self._on_filtered, 10)
        self.node.create_subscription(DiagnosticArray, "/diagnostics", self._on_diag, 10)

        # ROS spinner 线程
        self.ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self.ros_thread.start()

    def _acquire_lease(self):
        lease_client = self.node.create_client(AcquireLease, "/motion/acquire_lease")
        while rclpy.ok() and not lease_client.wait_for_service(timeout_sec=1.0):
            pass
        req = AcquireLease.Request()
        req.source = 1  # WEB_CONTROL
        future = lease_client.call_async(req)
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.01)
        result = future.result()
        if result.granted:
            self.lease_id = result.lease_id
            print(f"✓ 获得租约 {self.lease_id}")
        else:
            print(f"✗ 获取租约失败: {result.reason}")

    def _ros_spin(self):
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def _on_scan(self, msg):
        self.sensor_data["scan_count"] += 1
        now = time.monotonic()
        if self.timestamps["scan"] > 0:
            self.sensor_data["scan_rate_hz"] = 1.0 / (now - self.timestamps["scan"])
        self.timestamps["scan"] = now

    def _on_joints(self, msg):
        self.sensor_data["imu_count"] += 1
        now = time.monotonic()
        if self.timestamps["imu"] > 0:
            self.sensor_data["imu_rate_hz"] = 1.0 / (now - self.timestamps["imu"])
        self.timestamps["imu"] = now

    def _on_odom(self, msg):
        self.sensor_data["odom_count"] += 1
        now = time.monotonic()
        if self.timestamps["odom"] > 0:
            self.sensor_data["odom_rate_hz"] = 1.0 / (now - self.timestamps["odom"])
        self.timestamps["odom"] = now
        self.sensor_data["odom_x"] = msg.pose.pose.position.x
        self.sensor_data["odom_y"] = msg.pose.pose.position.y
        self.sensor_data["odom_vx"] = msg.twist.twist.linear.x
        self.sensor_data["odom_wz"] = msg.twist.twist.angular.z
        q = msg.pose.pose.orientation
        self.sensor_data["odom_yaw"] = math.atan2(
            2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)
        )

    def _on_filtered(self, msg):
        self.sensor_data["filtered_count"] += 1
        now = time.monotonic()
        if self.timestamps["filtered"] > 0:
            self.sensor_data["filtered_rate_hz"] = 1.0 / (now - self.timestamps["filtered"])
        self.timestamps["filtered"] = now

    def _on_diag(self, msg):
        for status in msg.status:
            self.sensor_data["diagnostics"][status.name] = {
                "level": int(status.level[0]) if isinstance(status.level, (bytes, bytearray)) else int(status.level),
                "message": status.message,
                "values": {v.key: v.value for v in status.values}
            }

    async def handle_websocket(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        self.clients.add(ws)

        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    try:
                        cmd = json.loads(msg.data)
                        await self.process_command(cmd)
                    except json.JSONDecodeError:
                        pass
        finally:
            self.clients.discard(ws)

        return ws

    async def process_command(self, cmd):
        action = cmd.get("action")
        if action == "move":
            linear = cmd.get("linear", 0.0)
            angular = cmd.get("angular", 0.0)
            self.send_motion(linear, angular)

    def send_motion(self, linear, angular):
        if self.lease_id is None:
            return
        msg = MotionRequest()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.source = 1
        msg.lease_id = self.lease_id
        msg.seq = 0
        msg.twist = Twist()
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.motion_pub.publish(msg)

    async def handle_index(self, request):
        """提供 HTML 文件"""
        html_path = Path(__file__).parent / "acceptance.html"
        if not html_path.exists():
            return web.Response(text="❌ 找不到 acceptance.html", status=404)

        with open(html_path, 'r', encoding='utf-8') as f:
            return web.Response(text=f.read(), content_type='text/html')

    async def broadcast_status(self):
        """每 100ms 广播一次传感器状态"""
        while True:
            await asyncio.sleep(0.1)
            status = {
                "type": "status",
                "data": self.sensor_data
            }
            for client in self.clients.copy():
                try:
                    await client.send_json(status)
                except:
                    pass

    async def start(self):
        # 创建 aiohttp 应用
        app = web.Application()
        app.router.add_get("/", self.handle_index)
        app.router.add_get("/acceptance.html", self.handle_index)
        app.router.add_get("/ws", self.handle_websocket)

        # 启动广播任务
        asyncio.create_task(self.broadcast_status())

        # 启动服务器
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, "0.0.0.0", self.port)
        await site.start()

        print("")
        print("✓ HTTP 服务启动")
        print("✓ WebSocket 服务启动")
        print("")
        print(f"📍 访问地址: http://0.0.0.0:{self.port}")
        print(f"🌐 或: http://10.7.5.88:{self.port}")
        print(f"🌐 或: http://localhost:{self.port} (SSH 转发)")
        print("")
        print("按 Ctrl+C 停止服务")
        print("")

        # 永不停止
        await asyncio.Future()


async def main():
    backend = AcceptanceBackend(port=8765)
    try:
        await backend.start()
    except KeyboardInterrupt:
        print("\n👋 已停止")


if __name__ == "__main__":
    asyncio.run(main())

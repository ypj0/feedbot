#!/usr/bin/env python3
"""充电桩标定：读取当前 AMCL 位姿，计算桩位姿并持久化。

约定：标定时车辆静止在充电桩前方 0.5m，车头朝向充电桩。
- dock = 车位置 + 0.5m * 车头方向
- dock.yaw = 车 yaw（回充时掉头 180° 后倒车入桩，车尾触点朝桩）
- pre_dock = 当前车位姿（回充时先导航到该预对接点）
"""
import json
import math
import pathlib
import subprocess
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    node = rclpy.create_node('calibrate_dock')
    latest = {}
    def cb(msg):
        p = msg.pose.pose
        latest['pose'] = [p.position.x, p.position.y, yaw(p.orientation)]
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', cb, 1)
    import time
    end = time.time() + 3.0
    while time.time() < end and 'pose' not in latest:
        rclpy.spin_once(node, timeout_sec=0.1)
    if 'pose' not in latest:
        print('no amcl pose')
        sys.exit(1)
    x, y, th = latest['pose']
    offset = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
    dock = {
        'x': x + offset * math.cos(th),
        'y': y + offset * math.sin(th),
        'yaw': th,
    }
    pre_dock = {'x': x, 'y': y, 'yaw': th}
    cfg = {'dock': dock, 'pre_dock': pre_dock}
    print(json.dumps({'pose': [x, y, th], 'dock': dock,
                      'pre_dock': pre_dock}, ensure_ascii=False))
    # 写入本地文件（由部署流程同步到车机）
    out = pathlib.Path(__file__).parent / 'charge_dock_config.json'
    out.write_text(json.dumps(cfg, ensure_ascii=False) + '\n', encoding='utf-8')
    print('written to', out)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

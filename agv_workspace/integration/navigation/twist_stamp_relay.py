"""Twist -> TwistStamped relay with front-only obstacle stop.

nav2 velocity_smoother outputs Twist; the chassis subscribes TwistStamped on /wheel/cmd_vel.
正前方安全过滤：只有“前向距离 < stop_dist 且侧向 < half_width+margin”的激光点才视为阻挡，
侧面障碍物不参与判断。被阻挡时把前进速度截为 0，障碍消失后自动恢复。
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


class Relay(Node):
    def __init__(self):
        super().__init__('twist_stamp_relay')
        self.declare_parameter('stop_dist', 0.40)
        self.declare_parameter('half_width', 0.275)
        self.declare_parameter('margin', 0.0)
        self.stop_dist = self.get_parameter('stop_dist').value
        self.half_width = self.get_parameter('half_width').value
        self.margin = self.get_parameter('margin').value
        self.front_blocked = False
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel_smoothed', self.cmd_cb, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(TwistStamped, '/wheel/cmd_vel', 10)

    def scan_cb(self, msg):
        lateral_limit = self.half_width + self.margin
        blocked = False
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= 0.0 or r > self.stop_dist:
                continue
            a = msg.angle_min + i * msg.angle_increment
            dx = r * math.cos(a)
            dy = r * math.sin(a)
            if 0.0 < dx < self.stop_dist and abs(dy) < lateral_limit:
                blocked = True
                break
        self.front_blocked = blocked

    def cmd_cb(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist.linear.x = msg.linear.x
        out.twist.linear.y = msg.linear.y
        out.twist.angular.z = msg.angular.z
        if self.front_blocked and msg.linear.x > 0.0:
            out.twist.linear.x = 0.0
        self.pub.publish(out)


def main():
    rclpy.init()
    node = Relay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

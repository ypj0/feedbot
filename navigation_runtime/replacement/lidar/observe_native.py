#!/usr/bin/env python3
import time,json,statistics
import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
rclpy.init();node=rclpy.create_node('native_lidar_validation');ages=[];sizes=set();stamp_errors=0;last=None
start=time.monotonic()
def cb(m):
 global last,stamp_errors
 s=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 if last is not None and s<=last:stamp_errors+=1
 last=s;ages.append((node.get_clock().now().nanoseconds*1e-9-s)*1000);sizes.add(len(m.ranges))
sub=node.create_subscription(LaserScan,'/candidate/scan',cb,qos_profile_sensor_data)
while time.monotonic()-start<18:rclpy.spin_once(node,timeout_sec=.1)
print(json.dumps({'received_scans':len(ages),'beam_counts':sorted(sizes),'nonmonotonic_stamps':stamp_errors,'estimated_scan_age_ms':{'min':min(ages),'median':statistics.median(ages),'max':max(ages)} if ages else None}))
node.destroy_node();rclpy.shutdown()
if len(ages)<150 or stamp_errors or sizes!={1075}:raise SystemExit(2)

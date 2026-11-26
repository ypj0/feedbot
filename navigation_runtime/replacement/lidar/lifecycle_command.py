#!/usr/bin/env python3
import sys,rclpy
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
rclpy.init();n=rclpy.create_node('lidar_supervisor_transition');c=n.create_client(ChangeState,'/ltr2_driver/change_state')
if not c.wait_for_service(timeout_sec=5):raise SystemExit('lifecycle service unavailable')
q=ChangeState.Request();q.transition.id={'configure':1,'cleanup':2,'activate':3,'deactivate':4}[sys.argv[1]]
f=c.call_async(q);rclpy.spin_until_future_complete(n,f,timeout_sec=5);ok=f.done() and f.result() and f.result().success;n.destroy_node();rclpy.shutdown();sys.exit(0 if ok else 2)

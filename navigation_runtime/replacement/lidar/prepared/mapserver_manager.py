#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)
#
# Author: Jorge Santos
# from flask import Flask, request, jsonify
import subprocess
import heapq
import threading
import json 
import math
import copy

import PyKDL
import io
import os, sys, stat  # 加一个sys模块,再加一个stat修改文件权限
import time

import base64
import numpy as np
import cv2
from PIL import Image
from io import BytesIO

import roslaunch
import rospy
import yaml

from actionlib_msgs.msg import *

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from tf_conversions import transformations
from math import pi, radians, copysign, sqrt, sin, cos,atan2
import tf
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import *
from nav_msgs.srv import *
from cartographer_ros_msgs.srv import FinishTrajectory
from cartographer_ros_msgs.srv import WriteState
from cartographer_ros_msgs.msg import StatusResponse
from map_msgs.msg import *
from map_msgs.srv import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from std_srvs.srv import *
from visualization_msgs.msg import *
from maps_bean import *
# maps_bean 和geometry_msgs.msg的 Point类相同冲突 ，这里需要对maps_bean 的Point重命名
from maps_bean import Point as MapsBeanPoint
from maps_bean import GlobalPlan as MapsBeanGlobalplan
from geometry_msgs.msg import Point as gemoetryPoint


class MapManager:

    ##########################################################################
    # Initialization
    ###def read_allmapcontent(self):
    #######################################################################

    def read_param(self, url):
        with io.open(url, 'r', encoding="utf-8") as f:
            file_content = f.read()
            content = yaml.load(file_content)
        return content

    def writ_patam(self, url, content):
        try:
            with io.open(url, "w", encoding="utf-8") as f:
                yaml.dump(content, f)
                f.flush()  # 确保所有数据都被写入到磁盘
                os.fsync(f.fileno())  # 确保文件被同步到磁盘
            return True  # 写入成功
        except Exception as e:
            print("An error occurred:")  # 可选: 记录异常
            return False  # 写入失败

    # 写入 JSON 数据
    def write_json(self, data, url):
        data = json.dumps(data, default=lambda o: o.__dict__, ensure_ascii=False, indent=4)
        with io.open(url, "w", encoding="utf-8")as f:
            f.write(u'{}\n'.format(data))

    # 读取 JSON 数据
    def read_json(self, uri):
        data = json.load(io.open(uri, encoding="utf-8"))
        return data

    def bean_to_dict(self, object):
        dict = json.dumps(object, default=lambda o: o.__dict__, sort_keys=True)
        return json.loads(dict)

    def calculate_distance(self,point1, point2):
        return sqrt(pow((point1["positionX"] - point2["positionX"]),2) + pow((point1["positionY"] - point2["positionY"]),2))

    def dijkstra(self, lines, points, map_bean, start_point_id, end_point_id):
        graph = {}
        isfindpoint = False  # type: bool
        start_distance = -1.0
        start_id = 0
        for line in lines["nLines"]:
            start_point = next((point for point in points["points"] if point["pointId"] == line["startNid"]), None)
            if start_point is None:
                start_point = next((point for point in points["systemPoints"] if point["pointId"] == line["startNid"]), None)
            end_point = next((point for point in points["points"] if point["pointId"] == line["endNid"]), None)
            if end_point is None:
                # 为充电桩建高速做的备份
                end_point = next((point for point in points["systemPoints"] if point["pointId"] == line["endNid"]), None)

            if start_point and end_point:
                # rospy.loginfo("根据路线开始生成视图")
                distance = self.calculate_distance(start_point, end_point) * line["weight"]
                if line["direction"] == 1 or line["direction"] == 2:
                    if start_point["pointId"] not in graph:
                        graph[start_point["pointId"]] = []
                    graph[start_point["pointId"]].append((end_point["pointId"], distance))
                if line["direction"] == 0 or line["direction"] == 2:
                    if end_point["pointId"] not in graph:
                        graph[end_point["pointId"]] = []
                    graph[end_point["pointId"]].append((start_point["pointId"], distance))

            #检测终点是否在路线上
            if start_point["pointId"] == end_point_id or end_point["pointId"] == end_point_id:
                isfindpoint = True
            # 查找起点,也就是最近的点
            
            if start_point_id ==0:
                currentpose = self.get_pos()               
                if currentpose :
                    distanceS = sqrt(pow((start_point["positionX"] - currentpose.x), 2) + pow((start_point["positionY"] - currentpose.y), 2))
                    distanceE = sqrt(pow((end_point["positionX"] - currentpose.x), 2) + pow((end_point["positionY"] - currentpose.y), 2))
                else:
                    rospy.loginfo("没找到起点,tf异常")
                    return None
            else:
                start_point_bean = self.get_point_by_id(map_bean, start_point_id)
                if start_point_bean :
                    distanceS = sqrt(pow((start_point["positionX"] - start_point_bean.getPositionX()), 2) + pow((start_point["positionY"] - start_point_bean.getPositionY()), 2))
                    distanceE = sqrt(pow((end_point["positionX"] - start_point_bean.getPositionX()), 2) + pow((end_point["positionY"] - start_point_bean.getPositionY()), 2))
                else:
                    rospy.loginfo("没找到起点,tf异常")
                    return None

            if start_distance != -1.0:
                if distanceS < start_distance:
                    start_distance = distanceS
                    start_id = start_point["pointId"]
                if distanceE < start_distance:
                    start_distance = distanceE
                    start_id = end_point["pointId"]
            else:
                if distanceS<distanceE:
                    start_distance = distanceS
                    start_id = start_point["pointId"]
                else:
                    start_distance = distanceE
                    start_id = end_point["pointId"]

        if not isfindpoint: #终点不在路线上直接返回
            rospy.loginfo("终点不在路线上直接返回")
            return None
        if start_distance == -1 or start_id == 0:
            rospy.loginfo("没找到起点 2")
            return None
        start_point_id = start_id

        pq = [(0, start_point_id, [])]
        visited = set()

        while pq:
            (cost, current, path) = heapq.heappop(pq)
            if current in visited:
                continue
            path = path + [current]
            if current == end_point_id:
                return path
            visited.add(current)
            for neighbor, weight in graph.get(current, ()):
                if neighbor not in visited:
                    heapq.heappush(pq, (cost + weight, neighbor, path))
        rospy.loginfo("找不到最短路线")
        return None
    
    def getId(self):
        # 这里是id是 时间戳向后扩展三位
        time_pose = PoseWithCovarianceStamped()
        time_pose.header.stamp = rospy.Time.now()
        nsecs = time_pose.header.stamp.nsecs
        first_three_nsecs = int(str(nsecs).zfill(9)[:3])
        id = time_pose.header.stamp.secs *1000 +first_three_nsecs
        return id
    
    def getTimeSecs(self):
        time_pose = PoseWithCovarianceStamped()
        time_pose.header.stamp = rospy.Time.now()
        id = time_pose.header.stamp.secs
        return id

    def __init__(self):

        # system
        self.carto_build_launch = None
        self.carto_local_launch = None
        self.movebase_launch = None
        self.lidarcharge_launch = None
        self.pointcloud_launch = None
        self.pointcloud1_launch = None
        self.pointcloud2_launch = None
        self.rosbridge_launch = None
        self.camera_launch = None
        self.multirobotsAvoid_launch = None
        self.carto_rebuild_launch = None
        self.imu_launch = None
        self.scan_launch = None
        self.driver_launch = None
        self.waypoit_launch = None
        self.bag_launch = None
        self.backend_launch = None
        self.autoupgrade_launch = None
        self.mapserver_launch = None
        self.lora_launch = None
        self.ekf_launch = None

        self.movebase_launch_flag = False
        self.lidarcharge_launch_flag = False
        self.pointcloud_launch_flag = False
        self.camera_launch_flag = False
        self.multirobotsAvoid_launch_flag = False
        self.rosbridge_launch_flag = False
        self.carto_build_launch_flag = False
        self.carto_local_launch_flag = False
        self.carto_rebuild_launch_flag = False
        self.scan_launch_flag = False
        self.imu_launch_flag = False
        self.driver_launch_flag = False
        self.waypoint_launch_flag = False
        self.bag_launch_flag = False
        self.backend_launch_flag = False
        self.autoupgrade_launch_flag = False
        self.mapserver_launch_flag = False
        self.lora_launch_flag = False
        self.ekf_launch_flag = False

        self.startMultiRobotsAvoid = 0
        self.startcamera = True
        self.startImu = True
        self.startScan = True
        self.startDriver = True
        self.startWaypoint = True
        self.startBag = False
        self.startBackend = False
        self.startAutoupgrade = False
        self.startLora = False
        self.startDebug = False
        self.slaveMachine = False
        self.startEkf = False

        self.reboot_time = 0
        self.shutdown = 0
        self.car_type = 150

        self.testmultirobotavoidThread = None
        self.thread_running = False

        self.testmultirobotavoidThread1 = None
        self.thread_running1 = False

        # map 
        self.map_loading = False
        self.map_status = -1  # 当前是否处于建图状态 0 no 1 yes -1 not_status
        self.map_isEmpty = 1
        self.allMap = None
        self.local_point_list = []  # list
        self.local_systempoint_list = []  # list
        self.local_json_map = "" # type: str
        self.local_json_nglobalPlan = "" # type: str
        self.local_globalplans_list = []
        self.map_file = ""  # type: str
        self.build_points = []
        self.build_systempoints = []

        # init status
        self.init_status = 0
        self.list_driverstatus_srv = None
        self.app_status = 0
        self.update_status = 0
        init_flag = 0
        self.record_bag_type = 0
        self.isheart = False
        self.isMove = False

        self.initial_set_flag = False
        self.initial_set_timeout = 0      

        # for test interface
        self.testodom_value = 0
        self.testodom_type = 0
        self.testodom_switch = False

        # movebase
        self.movebase_ready = False
        self.movebase_heart = 0
        start_movebase_timeout = 0
        self.restartNavi = False

        # config
        self.need_navigation = True
        self.navigation_safeset = 0
        self.powerThreashold = 10  
        self.powerThreasholdMax = 60 #导航恢复电量
        self.startSimulator = False

        # 跟安卓主控板和STM32板通讯接口
        self.map_driverinfo = DriverEntry()
        self.map_driverinfo.MotorInitStatus = 99
        self.map_driverinfo.EmergencyStatus = 99
        self.map_driverinfo.MotorEnabled = 99
        self.map_driverinfo.Power = 100
        self.map_driverinfo.ChargeStatus = 99
        self.map_driverinfo.Mode = 0
        self.map_driverinfo.LeftAlarm = 99
        self.map_driverinfo.RightAlarm = 99
        self.map_driverinfo.MoveAlarm = 99
        self.map_driverinfo.StopAlarm = 99
        self.map_driverinfo.LeftEncoder = 0
        self.map_driverinfo.RightEncoder = 0
        self.map_driverinfo.Speedx = 0
        self.map_driverinfo.Speedth = 0

        driver_setting = dict()
        driver_setting.clear()
        driver_setting = self.read_param(robot_driver_param_url)

        # 导航
        self.navigation_status = 0
        self.publishlocation_miss= 0
        self.is_across_floor = False # init 是否跨楼层导航
        self.is_across_map = False   # 是否跨地图
        self.is_across_loadingMap = False # 用于切换地图后 是否可以导航的标记

        self.navi_queue_list = []               # 记录手动发出的 货物指令队列
        self.target_goal_lock = threading.Lock()  # 队列锁，避免增删查询异常
        self.navi_queue_isrun = False           # 队列是否运行中
        self.navi_queue_index = 0               # 队列当前任务的下标

        self.lora_call_id = 0           # 当前反馈的 lora 信号id

        self.current_floor_num = 0 # 当前楼层id
        self.current_floor_name = "" # type 
        self.current_map_id = 0
        self.current_map_name = ""
        self.current_jacking_status = 0

        self.elevator_floor = 0

        self.first_targetjump = False

        # 顶货或者卸货
        self.start_liftcheck = False
        self.start_putdowncheck = False

        self.use_kua = False
        self.kua_p_x = 0
        self.kua_p_y = 0

        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = 'map'
        self.current_pose.header.stamp = rospy.Time.now()  
        self.current_pose.pose.orientation.w = 1.0  

        self.send_initial_pose = 0
        self.initial_pose = PoseWithCovarianceStamped()

        self.location_missing = 0
        self.dwaspeed  = 0
        self.relocation_flag = 0

        self.save_position_flag = 0
        
        self.erasermode = 2  # 设置橡皮檫模式为小圆点模式
        self.sendmapdata = OccupancyGrid()

        self.recordbag = RecordBag()

        self.operated_targetGoals = []         #记录导航存储运行过的
        self.high_targetgoals = [] #NewTargetGoalList()     # 高优先级导航任务点，比如呼叫器
        self.low_targetgoals = [] #NewTargetGoalList()      # 低优先级导航任务点
        self.high_targetgoals_backup = [] #NewTargetGoalList()     # 高优先级导航任务点，比如呼叫器
        self.low_targetgoals_backup = [] #NewTargetGoalList()      # 低优先级导航任务点
        self.subtask_targetgoals = [] #NewTargetGoalList()  # 跨楼层未完成任务点

        self.target_goal = TargetGoalEntry()
        self.target_goal.floor_id = 0                # 要去的 楼层 地图 和 导航点 id
        self.target_goal.map_id = 0
        self.target_goal.point_id = 0
        self.target_goal.type = 0
        self.target_goal_x = 0
        self.target_goal_y = 0
        self.checkTargetNaviFail = False
        self.checkTargetNaviFail_count = 0
        self.navi_retry_times = 0
        self.navi_retry_times_max = 60
        self.current_floor_num = 0
        self.change_map_flag = False
        self.charge_relocation_change_map_flag = False
        self.start_elevator_pointid = 0
        self.start_innerelevator_pointid = 0

        self.task_circle = False
        self.circle_times = 0
        self.task_executing_wait_time = 5  # 队列运行点的等待间隔 秒
        self.task_current_wait_time = -1     # 队列中当前任务结束等待时间

        self.task_executing = 0            # 1: 导航点任务正在执行 0: 无导航点正在执行  3: 循环执行
        self.task_runs_status = False
        self.get_nexttask = False
        self.task_delay = 0                # 任务执行后等待时间
        self.changeTestTime = 5            # 充电测试 在充电桩上呆着的时间
        self.need_gotodefaultPoint = False

        self.planner_goal = PoseStamped()

        self.across_elevator_floor = 0 #切换地图时候的要按的电梯号
        self.across_elevator_point = MapsBeanPoint() # 切换地图后要定位的点详情,目标的电梯内
        self.across_elevator_entry = TargetGoalEntry()

        self.base_frame = '/base_link'
        self.scan_frame = '/scan'
        self.base_footprint_frame = '/base_footprint'
        self.scan_pose_pub = rospy.Publisher('/scan_pose', Pose, latch=True, queue_size=10)

        self.odom_frame = '/odom'
        self.map_frame = '/map'
        self.tf_listener = tf.TransformListener()

        # charge 
        self.mode = 0
        self.waitfornavi = False

        self.need_outofcharge = False
        self.fix_chargepoint_flag = False
        self.start_chargecheck = False
        self.lowpowerExecute = False #
        self.lowpowerChargepoint_add = False #

        # for sensors
        self.use_Sonar = False
        self.sonardata = [0, 0, 0, 0, 0, 0, 0]
        self.sonar_flag = 0

        self.lora_status = False
        self.esp_status = False

        self.imu_angle = gemoetryPoint()
        self.init_imu_angle = gemoetryPoint()

        self.wifilist = dict()
        self.wifilist.clear()
        self.network_report = 0

        self.cartographer_status = -1
        self.imu_status = 0
        self.scan_status = 0
        self.driver_status = 0
        self.movebae_timeout = 0

        self.scan_ready = False
        self.odom_ready = False
        self.map_ready = False
        self.localmap_ready = False
        self.imu_ready = False
        self.slam_ready = False
        self.driver_ready = 0

        self.laser_switch = 1
        self.laser_switch_timeout = 0

        # for clean task
        self.cleanArea_json =None # CleanAreas
        # for android interface
        self.AndroidAlarm = AndroidAlarm()
        self.AndroidAlarm.androidmsg_chargestatus = 0
        self.AndroidAlarm.androidmsg_navigationstatus = 2
        self.AndroidAlarm.androidmsg_outofchargepoint = 50
        self.AndroidAlarm.androidmsg_locationstatus = 0
        self.AndroidAlarm.androidmsg_motor1status = 0
        self.AndroidAlarm.androidmsg_sonarstatus = 0
        self.AndroidAlarm.androidmsg_leftmotorstatus = 0
        self.AndroidAlarm.androidmsg_rightmotorstatus = 0
        self.AndroidAlarm.androidmsg_camerastatus = 118
        self.AndroidAlarm.androidmsg_motor2status = 0
        self.AndroidAlarm.androidmsg_batterystatus = 124
        self.AndroidAlarm.androidmsg_emergencystatus = 0
        self.AndroidAlarm.androidmsg_motorenabledstatus = 0
        self.AndroidAlarm.androidmsg_mapstatus = 0
        self.AndroidAlarm.androidmsg_virtualmapstatus = 0
        self.AndroidAlarm.androidmsg_mapbuildstatus = 0
        self.AndroidAlarm.androidmsg_initstatus = 0
        self.AndroidAlarm.androidmsg_lidarstatus = 0
        self.AndroidAlarm.androidmsg_imustatus = 0
        self.AndroidAlarm.androidmsg_stm32status = 0
        self.AndroidAlarm.androidmsg_odomstatus = 0
        self.AndroidAlarm.androidmsg_navinetworkstatus = 0

        if not bool(driver_setting):
            rospy.loginfo("驱动配置文件为空！！！")
        try:
            self.dwaspeed = driver_setting["speed_dwa"]
            self.navigation_safeset = driver_setting["navigtaion_safeset"]
        except:
            rospy.loginfo("读取驱动速度值和安全配置为空！")
        
        tmpwifilist = self.read_param(wifi_param_url)
        if bool(tmpwifilist):
            self.wifilist = tmpwifilist.copy()

        try:
            self.startSimulator = rospy.get_param(mapserver_param + "startSimulator", False)
        except:
            self.startSimulator = False
        
        try:
            self.startcamera = rospy.get_param(mapserver_param + "startcamera", False)
        except:
            self.startcamera = True

        try:
            self.startMultiRobotsAvoid = rospy.get_param(mapserver_param + "startMultiRobotsAvoid", 0)
        except:
            self.startMultiRobotsAvoid = 0

        try:
            self.startImu = rospy.get_param(mapserver_param + "startImu", True)
        except:
            self.startImu = True

        try:
            self.startScan = rospy.get_param(mapserver_param + "startScan", True)
        except:
            self.startScan = True
        
        try:
            self.startDriver = rospy.get_param(mapserver_param + "startDriver", True)
        except:
            self.startDriver = True
        
        try:
            self.startWaypoint = rospy.get_param(mapserver_param + "startWaypoint", True)
        except:
            self.startWaypoint = True

        try:
            self.startBag = rospy.get_param(mapserver_param + "startBag", False)
        except:
            self.startBag = False        
       
        try:
            self.startBackend = rospy.get_param(mapserver_param + "startBackend", False)
        except:
            self.startBackend = False
        
        try:
            self.startAutoupgrade = rospy.get_param(mapserver_param + "startAutoupgrade", False)
        except:
            self.startAutoupgrade = False
        
        try:
            self.startLora = rospy.get_param(mapserver_param + "startLora", False)
        except:
            self.startLora = False
        
        try:
            self.startEkf = rospy.get_param(mapserver_param + "startEkf", False)
        except:
            self.startEkf = False

        try:
            self.startDebug = rospy.get_param(mapserver_param + "startDebug", False)
        except:
            self.startDebug = False        

        try:
            self.slaveMachine = rospy.get_param(mapserver_param + "slaveMachine", False)
        except:
            self.slaveMachine = False

        try:
            self.ledWarnTime = rospy.get_param(mapserver_param + "ledWarnTime", 1)
        except:
            self.ledWarnTime = 30
        
        try:
            self.car_type = rospy.get_param(mapserver_param + "car_type", 150)
        except:
            self.car_type = 150

        if not self.slaveMachine:
            os.system("timedatectl set-ntp false")

        # 启动IMU
        self.loading_imu()

        # 启动scan        
        from feedbot_factory_owner import FactoryOwner
        self.lidar_owner = FactoryOwner(self)
        self.loading_scan()

        # 启动rola
        self.loading_lora()

        # 启动Ekf
        self.loading_ekf()

        if self.startSimulator:
            self.scan_ready = True
            self.odom_ready = True
            self.map_ready = True
            self.localmap_ready = True
            self.imu_ready = True
            self.slam_ready = True
            self.movebase_ready = True
        
        try:
            self.powerThreashold = rospy.get_param(mapserver_param + "powerThreashold", 0)
        except Exception as e:
            self.powerThreashold = 0
        
        try:
            self.powerThreasholdMax = rospy.get_param(mapserver_param + "powerThreasholdMax", 90)
        except Exception as e:
            self.powerThreasholdMax = 0
        
        try:
            self.navi_retry_times_max = rospy.get_param(mapserver_param + "navi_retry_times_max", 60)
        except Exception as e:
            self.navi_retry_times_max = 60
        
        try:
            self.task_circle = rospy.get_param(mapserver_param + "task_circle", False)
        except Exception as e:
            self.task_circle = False

        try:
            self.task_executing_wait_time = rospy.get_param(mapserver_param + "task_executing_wait_time", 60)
        except Exception as e:
            self.task_executing_wait_time = 60

        # 开机检查，删除三天前的日志
        try:
            os.system("find /root/.ros/log -mtime +3 | xargs rm -rf")
        except Exception as e:
            rospy.loginfo("delete root logs failed!!!")

        try:
            os.system("find /home/spirit/.ros/log -mtime +3 | xargs rm -rf")
        except Exception as e:
            rospy.loginfo("delete spirit logs failed!!!")

        # 取消文件自检查
        if self.startSimulator == False:
            os.system("touch /fastboot")

        # 支持的服务接口
        self.TestDriver = rospy.ServiceProxy('test_driver_service', TestDriver)
        self.getPlanMoveBse = rospy.ServiceProxy('/get_plan_movebase', GetGPlan)        
        # cartographer接口
        self.finish_trajectory = rospy.ServiceProxy('finish_trajectory', FinishTrajectory)
        self.writestate = rospy.ServiceProxy('write_state', WriteState)

        # publish函数初始化
        self.laserswitch_pub = rospy.Publisher('/laser_switch', Int16, queue_size=5)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.motionmode_pub = rospy.Publisher('/motion_mode', Int16, queue_size=5)
        self.start_lidarcharge_pub = rospy.Publisher('/start_lidarcharge', PoseStamped, queue_size=5)
        self.androidcmd_pub = rospy.Publisher('/android_cmd', AndroidCmd, queue_size=5)

        self.PointsPose_Pub = rospy.Publisher('/points_pose', PointPoseArray, queue_size=30)
        self.initpose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.dockpose_pub = rospy.Publisher('dock_pose', ChargeInitPoint, queue_size=5)
        self.sendSpeed_pub = rospy.Publisher('/speed_dwa', Int16, queue_size=5)
        self.sonars_pub = rospy.Publisher('/sonars_set', RobotSonarEntrys, queue_size=5)
        self.navitestresults_pub = rospy.Publisher('/navitest_result', NaviTestResultEntrys, latch=True, queue_size=5)
        self.testodom_pub = rospy.Publisher('/testodom', Float32, queue_size=5)
        self.chargeStatus_pub = rospy.Publisher('/charge_status', Int16, queue_size=5)

        self.espload_pub = rospy.Publisher('/esp_load', Int16, latch=True, queue_size=5)
        self.RobotInformation_pub = rospy.Publisher('/receive_robot_information', RobotInformation, queue_size=5)

        self.imu_initxyz_pub = rospy.Publisher('/imu_init_xyz', gemoetryPoint, queue_size=5)        
        self.naviqueuelist_pub = rospy.Publisher('/navi_list_queue', NaviQueueList, latch=True, queue_size=5)
        self.navi_targetgoallist_pub = rospy.Publisher('/navi_target_goal_list', TargetGoalEntryRunList, queue_size=5)

        self.androidNavigationStatus_pub = rospy.Publisher('/androidmsg_navigationstatus', Int16, latch=False, queue_size=5)
        self.androidLocationStatus_pub = rospy.Publisher('/androidmsg_locationstatus', Int16, latch=True, queue_size=5)
        self.androidChargeStatus_pub = rospy.Publisher('/androidmsg_chargestatus', Int16, latch=True, queue_size=5)
        self.androidOutofChargepoint_pub = rospy.Publisher('/androidmsg_outofchargepoint', Int16, latch=True, queue_size=5)
        self.androidMotor1Status_pub = rospy.Publisher('/androidmsg_motor1status', Int16, latch=True, queue_size=5)
        self.androidMotor2Status_pub = rospy.Publisher('/androidmsg_motor2status', Int16, latch=True, queue_size=5)
        self.androidSonarStatus_pub = rospy.Publisher('/androidmsg_sonarstatus', Int16, latch=True, queue_size=5)
        self.androidLeftmotorStatus_pub = rospy.Publisher('/androidmsg_leftmotorstatus', Int16, latch=True, queue_size=5)
        self.androidRightmotorStatus_pub = rospy.Publisher('/androidmsg_rightmotorstatus', Int16, latch=True, queue_size=5)
        self.androidLidarStatus_pub = rospy.Publisher('/androidmsg_lidarstatus', Int16, latch=True, queue_size=5)
        self.androidImuStatus_pub = rospy.Publisher('/androidmsg_imustatus', Int16, latch=True, queue_size=5)
        self.androidCameraStatus_pub = rospy.Publisher('/androidmsg_camerastatus', Int16, latch=True, queue_size=5)
        self.androidBatteryStatus_pub = rospy.Publisher('/androidmsg_batterystatus', Int16, latch=True, queue_size=5)
        self.androidEmergencyStatus_pub = rospy.Publisher('/androidmsg_emergencystatus', Int16, latch=True, queue_size=5)
        self.androidMotorEnabled_pub = rospy.Publisher('/androidmsg_motorenabledstatus', Int16, latch=True, queue_size=5)
        self.androidMapStatus_pub = rospy.Publisher('/androidmsg_mapstatus', Int16, latch=True, queue_size=5)
        self.androidMapBuildStatus_pub = rospy.Publisher('/androidmsg_mapbuildstatus', Int16, latch=True, queue_size=5)
        self.androidVirtualmapStatus_pub = rospy.Publisher('/androidmsg_virtualmapstatus', Int16, latch=True, queue_size=5)
        self.androidStm32Status_pub = rospy.Publisher('/androidmsg_stm32status', Int16, latch=True, queue_size=5)
        self.androidInitStatus_pub = rospy.Publisher('/androidmsg_initstatus', Int16, latch=True, queue_size=5)
        self.androidOdomStatus_pub = rospy.Publisher('/androidmsg_odomstatus', Int16, latch=True, queue_size=5)
        self.androidNaviNetworkStatus_pub = rospy.Publisher('/androidmsg_navinetworkstatus', Int16, latch=True, queue_size=5)
        self.androidImportfilesStatus_pub = rospy.Publisher('/androidmsg_importfilesstatus', Int16, latch=True, queue_size=5)
        self.androidStartGoal_pub = rospy.Publisher('/android_startgoal', TargetGoalEntry, queue_size=5)

        self.PowerReportPub = rospy.Publisher('/power_report', Int16, latch=True, queue_size=5)
        self.heartbeat_pub = rospy.Publisher('/heart_beat', Int16, queue_size=5)
        self.mapStatus_pub = rospy.Publisher('/map_status', Int16, latch=True, queue_size=5)
        self.mapupdate_pub = rospy.Publisher('/map', OccupancyGrid, latch=True, queue_size=1)

        self.Errormsg_pub = rospy.Publisher('/Error_msg', String, queue_size=20)
        self.commitFile_pub = rospy.Publisher('/commit_file', CommitFile, queue_size=20)
        self.driver_config_pub = rospy.Publisher('config_update', Int32, queue_size=5)
        self.wall_update_pub = rospy.Publisher('/wall_update', Int32, latch=True, queue_size=1)
        self.sendToRecordbag_pub = rospy.Publisher('/send_to_record_bag', RecordBag, queue_size=1)

        self.appStatus_pub = rospy.Publisher('/to_app_status', Int16, latch=True, queue_size=1)
        self.locationstatus_pub = rospy.Publisher('/currentpose_locationmiss', Int16, latch=True, queue_size=1)
        self.neednavigation_pub = rospy.Publisher('/need_navigation', Int16, latch=True, queue_size=1)
        self.globalplans_path_pub = rospy.Publisher('/globalplans_path', PoseEntryList, queue_size=1)
        
        self.target_goal_list_pub  =rospy.Publisher('/target_goals', TargetGoalEntryList, queue_size=1) 

        self.go_target_op_pub = rospy.Publisher('/go_target_op', Int16, queue_size=1)

        self.robot_reboot_pub = rospy.Publisher('/robot_reboot', Int16, queue_size=5)   # 1 重启(屏幕也要重启) 2 重启(屏幕提示重启) 3 关机(屏幕也要关机) 4 关机(屏幕提示关机) 
        self.charge_point_pub = rospy.Publisher('/charge_point', Point32, latch=True, queue_size=1)

        self.current_floormap_pub = rospy.Publisher('/current_floormap', RobotFloorMap, latch=True, queue_size=10)

        self.elevator_status_req_pub = rospy.Publisher('/elevator/request_status', UInt16, queue_size=10)
        self.elevator_press_button_pub_ = rospy.Publisher('/elevator/press_button', UInt16MultiArray, queue_size=10)

        self.navi_stop_pub = rospy.Publisher("/navi_stop", Int16, queue_size=10)
        self.page_status_pub = rospy.Publisher('/pager/response_status', UInt16MultiArray, queue_size=10)    # 1 有车 2 无车 3 到点 4 取消
        self.task_exit_pub = rospy.Publisher('/task_exit', Int16, queue_size=10)            
        self.ledWarn_call_pub=rospy.Publisher('/ledwarn', UInt16MultiArray, queue_size=10)

        self.identify_and_lift_shelves_pub = rospy.Publisher('/identify_and_lift_shelves', PoseStamped, queue_size=10)
        self.arrive_and_place_shelves_pub = rospy.Publisher('/arrive_and_place_shelves', PoseStamped, queue_size=10)

        self.test_multi_robot_avoid_goal_pub = rospy.Publisher('/test_multi_robot_avoid_goal', PointStamped, queue_size=10)

        # Set up map management services
        self.chargingPriority_srv = rospy.Service('set_charpriority', SetInt64Array, self.set_charpriority)
        self.default_floorPoint_id_srv = rospy.Service('set_defaultfloor_pointid', SetInt64, self.set_defaultfloor_pointid)
        self.reset_wait_time_srv = rospy.Service('reset_wait_time', SetInt64, self.reset_wait_time)
        self.clear_map_srv = rospy.Service('clear_map', ClearMap, self.clear_map)
        self.rebuild_map_srv = rospy.Service('rebuild_map', ClearMap, self.rebuild_map)
        self.publish_map_srv = rospy.Service('publish_map', PublishMap, self.publish_map)
        self.delete_map_srv = rospy.Service('delete_map', DeleteMap, self.delete_map)
        self.delete_allmap_srv = rospy.Service('delete_allmap', DeleteMap, self.delete_allmap)
        self.rename_map_srv = rospy.Service('rename_map', RenameMap, self.rename_map)
        self.rename_floor_srv = rospy.Service('rename_floor', RenameMap, self.rename_floor)
        self.delete_floor_srv = rospy.Service('delete_floor', DeleteMap, self.delete_floor)
        self.save_map_srv = rospy.Service('save_map', SaveMap, self.save_map)
        self.saveerase_map_srv = rospy.Service('saveerase_map', SaveMap, self.saveerase_map)
        self.save_extmap_srv = rospy.Service('save_ext_map', SaveMapString, self.save_ext_map)
        self.robotspeed_set_srv = rospy.Service('robot_speedset', RobotSpeedSet, self.robotspeed_set)
        self.robotspeed_list_srv = rospy.Service('robot_speedlist', RobotSpeedList, self.robotspeed_list)
        self.navigation_safeset_srv = rospy.Service('navigation_safeset', NavigationSafeSet, self.navigationsafeset_set)
        self.navigationsafeset_list_srv = rospy.Service('navigation_safesetlist', NavigationSafeSetList, self.navigationsafeset_list)
        self.robotsonar_list_srv = rospy.Service('robot_sonarlist', RobotSonarList, self.robotsonar_list)
        self.robotsonar_set_srv = rospy.Service('robot_sonarset', RobotSonarSet, self.robotsonar_set)
        self.point_update_srv = rospy.Service('point_update', PointUpdate, self.point_update)
        self.point_set_srv = rospy.Service('point_set', PointSet, self.point_set) # 添加删除导航点
        self.point_rename_srv = rospy.Service('point_rename', PointRename, self.point_rename)
        self.list_test_points_srv = rospy.Service('list_test_points', ListTestPoints, self.list_test_points)
        self.list_navi_points_srv = rospy.Service('list_navi_points', ListNaviPoints, self.list_navi_points)
        self.delete_test_point_srv = rospy.Service('delete_test_point', DeleteTestPoint, self.delete_test_point)
        self.get_target_goal_plan_srv = rospy.Service('navi_targegoalplan', TargetGoalPlan, self.target_goal_plan)

        self.set_default_point_srv = rospy.Service('set_default_point', SetDefaultPoint, self.set_default_point)
        self.set_mode_srv = rospy.Service('set_mode', SetMode, self.set_mode)
        self.set_navi_cmd_srv = rospy.Service('set_navi_cmd', SetNaviCmd, self.set_navi_cmd)
        self.set_wifi_srv = rospy.Service('set_wifi', SetWifi, self.set_wifi)
        self.get_driverstatus_srv = rospy.Service('get_driver_status', ListDriverStatus, self.get_driver_status)
        self.set_coconum_srv = rospy.Service('set_coconum', SetCoco, self.set_coconum_cb)
        self.get_position_srv = rospy.Service('get_position', GetPosition, self.get_position)
        self.set_relocation_srv = rospy.Service('set_relocation', SetRelocation, self.set_relocation)
        self.pointmanu_set_srv = rospy.Service('pointmanu_set', PointManuSet, self.pointmanu_set) # 添加删除 带坐标的导航点
        self.get_lorastatus_srv = rospy.Service('get_lorastatus', SetInt64, self.get_lorastatus_cb)
        self.get_espstatus_srv = rospy.Service('get_espstatus', SetInt64, self.get_espstatus_cb)

        self.bag_list_srv = rospy.Service('bag_list', BagList, self.bag_list_cb)
        self.delete_allbags_srv = rospy.Service('delete_allbags', DeleteAllbags, self.delete_allbags_cb)
        self.delete_alllogs_srv = rospy.Service('delete_alllogs', DeleteAlllogs, self.delete_alllogs_cb)
        self.get_log_srv = rospy.Service('get_log', GetLog, self.get_log_cb)
        self.upload_server_srv = rospy.Service('upload_server', UploadServer, self.upload_server_cb)
        self.move_calibration_srv = rospy.Service('move_calibration', ScaleTest, self.move_calibration_cb)
        self.auto_upgrade_srv = rospy.Service('auto_upgrade', AutoUpgrade, self.auto_upgrade_cb)
        self.navi_info_srv = rospy.Service('navi_info', NaviInfo, self.navi_info_cb)
        self.robot_ssh_srv = rospy.Service('robot_ssh', RobotSsh, self.robot_ssh_cb)
        self.set_diypath_srv = rospy.Service('set_diypath', SetDiyPath, self.set_diypath_cb)
        self.get_configs_srv = rospy.Service('get_configs', GetConfigs, self.get_config_cb)
        self.set_config_srv = rospy.Service('set_configs', SetOrDelConfigs, self.set_config_cb)
        self.del_config_srv = rospy.Service('del_configs', SetOrDelConfigs, self.del_config_cb)
        self.get_map_srv = rospy.Service('get_maps', Trigger, self.get_maps_cb)
        self.list_androidalarm_report_srv = rospy.Service('list_androidmsg_report', AndroidAlarmReport, self.list_androidalarm_report_cb)
        self.set_virtualwall_srv = rospy.Service('set_shape', SetShape, self.set_virtualwall_cb)
        self.set_region_srv = rospy.Service('set_region', SetRegion, self.set_region_cb)
        self.set_cleanArea_srv = rospy.Service('set_cleanArea', SetCleanArea, self.set_cleanArea_cb)
        self.import_config_srv = rospy.Service('import_configs', ImportConfigs, self.import_config_cb)

        self.addpoints_globalplan_srv = rospy.Service('addpoints_globalplan', AddpointsGlobalplan, self.addpoints_globalplan_cb)
        self.deletepoints_globalplan_srv = rospy.Service('deletepoints_globalplan', AddpointsGlobalplan, self.deletepoints_globalplan_cb)
        self.rename_globalplan_srv = rospy.Service('rename_globalplan', RenameGlobalplan, self.rename_globalplan_cb)
        self.set_erasermode_srv = rospy.Service('set_erasermode', SetEraserMode, self.set_eraser_cb)

        # 上报 wifi到Android  self.wifilist
        self.update_wifi_pub = rospy.Publisher('/update_wifi',WifiArray,latch=True, queue_size=30)

        fo = open(robot_driver_param_url)
        # d = yaml.load(fo, Loader=yaml.FullLoader)
        d = yaml.load(fo)
        if (d['useSonar'] == True):
           self.use_Sonar = True
           rospy.loginfo("useSonar is set True")
        else:
           self.use_Sonar = False
           rospy.loginfo("useSonar is set False")
        # 关闭文件
        fo.close()

        # subscribe函数初始化
        # 导航相关
        rospy.Subscriber("/add_target_goals", TargetGoalEntryList, self.add_target_goal_CB)
        rospy.Subscriber("/del_target_goal", TargetGoalEntryDel, self.del_target_goal_CB)
        rospy.Subscriber("/navi_targetgoal", TargetGoal, self.TargetGoal_CB)
        rospy.Subscriber("/navi_multi_targetgoal", NewTargetGoalList, self.naviMultiTargetGoal_CB)

        rospy.Subscriber("/set_types_target_goal", TargetGoalType, self.set_types_target_goal_CB) # 设置 运行状态 是否循环和等待时间
        rospy.Subscriber("/add_wait_time_target_goal", Int32, self.add_wait_time_target_goal_CB) # 对当前任务临时追击加延长等待

        rospy.Subscriber("/navigation_status", NavigationStatusResult, self.navigationStatusCallback)
        rospy.Subscriber("/location_status", Int16, self.LocationStatusCallback)

        rospy.Subscriber("/path_validate", SaveGlobalPlan, self.PathValidateCallback)
        rospy.Subscriber("/npath_addorDel", NGlobalPlan, self.NPathAddOrDelCallback)

        rospy.Subscriber("/relocation_pose", RelocationPose, self.RelocationPoseCallback)

        rospy.Subscriber("/Record_bag", RecordBag, self.RecordBag_CB)
        rospy.Subscriber("/send_msgreport", Int16, self.SendMsgreport_CB)
        rospy.Subscriber("/movebase_msgreport", Int16, self.MovebaseMsgreport_CB)
        rospy.Subscriber("/virtualwall_msgreport", Int16, self.VirtualwallMsgreport_CB)
        rospy.Subscriber("/erase_pose", ErasePose, self.ErasePose_CB)
        rospy.Subscriber("/move_base_ready", Int16, self.MovebaseReady_CB)
        rospy.Subscriber("/tracked_pose", PoseStamped, self.TrackedPose_CB)

        rospy.Subscriber("/navi_cmd_vel", Twist, self.navicmd_CB)
        rospy.Subscriber("/navi_stop", Int16, self.NaviStop_CB)

        rospy.Subscriber("/test_multi_robot_avoid", RobotInformation, self.test_multrobot_avoid)
        rospy.Subscriber("/test_multi_robot_avoid1", RobotInformation, self.test_multrobot_avoid1)

        # 传感器
        rospy.Subscriber("/pager/call_status", UInt16MultiArray, self.loraClientReceived_CB)
        rospy.Subscriber("/elevator/current_status", UInt16MultiArray, self.elevatorStatus_CB)

        rospy.Subscriber("/sonar_actual", Float32MultiArray, self.Sonar_acturalCallback)
        rospy.Subscriber("/imu_initset", Int16, self.ImuInitSetCallback)
        rospy.Subscriber("/scan", LaserScan, self.scan_CB)
        rospy.Subscriber("/raw_odom", Odometry, self.odom_CB)
        rospy.Subscriber("/localmap", OccupancyGrid, self.localmap_CB)
        rospy.Subscriber("/map", OccupancyGrid, self.map_CB)
        rospy.Subscriber("/imu/data", Imu, self.imu_CB)
        rospy.Subscriber("/imu_angle", gemoetryPoint, self.imu_angle_CB)
       
        # 驱动         
        rospy.Subscriber("/driver_info", DriverEntry, self.DriverinfoCallback)        
        rospy.Subscriber("/network_beat", Int16, self.NetworkbeatCallback)        

        # 顶升相关  
        rospy.Subscriber("/current_jacking_status", Int16, self.CurrentJackingStatuslCallback)
        
        # 充电桩
        rospy.Subscriber("/chargepoint_set", PoseStamped, self.chargepoint_set)
        rospy.Subscriber("/charge_fail", Int16, self.charge_fail)
        rospy.Subscriber("/autocharge", Int16, self.AutoCharge_CB)

        # 配置        
        rospy.Subscriber("/apptorobot", Int16, self.AppToRobot_CB)
        rospy.Subscriber("/update_status", Int16, self.Updatestatus_CB)  
        rospy.Subscriber("/reset_params", Int16, self.resetParams_CB)

        if self.startScan or self.slaveMachine:
            try:
                self.tf_listener.waitForTransform(self.base_footprint_frame, self.scan_frame, rospy.Time(), rospy.Duration(3.0))
                (scan_position, scan_rot) = self.tf_listener.lookupTransform(self.base_footprint_frame, self.scan_frame, rospy.Time(0))
                scan_pose = Pose()
                scan_pose.position = gemoetryPoint(*scan_position)
                scan_pose.orientation = Quaternion(*scan_rot)
                self.scan_pose_pub.publish(scan_pose)
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.logerr("Map manager : initialize scan to base_footprint frame fail, TF can't create.")

        # ping the gate way
        if self.startSimulator == False:
            os.system("ping -c 1 10.7.5.1")

        # 启动多机避障
        self.loading_multirobotsAvoid()

        # 启动双目摄像头
        self.loading_camera()

        # 启动ODO驱动
        self.loading_driver()

        # 启动waypoint
        self.loading_waypoint()

        # 启动bag
        self.loading_bag()

        # 启动backend
        self.loading_backend()

        # 启动rosbridge
        self.loading_rosbridge()
        
        # 启动autoupgrade
        self.loading_autoupgrade()        
        
        if self.startSimulator:
            self.simulator_init = 0
            while not rospy.is_shutdown():
                if (self.simulator_init == 5):
                    self.check_map()
                    self.map_status = 0
                if (self.simulator_init > 6):
                    self.simulator_init  = 6
                self.simulator_init = self.simulator_init+1
                time.sleep(1.0)

        while not rospy.is_shutdown():
            #rospy.loginfo("主循环开始, map_loading: %d", self.map_loading)
            if self.update_status == 0:   
                # 广播一下机器的状态给上面安卓，可以页面跳转
                self.androidNavigationStatus_pub.publish(self.AndroidAlarm.androidmsg_navigationstatus)
                
                if self.save_position_flag == 1:                
                    self.save_position(1)   #保存当前位置                    
                    self.save_position_flag = 0
                elif self.save_position_flag == 2:                    
                    self.save_position(2)     # 保存电梯位置
                    self.save_position_flag = 0
                elif self.save_position_flag == 3:                    
                    self.save_position(3)   # 清除为0位置
                    self.save_position_flag = 0
                
                # 调度导航点任务 ----------------------------------------------------------

                # 任务结果判定
                if (self.is_across_loadingMap and self.movebase_ready):
                    # 切换地图成功
                    if (self.task_executing == 0):
                        self.task_executing = 1
                    self.get_nexttask = True
                    self.task_runs_status = False
                    self.is_across_loadingMap = False
                
                if (self.restartNavi and self.movebase_ready):
                    # 检测到movebase挂死重启后 这里开启一次检查导航队列
                    if (self.task_executing == 0):
                        self.task_executing = 1
                    self.get_nexttask = True
                    self.task_runs_status = False
                    self.restartNavi = False
                
                if self.start_chargecheck and self.map_driverinfo.ChargeStatus > 0 and self.map_driverinfo.ChargeStatus < 5:
                    # 充电成功
                    rospy.loginfo("充电成功，删除队列中的充电点")
                    self.start_chargecheck = False
                    self.target_goal_lock.acquire()
                    if self.low_targetgoals and self.low_targetgoals[0].status == 1:
                        self.low_targetgoals[0].status = 2
                        self.operated_targetGoals.append(self.low_targetgoals[0])
                        self.low_targetgoals.pop(0)
                    self.target_goal_lock.release()
                    self.get_nexttask = True
                    self.task_runs_status = False
                    self.AndroidAlarm.androidmsg_navigationstatus = 86  #对桩成功
                    self.task_current_wait_time = self.task_executing_wait_time

                if self.checkTargetNaviFail_count <= 60 and self.checkTargetNaviFail:
                    self.checkTargetNaviFail = False
                    if self.subtask_targetgoals or self.high_targetgoals or self.low_targetgoals:
                        rospy.loginfo("split_targetGoal 分析路线失败，retry, %d", self.checkTargetNaviFail_count)
                        if (self.task_executing == 0):
                            self.task_executing = 1
                        self.get_nexttask = True
                        self.task_runs_status = False
                
                if self.navi_retry_times > self.navi_retry_times_max:
                    self.navi_retry_times = 0
                    rospy.loginfo("navigation fail retry more than navi_retry_times_max, stop....")
                    self.task_stop()

                if self.map_driverinfo.Power > self.powerThreasholdMax and self.powerThreasholdMax > 0:
                    if self.lowpowerExecute == True:
                        self.lowpowerExecute = False
                        if self.task_executing == 0:
                            self.task_executing = 1
                        self.get_nexttask = True
                        self.task_runs_status = False
                        rospy.logwarn("电量足够 准备恢复队列任务 power: %d powerThreasholdMax %d", self.map_driverinfo.Power, self.powerThreasholdMax)

                if self.map_driverinfo.ChargeStatus > 0 and self.map_driverinfo.ChargeStatus < 5 and self.lowpowerChargepoint_add:
                    self.lowpowerExecute = True
                    self.lowpowerChargepoint_add = False
                    self.task_runs_status = False

                if self.powerThreashold > 0 and self.map_driverinfo.Power <= self.powerThreashold: 
                    if self.map_driverinfo.ChargeStatus > 0 and self.map_driverinfo.ChargeStatus < 5:
                        self.lowpowerChargepoint_add = False
                    else:
                        if self.lowpowerChargepoint_add == False and self.lowpowerExecute == False:
                            rospy.logwarn("电量低于门限值，增加低电回充任务")                    
                            self.lowpowerChargepoint_add = True
                            (isfind, chtg) = self.findChargeTargetGoalEntry() 
                            if isfind: # 跨楼层跨区域充电
                                tg2_change =copy.deepcopy(chtg)
                                # 低电回充任务放到队列末尾
                                self.target_goal_lock.acquire()
                                self.low_targetgoals.append(tg2_change)
                                self.target_goal_lock.release()
                                if self.task_executing == 0:
                                    self.task_executing = 1
                                    self.get_nexttask = True
                                    self.task_runs_status = False

                # 检查任务队列，如果有任务，符合执行条件，但没有执行，执行
                if (self.task_executing !=0 and self.lowpowerExecute == False):
                    if self.task_runs_status == False:
                        self.get_nexttask = True
                
                if self.init_status == 1:
                    if self.lowpowerExecute == False:
                        if (self.task_executing != 0 and self.get_nexttask):
                            # 有任务在执行
                            self.get_nexttask = False
                            if self.task_circle and not self.subtask_targetgoals and not self.high_targetgoals and not self.low_targetgoals:
                                #循环执行 
                                if (len(self.high_targetgoals_backup)+ len(self.low_targetgoals_backup)) > 1: # 一个任务不能循环
                                    rospy.loginfo(" task circle again: %d",len(self.low_targetgoals_backup))
                                    self.target_goal_lock.acquire()
                                    self.high_targetgoals = copy.deepcopy(self.high_targetgoals_backup)
                                    self.low_targetgoals = copy.deepcopy(self.low_targetgoals_backup)
                                    self.operated_targetGoals = []
                                    self.target_goal_lock.release()
                                    self.circle_times = self.circle_times + 1
                                else:
                                    rospy.loginfo(" task circle fail!")

                            if self.subtask_targetgoals:
                                # 取出一个子任务新导航任务执行
                                self.target_goal_lock.acquire()
                                sub_goal = self.subtask_targetgoals.pop(0)
                                self.target_goal_lock.release()
                                if (sub_goal.type == 25):
                                    #开始电梯切换地图
                                    self.change_map_flag = True
                                    self.AndroidAlarm.androidmsg_navigationstatus = 71    # 切换地图
                                    self.changeTargetMap(1)   
                                    # 及时保存一下当前位置
                                    self.save_position(2)     # 保存电梯位置
                                    rospy.loginfo("开始切换地图...")  

                                if (sub_goal.type == 26):
                                    #开始充电重定位切换地图
                                    self.charge_relocation_change_map_flag = True
                                    self.changeTargetMap(2)   
                                    rospy.loginfo("充电中，开始切换地图...")  
                                
                                if (sub_goal.type == 24):
                                    #开始对桩
                                    self.start_chargecheck = True
                                    self.AndroidAlarm.androidmsg_navigationstatus = 85    # 开始对桩
                                    self.addChargepointTargetGoalEntry()   
                                    rospy.loginfo("开始进行充电桩对桩...")  
                            
                                if (sub_goal.type == 22):
                                    #开始顶升
                                    self.start_liftcheck = True
                                    self.AndroidAlarm.androidmsg_navigationstatus = 88    # 开始顶升
                                    self.addLiftTargetGoalEntry(sub_goal)   
                                    rospy.loginfo("开始进顶升任务...")  

                                if (sub_goal.type == 23):
                                    #开始卸货
                                    self.start_putdowncheck = True
                                    self.AndroidAlarm.androidmsg_navigationstatus = 91    # 开始卸货
                                    self.addPutdownTargetGoalEntry()   
                                    rospy.loginfo("开始进行卸货任务...")  
                                self.task_runs_status = True
                        
                                rospy.loginfo("Get a subtask goal: %d, %d, %d, type: %d", self.across_elevator_entry.floor_id, self.across_elevator_entry.map_id, self.across_elevator_entry.point_id, sub_goal.type)

                            elif self.high_targetgoals:
                                # 取出一个高优先级新导航任务执行
                                #self.locationstatus_pub.publish(10)
                                #rospy.loginfo("机器人开启新任务，进行局部定位！！！！")

                                self.target_goal_lock.acquire()
                                self.high_targetgoals[0].status = 1
                                target_goal = copy.deepcopy(self.high_targetgoals[0])
                                self.target_goal = target_goal
                                self.target_goal_lock.release()

                                # 广播给安卓
                                #if self.checkTargetNaviFail_count == 0 and self.change_map_flag == False:
                                #    self.androidStartGoal_pub.publish(target_goal)

                                rospy.loginfo("Get a hight goal: %d, %d, %d, type: %d", self.target_goal.floor_id, self.target_goal.map_id, self.target_goal.point_id, self.target_goal.type)
                                (isnavi,distance) = self.split_targetGoal(0) 
                                if isnavi==False: # isnavi 是否导航检查成功
                                    if self.first_targetjump == True:
                                        self.androidStartGoal_pub.publish(self.target_goal)  
                                        self.first_targetjump = False
                                        self.AndroidAlarm.androidmsg_navigationstatus = 1
                                    if self.checkTargetNaviFail_count < 60:
                                        self.checkTargetNaviFail = True
                                        self.checkTargetNaviFail_count = self.checkTargetNaviFail_count + 1
                                        if self.AndroidAlarm.androidmsg_navigationstatus !=8:
                                            self.AndroidAlarm.androidmsg_navigationstatus = 8   
                                    else:
                                        rospy.loginfo("high_targetgoals split_targetGoal retry times more than 60, stop current target, getnext...")
                                        self.target_goal_lock.acquire()
                                        if self.high_targetgoals: # 移除h，并且放入运行记录为失败
                                            self.high_targetgoals[0].status = 10
                                            self.operated_targetGoals.append(self.high_targetgoals[0])
                                            self.high_targetgoals.pop(0)
                                            self.get_nexttask = True
                                            self.task_runs_status = False
                                            self.waitfornavi = False
                                            self.start_liftcheck = False
                                            self.start_putdowncheck = False
                                            self.AndroidAlarm.androidmsg_navigationstatus = 3    # 导航失败                                        
                                        self.target_goal_lock.release()
                                        self.checkTargetNaviFail_count = 0
                                        self.navi_retry_times = self.navi_retry_times + 1
                                else:
                                    self.checkTargetNaviFail_count = 0
                                self.task_runs_status = True

                            elif self.low_targetgoals:
                                if self.task_current_wait_time >= 0:
                                    self.task_current_wait_time = self.task_current_wait_time-1
                                    rospy.loginfo("task wait, task_current_wait_time: %d", self.task_current_wait_time)
                                    self.get_nexttask = True
                                    self.task_runs_status = False
                                else:                            
                                    # 取出一个低优先级新导航任务执行
                                    #self.locationstatus_pub.publish(10)
                                    #rospy.loginfo("机器人开启新任务，进行局部定位！！！！")

                                    self.target_goal_lock.acquire()
                                    self.low_targetgoals[0].status = 1
                                    target_goal = copy.deepcopy(self.low_targetgoals[0]) # 避免修改数据导致 可变对象跟随改变，这里用深度备份
                                    self.target_goal = target_goal
                                    self.target_goal_lock.release()

                                    # 广播给安卓
                                    #if self.checkTargetNaviFail_count == 0 and self.change_map_flag == False:
                                    #    self.androidStartGoal_pub.publish(target_goal)

                                    if target_goal.type == 24:
                                        # 是充电桩导航，自动充电
                                        self.AndroidAlarm.androidmsg_navigationstatus = 71    # 开始自动充电
                                        self.waitfornavi = True
                                        self.start_chargecheck = False
                                    else:
                                        self.waitfornavi = False
                                    
                                    rospy.loginfo("Get a low goal: %d, %d, %d, type: %d", self.target_goal.floor_id, self.target_goal.map_id, self.target_goal.point_id, self.target_goal.type)

                                    (isnavi,distance) = self.split_targetGoal(0) 
                                    rospy.loginfo("isnavi: %d, Fail_count: %d",isnavi,self.checkTargetNaviFail_count)
                                    if isnavi==False: # isnavi 是否导航检查成功
                                        if self.first_targetjump == True:
                                            self.androidStartGoal_pub.publish(self.target_goal)  
                                            self.first_targetjump = False
                                            self.AndroidAlarm.androidmsg_navigationstatus = 1

                                        if self.checkTargetNaviFail_count < 60:
                                            self.checkTargetNaviFail = True
                                            self.checkTargetNaviFail_count = self.checkTargetNaviFail_count + 1
                                            if self.AndroidAlarm.androidmsg_navigationstatus !=8:
                                                self.AndroidAlarm.androidmsg_navigationstatus = 8    
                                        else:
                                            # 导航任务超时，去除当前导航点，进入下一个点导航
                                            rospy.loginfo("low_targetgoals split_targetGoal retry times more than 60, stop current target, getnext...")
                                            self.target_goal_lock.acquire()
                                            if self.low_targetgoals: # 移除l，并且放入运行记录为失败
                                                self.low_targetgoals[0].status = 10
                                                self.operated_targetGoals.append(self.low_targetgoals[0])
                                                self.low_targetgoals.pop(0)
                                                self.get_nexttask = True
                                                self.task_runs_status = False
                                                self.task_current_wait_time = self.task_executing_wait_time
                                                self.waitfornavi = False
                                                self.start_liftcheck = False
                                                self.start_putdowncheck = False
                                                self.AndroidAlarm.androidmsg_navigationstatus = 3    # 导航失败

                                            self.target_goal_lock.release()
                                            self.checkTargetNaviFail_count = 0
                                            self.navi_retry_times = self.navi_retry_times + 1
                                    else:
                                        self.checkTargetNaviFail_count = 0
                                self.task_runs_status = True
                                
                            else:
                                self.task_runs_status = False
                                if self.task_executing != 0:
                                    # 所有task执行完毕
                                    rospy.loginfo("all task has executed to finished!!!")
                                    self.task_executing = 0
                                if self.need_gotodefaultPoint:
                                    self.need_gotodefaultPoint = False
                                    # 需要去待命区
                                    self.add_default_point()

                self.update_navi_target_lists_pub() 
                # 调度导航点任务结束 ---------------------------------------------------------
                    
                if self.map_loading:
                    self.init_status = 0
                    init_flag = 0
                    self.map_loading = False
                    rospy.logwarn("start main thread load map!")
                    self.map_ready = False
                    self.localmap_ready = False
                    self.cartographer_status = -1
                    if self.map_status == 0:
                        if self.navigation_safeset == 1 or self.navigation_safeset == 2:
                            self.need_navigation = False
                            self.neednavigation_pub.publish(0)
                        else:
                            self.neednavigation_pub.publish(1)

                        #开机重定位用，延迟6秒重定位
                        self.relocation_flag = 6
                        rospy.loginfo("开始cartographer加载地图!")
                        self.loading_cartographer_loadMap(self.map_file)
                        self.slam_ready = True
                        rospy.loginfo("Robot slam is ready!")

                        #self.loading_mapserver_loadMap(self.map_file)

                        self.shutdown_movebase()# 首次加载地图 或者 切换地图
                        start_movebase_timeout = 0
                        self.loading_movebase() # 首次加载地图 或者 切换地图

                        self.check_navigation_isOK()
                    if self.map_status == 1:
                        self.fix_chargepoint_flag = False
                        self.shutdown_movebase() # 开始cartographer新建地图
                        rospy.loginfo("开始cartographer新建地图!")
                        self.loading_cartographer_buildMap()
                        self.slam_ready = True
                        rospy.loginfo("Robot slam is ready!")
                        self.check_navigation_isOK()
                    if self.map_status == 2:
                        self.fix_chargepoint_flag = False
                        self.shutdown_movebase() # 开始cartographer扩展地图
                        rospy.loginfo("开始cartographer扩展地图!")
                        self.loading_cartographer_rebuildMap(self.map_file)
                        self.slam_ready = True
                        rospy.loginfo("Robot slam is ready!")
                        self.check_navigation_isOK()

                if self.cartographer_status >= 60 and self.startDebug == False:
                    # cartographer 僵死，重启cartographer，workaround
                    self.cartographer_status = 1
                    if self.map_status == 0:
                        #开机重定位用，延迟6秒重定位
                        self.relocation_flag = 6
                        rospy.loginfo("开始cartographer加载地图！")
                        self.loading_cartographer_loadMap(self.map_file)
                        rospy.loginfo("cartographer挂死，重启cartographer成功!")

                    if self.map_status == 1:
                        rospy.loginfo("开始cartographer新建地图！")
                        self.loading_cartographer_buildMap()
                        rospy.loginfo("cartographer挂死，重启cartographer成功!")

                    if self.map_status == 2:
                        rospy.loginfo("开始cartographer扩展地图！")
                        self.loading_cartographer_rebuildMap(self.map_file)
                        rospy.loginfo("cartographer挂死，重启cartographer成功!")


                if (init_flag > 100):
                    # 100s system is not OK
                    if (self.init_status != 1):
                        if (self.AndroidAlarm.androidmsg_initstatus !=11):
                            self.androidInitStatus_pub.publish(11)
                            #self.AndroidmsgReportPub.publish(11)
                            self.AndroidAlarm.androidmsg_initstatus = 11
                            self.list_driverstatus_srv = rospy.Service('list_driver_status', ListDriverStatus, self.list_driver_status)
                            rospy.loginfo("机器人导航初始化失败，导航模块不工作！odom = %s , scan = %s , map = %s , movebase = %s , slam = %s , init = %s",
                                          self.odom_ready,self.scan_ready,self.map_ready,self.movebase_ready,self.slam_ready,self.init_status)
                else:
                    init_flag = init_flag + 1

                if self.map_status == 0:
                    if (self.movebase_heart == 0 and self.movebase_ready and self.startDebug == False):
                        # Movebase 挂了，重启movebase
                        if (self.movebae_timeout > 5):
                            self.movebae_timeout = 0
                            
                            rospy.logwarn("长时间没有movebase 心跳 认为 Movebase死机了!!!!")
                            self.shutdown_movebase()# 长时间没有movebase 心跳
                            rospy.logwarn("重新启动Movebase中")
                            self.loading_movebase()
                            self.restartNavi = True
                            start_movebase_timeout = 0
                        else:
                            self.movebae_timeout = self.movebae_timeout + 1

                    if self.movebase_heart  == 1:
                        self.movebase_heart = 0
                        self.movebae_timeout = 0

                    if start_movebase_timeout == 5:
                        #rospy.loginfo("start_movebase_timeout: %d, 启动movebaseAction", start_movebase_timeout)
                        start_movebase_timeout = start_movebase_timeout + 1
                    elif start_movebase_timeout < 5:
                        start_movebase_timeout = start_movebase_timeout + 1
                elif self.map_status == 1 or self.map_status == 2:
                    self.movebase_ready = True


                if (self.map_driverinfo.ChargeStatus > 0 and self.map_driverinfo.ChargeStatus <5 and self.relocation_flag > 0):
                    if self.relocation_flag == 1:
                        self.relocation_flag = 0
                        rospy.loginfo("机器人在充电中，开机重定位一次！！！")
                        self.relocation_set(1)
                    self.relocation_flag = self.relocation_flag -1
                else:
                    self.relocation_flag = 0                

                self.cartographer_status = self.cartographer_status +1
                if self.cartographer_status >60:
                    self.cartographer_status = 60
                
                if (self.shutdown == 1):
                    rospy.loginfo("机器人收到底盘关机消息，机器人开始关机！！！")
                    if not self.slaveMachine:
                        os.system("poweroff")
                    else:
                        rospy.signal_shutdown("mapserver_manager")

                elif (self.shutdown == 2):
                    rospy.loginfo("机器人收到底盘重启消息，机器人开始重启！！！")
                    if not self.slaveMachine:
                        os.system("reboot")
                    else:
                        rospy.signal_shutdown("mapserver_manager")

            if self.imu_status >= 20 and self.init_status == 1 and self.startImu == True and self.startDebug == False:
                # imu 僵死，重启imu，workaround
                self.imu_status = 0
                rospy.loginfo("开始imu重启！")
                self.shutdown_imu()
                self.loading_imu()
                rospy.loginfo("imu挂死，重启imu成功!")
            
            if self.scan_status >= 20 and self.init_status == 1 and self.startScan == True and self.startDebug == False and self.lidar_owner.mode() == 'factory':
                # scan 僵死，重启scan，workaround
                self.scan_status = 0
                rospy.loginfo("开始scan重启！")
                self.shutdown_scan()
                self.loading_scan()
                rospy.loginfo("imu挂死，重启scan成功!")
            
            if self.driver_status >= 60 and self.init_status == 1 and self.startDebug == False:
                # odom驱动僵死，重启odom驱动，workaround
                self.driver_status = 0
                rospy.loginfo("开始ODOM驱动重启！")
                self.shutdown_driver()
                self.loading_driver()
                rospy.loginfo("odom驱动挂死，重启ODOM驱动成功!")

            self.imu_status = self.imu_status + 1
            if self.imu_status > 20:
                self.imu_status = 20
            
            self.driver_status = self.driver_status + 1
            if self.driver_status > 60:
                self.driver_status = 60

            self.scan_status = self.scan_status + 1
            if self.scan_status > 20:
                self.scan_status = 20
            time.sleep(1.0)

        rospy.loginfo("start exit")
        try:
            self.shutdown_movebase() # 回收程序
            self.shutdown_camera()            
            self.shutdown_multirobotsAvoid()
        except:
            rospy.loginfo("尝试停止movebase出错")
        try:
            self.shutdown_imu()
        except:
            rospy.loginfo("尝试停止imu出错")
        try:
            self.shutdown_driver()
        except:
            rospy.loginfo("尝试停止odom driver出错")
        try:
            if self.carto_build_launch_flag:
                rospy.loginfo("停止cartographer建图")
                self.carto_build_launch.shutdown()
                time.sleep(3)
            self.carto_build_launch_flag = False
        except:
            rospy.loginfo("尝试停止cartographer建图出错！")

        try:
            if self.carto_local_launch_flag:
                rospy.loginfo("停止cartographer导航定位")
                self.carto_local_launch.shutdown()
                time.sleep(3)
            self.carto_local_launch_flag = False
        except:
            rospy.loginfo("尝试停止cartographer导航定位出错！")

        try:
            if self.carto_rebuild_launch_flag:
                rospy.loginfo("停止cartographer扩展建图")
                self.carto_rebuild_launch.shutdown()
                time.sleep(3)
            self.carto_rebuild_launch_flag = False
        except:
            rospy.loginfo("尝试停止cartographer扩展建图出错！")
        
        try:
            self.shutdown_scan()
        except:
            rospy.loginfo("尝试停止scan出错")
        
        try:
            self.shutdown_rosbridge()
        except:
            rospy.loginfo("尝试停止rosbridge出错")
        try:
            self.shutdown_waypoint()
        except:
            rospy.loginfo("尝试停止waypoint出错")
        try:
            self.shutdown_bag()
        except:
            rospy.loginfo("尝试停止bag出错")
        try:
            self.shutdown_backend()
        except:
            rospy.loginfo("尝试停止backend出错")
        try:
            self.shutdown_lora()
        except:
            rospy.loginfo("尝试停止lora出错")
        try:
            self.shutdown_ekf()
        except:
            rospy.loginfo("尝试停止ekf出错")
   
    def go_target_goalList(self, tago):#tago:TargetGoalEntryList
        
        # tago.goal_point_list = []
        # self.target_goal_list_pub.publish(tago)
        return

    def task_stop(self, isclearNaviQueue = True):
        rospy.loginfo("enter cancel goal!!")

        #self.ex_line_navition(4)
        if self.lora_call_id != 0:
            lora_page = UInt16MultiArray()
            lora_page.data.append(self.lora_call_id)
            lora_page.data.append(4) # 其他停止
            self.page_status_pub.publish(lora_page)
            self.lora_call_id = 0

        # 停止后 恢复字段，避免任务结束不掉
        self.target_goal.floor_id = 0
        self.target_goal.map_id = 0
        self.target_goal.point_id = 0

        self.navi_retry_times = 0
        self.need_gotodefaultPoint = False
        self.AndroidAlarm.androidmsg_navigationstatus = 83    #导航取消

        self.use_kua = False 
        if isclearNaviQueue:
            self.task_executing = 0
            self.task_runs_status = False
            self.lowpowerExecute = False
            self.circle_times = 0
            self.lowpowerChargepoint_add = False
            self.task_circle = False
            self.task_current_wait_time = -1
            self.get_nexttask = False
            self.waitfornavi = False
            self.start_chargecheck = False
            self.start_liftcheck = False
            self.start_putdowncheck = False
            self.subtask_targetgoals = []
            self.high_targetgoals = []
            self.low_targetgoals = []
            self.high_targetgoals_backup = []
            self.low_targetgoals_backup = []
            self.operated_targetGoals = []

        try:
            # 1 暂停 2 继续 3 停止
            self.go_target_op_pub.publish(3)
        except Exception as e:
            rospy.loginfo("cancel error!!!!!")

    def save_position(self, isClear):
        # 这个会检查文件及其路径是否存在，否则自动创建路径和空文件
        # isClear: 1：记录当前位置 2. 记录电梯位置，3 复位为0点位置 
        position_path = os.path.dirname(os.path.realpath(robot_cartopose_param_url))
        POSITION_PATH = os.path.join(os.path.dirname(position_path), 'carto')
        if not os.path.exists(POSITION_PATH):
            os.mkdir(POSITION_PATH)
        positionfile = os.path.join(POSITION_PATH, 'carto_config.yaml')
        if not os.path.exists(positionfile):
            os.system(r"touch {}".format(positionfile))

        if isClear == 1 and self.cartographer_status >=0:
            #rospy.loginfo("保存当前位置到carto.yaml文件!")
            if self.change_map_flag:
                rospy.loginfo("当前正处于电梯切换地图模式，保存当前位置到carto.yaml文件无效!")
                return
            
            try:
                #(position, rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
                carto_position_setting = dict()
                carto_position_setting.clear()
                carto_position_setting["set_inital_pose_x"] = self.current_pose.pose.position.x
                carto_position_setting["set_inital_pose_y"] = self.current_pose.pose.position.y
                carto_position_setting["set_inital_pose_z"] = self.current_pose.pose.position.z

                carto_position_setting["set_inital_pose_ox"] = self.current_pose.pose.orientation.x
                carto_position_setting["set_inital_pose_oy"] = self.current_pose.pose.orientation.y
                carto_position_setting["set_inital_pose_oz"] = self.current_pose.pose.orientation.z
                carto_position_setting["set_inital_pose_ow"] = self.current_pose.pose.orientation.w

                self.writ_patam(robot_cartopose_param_url, carto_position_setting)
                #rospy.loginfo("%f, %f, %f- %f,%f,%f,%f, currentpose: %f,%f,%f-%f,%f,%f,%f", position[0], position[1], position[2], rot[0], rot[1], rot[2], rot[3], self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z,
                #              self.current_pose.pose.orientation.x, self.current_pose.pose.orientation.y, self.current_pose.pose.orientation.z, self.current_pose.pose.orientation.w)
            except:
                rospy.loginfo("save_position出错！")

        elif isClear == 2:
            try:
                carto_position_setting = dict()
                carto_position_setting.clear()
                carto_position_setting["set_inital_pose_x"] = float(self.across_elevator_point.getPositionX())
                carto_position_setting["set_inital_pose_y"] = float(self.across_elevator_point.getPositionY())
                carto_position_setting["set_inital_pose_z"] = 0.0

                rot = transformations.quaternion_from_euler(0.0, 0.0, float(self.across_elevator_point.getPositionYaw()) / 180.0 * pi)
                
                carto_position_setting["set_inital_pose_ox"] = float(rot[0])
                carto_position_setting["set_inital_pose_oy"] = float(rot[1])
                carto_position_setting["set_inital_pose_oz"] = float(rot[2])
                carto_position_setting["set_inital_pose_ow"] = float(rot[3])
                self.writ_patam(robot_cartopose_param_url, carto_position_setting)
                rospy.loginfo("save_position2 write success self.across_elevator_point position(%f,%f) yaw=%f ",self.across_elevator_point.getPositionX(),self.across_elevator_point.getPositionY(),self.across_elevator_point.getPositionYaw())
            except:
                rospy.loginfo("2， save_position出错！")
        elif isClear == 3:
            try:
                rospy.loginfo("重置初始化位置到carto.yaml文件!")
                if self.change_map_flag:
                    rospy.loginfo("当前正处于电梯切换地图模式，清除当前位置到carto.yaml文件无效!")
                    return
                carto_position_setting = dict()
                carto_position_setting.clear()
                carto_position_setting["set_inital_pose_x"] = 0.0
                carto_position_setting["set_inital_pose_y"] = 0.0
                carto_position_setting["set_inital_pose_z"] = 0.0

                carto_position_setting["set_inital_pose_ox"] = 0.0
                carto_position_setting["set_inital_pose_oy"] = 0.0
                carto_position_setting["set_inital_pose_oz"] = 0.0
                carto_position_setting["set_inital_pose_ow"] = 1.0

                self.writ_patam(robot_cartopose_param_url, carto_position_setting)
            except:
                rospy.loginfo("3，save_position出错！")    
        else:
             rospy.loginfo("invalid isClear param %d", isClear)

    def networkconnect_thread_func(self):
        exit_code = os.system('ping -c 1 www.baidu.com')
        if exit_code:
            rospy.loginfo("连接百度网络失败，请检查网络连接")
            for wifi_name in self.wifilist.keys():
                os.system("nmcli device wifi connect " + wifi_name + " password " + self.wifilist[wifi_name])
                time.sleep(15)
                rospy.loginfo("连接wifi名： %s", wifi_name)
                exit_code = os.system('ping -c 1 www.baidu.com')
                if (exit_code == 0):
                    rospy.loginfo("重新连接wifi名: " + wifi_name + " 密码: " + self.wifilist[wifi_name] + "成功！导航模块已经联网！停止子线程。")
                    if (self.AndroidAlarm.androidmsg_navinetworkstatus != 22):
                        self.androidNaviNetworkStatus_pub.publish(22)
                        #self.AndroidmsgReportPub.publish(22)
                        self.AndroidAlarm.androidmsg_navinetworkstatus = 22
                    return
        else:
            if (self.AndroidAlarm.androidmsg_navinetworkstatus != 22):
                self.androidNaviNetworkStatus_pub.publish(22)
                #self.AndroidmsgReportPub.publish(22)
                self.AndroidAlarm.androidmsg_navinetworkstatus = 22
                rospy.loginfo("导航模块连接网络成功！")
                self.wifiUpdate()
            return
        if (self.AndroidAlarm.androidmsg_navinetworkstatus != 21):
            self.androidNaviNetworkStatus_pub.publish(21)
            #self.AndroidmsgReportPub.publish(21)
            self.AndroidAlarm.androidmsg_navinetworkstatus = 21
            rospy.loginfo("导航模块连接网络失败！")   

    def connectwifi(self):
        self.networkThread = threading.Thread(target=self.networkconnect_thread_func)
        # start 之后，子线程开始执行，主线程也会继续执行，不会等待
        self.networkThread.start()  # 子线程执行
    
    def testmultirobotavoid_thread_func(self, msg):
        rospy.loginfo("enter testmultirobotavoid_thread_func")
        pub_robot_img = RobotInformation()  
        pub_goal = PointStamped()      
        pub_goal.point.x = msg.x
        pub_goal.point.y = msg.y
        pub_goal.point.z = msg.yaw
        pub_goal.header.frame_id = 'map'
        pub_goal.header.stamp = rospy.Time.now()  
        theda = atan2((msg.points[0].y-msg.y), (msg.points[0].x-msg.x))
        self.test_multi_robot_avoid_goal_pub.publish(pub_goal)
        pub_robot_img = msg
        self.RobotInformation_pub.publish(pub_robot_img)
        while self.thread_running and not rospy.is_shutdown():
            theda = theda + msg.angularSpeed * 0.2
            pub_goal.point.x = pub_goal.point.x + msg.linearSpeed*0.2*cos(theda)
            pub_goal.point.y = pub_goal.point.y + msg.linearSpeed*0.2*sin(theda)
            pub_goal.header.stamp = rospy.Time.now()  # Update timestamp
            self.test_multi_robot_avoid_goal_pub.publish(pub_goal)
            pub_robot_img.x = pub_goal.point.x 
            pub_robot_img.y = pub_goal.point.y 
            pub_robot_img.yaw = theda
            self.RobotInformation_pub.publish(pub_robot_img)
            time.sleep(0.2)

    
    def test_multrobot_avoid(self, msg):
        # Ensure the thread is not already running
        if self.testmultirobotavoidThread is not None and self.testmultirobotavoidThread.is_alive():
            rospy.logwarn("Thread is already running!")
            self.thread_running = False
            self.testmultirobotavoidThread.join()

        self.thread_running = True
        # Start a new thread with the message as an argument
        self.testmultirobotavoidThread = threading.Thread(
            target=self.testmultirobotavoid_thread_func,
            args=(msg,)  # Note the comma to make it a tuple
        )
        # start 之后，子线程开始执行，主线程也会继续执行，不会等待
        self.testmultirobotavoidThread.start()  # 子线程执行
    
    def testmultirobotavoid_thread_func1(self, msg):
        rospy.loginfo("enter testmultirobotavoid_thread_func1")
        pub_robot_img = RobotInformation()  
        pub_goal = PointStamped()      
        pub_goal.point.x = msg.x
        pub_goal.point.y = msg.y
        pub_goal.point.z = msg.yaw
        pub_goal.header.frame_id = 'map'
        pub_goal.header.stamp = rospy.Time.now()  
        theda = atan2((msg.points[0].y-msg.y), (msg.points[0].x-msg.x))
        self.test_multi_robot_avoid_goal_pub.publish(pub_goal)
        pub_robot_img = msg
        self.RobotInformation_pub.publish(pub_robot_img)
        while self.thread_running1 and not rospy.is_shutdown():
            theda = theda + msg.angularSpeed * 0.2
            pub_goal.point.x = pub_goal.point.x + msg.linearSpeed*0.2*cos(theda)
            pub_goal.point.y = pub_goal.point.y + msg.linearSpeed*0.2*sin(theda)
            pub_goal.header.stamp = rospy.Time.now()  # Update timestamp
            self.test_multi_robot_avoid_goal_pub.publish(pub_goal)
            pub_robot_img.x = pub_goal.point.x 
            pub_robot_img.y = pub_goal.point.y 
            pub_robot_img.yaw = theda
            self.RobotInformation_pub.publish(pub_robot_img)
            time.sleep(0.2)

    
    def test_multrobot_avoid1(self, msg):
        # Ensure the thread is not already running
        if self.testmultirobotavoidThread1 is not None and self.testmultirobotavoidThread1.is_alive():
            rospy.logwarn("Thread1 is already running!")
            self.thread_running1 = False
            self.testmultirobotavoidThread1.join()

        self.thread_running1 = True
        # Start a new thread with the message as an argument
        self.testmultirobotavoidThread1 = threading.Thread(
            target=self.testmultirobotavoid_thread_func1,
            args=(msg,)  # Note the comma to make it a tuple
        )
        # start 之后，子线程开始执行，主线程也会继续执行，不会等待
        self.testmultirobotavoidThread1.start()  # 子线程执行

    # 建图
    #   roslaunch cartographer_ros spirit_demo.launch
    #
    # 保存地图
    #   1停止接收子图
    #   rosservice call /finish_trajectory "trajectory_id: 0"
    #
    #   2保存地图
    #   rosservice call /write_state "filename: '/home/spirit/.ros/maps/daomaps.pbstream'
    #           include_unfinished_submaps: true"
    #
    #   3pbstream转pgm 和 ymal
    #   rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=
    #   /home/spirit/.ros/maps/daomaps -pbstream_filename=/home/spirit/.ros/maps/daomaps.pbstream -resolution=0.05
    #
    #   定位
    #   roslaunch cartographer_ros spirit_localization.launch load_state_filename:=/home/spirit/download/carto_map.pbstream

    def loading_cartographer_buildMap(self):
        # 这个会检查文件及其路径是否存在，否则自动创建路径和空文件
        position_path = os.path.dirname(os.path.realpath(robot_cartopose_param_url))
        POSITION_PATH = os.path.join(os.path.dirname(position_path), 'carto')
        if not os.path.exists(POSITION_PATH):
            os.mkdir(POSITION_PATH)
        positionfile = os.path.join(POSITION_PATH, 'carto_config.yaml')
        if not os.path.exists(positionfile):
            os.system(r"touch {}".format(positionfile))

        CARTOGRAPHER_PATH = spirit_dir+"cartographer_ros"
        if not os.path.exists(CARTOGRAPHER_PATH):
            os.mkdir(CARTOGRAPHER_PATH)
            os.system("cp -r /home/spirit/cartographer_ws/install_isolated/share/cartographer_ros/launch /home/spirit/.ros/cartographer_ros/")
            os.system("cp -r /home/spirit/cartographer_ws/install_isolated/share/cartographer_ros/configuration_files /home/spirit/.ros/cartographer_ros/")

        try:
            if self.carto_build_launch_flag:
                rospy.loginfo("停止cartographer建图，然后重新加载cartographer！")
                self.carto_build_launch.shutdown()
                time.sleep(3)
            self.carto_build_launch_flag = False
        except:
            rospy.loginfo("尝试停止cartographer建图，当前不处于建图模式！")

        try:
            if self.carto_local_launch_flag:
                rospy.loginfo("停止cartographer导航定位，然后重新加载cartographer！")
                self.carto_local_launch.shutdown()
                time.sleep(3)
            self.carto_local_launch_flag = False
        except:
            rospy.loginfo("尝试停止cartographer导航定位，当前不处于导航定位模式！")

        try:
            if self.carto_rebuild_launch_flag:
                rospy.loginfo("停止cartographer重建图，然后重新加载cartographer！")
                self.carto_rebuild_launch.shutdown()
                time.sleep(3)
            self.carto_rebuild_launch_flag = False
        except:
            rospy.loginfo("尝试停止cartographer重建图，当前不处于重建图模式！")

        if self.carto_build_launch_flag == False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            roslaunch_file = ['/home/spirit/.ros/cartographer_ros/launch/spirit.launch']
            self.carto_build_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
            self.carto_build_launch.start()
            self.carto_build_launch_flag = True
        rospy.loginfo("cartographer建图模式重启成功！")

    def loading_cartographer_rebuildMap(self, filename):
        # 这个会检查文件及其路径是否存在，否则自动创建路径和空文件
        position_path = os.path.dirname(os.path.realpath(robot_cartopose_param_url))
        POSITION_PATH = os.path.join(os.path.dirname(position_path), 'carto')
        if not os.path.exists(POSITION_PATH):
            os.mkdir(POSITION_PATH)
        positionfile = os.path.join(POSITION_PATH, 'carto_config.yaml')
        if not os.path.exists(positionfile):
            os.system(r"touch {}".format(positionfile))

        CARTOGRAPHER_PATH = spirit_dir+"cartographer_ros"
        if not os.path.exists(CARTOGRAPHER_PATH):
            os.mkdir(CARTOGRAPHER_PATH)
            os.system("cp -r /home/spirit/cartographer_ws/install_isolated/share/cartographer_ros/launch /home/spirit/.ros/cartographer_ros/")
            os.system("cp -r /home/spirit/cartographer_ws/install_isolated/share/cartographer_ros/configuration_files /home/spirit/.ros/cartographer_ros/")

        try:
            if self.carto_build_launch_flag:
                rospy.loginfo("停止cartographer建图，然后重新加载cartographer！")
                self.carto_build_launch.shutdown()
                time.sleep(3)
            self.carto_build_launch_flag = False
        except:
            rospy.loginfo("尝试停止cartographer建图，当前不处于建图模式！")

        try:
            if self.carto_local_launch_flag:
                rospy.loginfo("停止cartographer导航定位，然后重新加载cartographer！")
                self.carto_local_launch.shutdown()
                time.sleep(3)
            self.carto_local_launch_flag = False
        except:
            rospy.loginfo("尝试停止cartographer导航定位，当前不处于导航定位模式！")

        try:
            if self.carto_rebuild_launch_flag:
                rospy.loginfo("停止cartographer重建图，然后重新加载cartographer！")
                self.carto_rebuild_launch.shutdown()
                time.sleep(3)
            self.carto_rebuild_launch_flag = False
        except:
            rospy.loginfo("尝试停止cartographer重建图，当前不处于重建图模式！")

        if self.carto_rebuild_launch_flag == False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            # filename = spirit_dir+'maps/zhengmaps3.pbstream'
            filename = filename.strip('.pbstream')

            cli_args = ['/home/spirit/.ros/cartographer_ros/launch/spirit_relocalization.launch','load_state_filename:='+filename]

            roslaunch_args = cli_args[1:]
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
            self.carto_rebuild_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
            self.carto_rebuild_launch.start()
            self.carto_rebuild_launch_flag = True
        rospy.loginfo("cartographer扩建建图模式重启成功！")

    def loading_cartographer_loadMap(self, filename):
        #从mapserver_param_url里读取carto_env的参数，如果没有，默认使用usually
        try:
            params = self.read_param(mapserver_param_url)
        except Exception as e:
            rospy.logerr("读取 YAML 文件失败: %s", e)
            params = {}
        carto_env = params.get("carto_env", "usually")
        rospy.logwarn("读取carto_env参数: %s",carto_env)

        # 这个会检查文件及其路径是否存在，否则自动创建路径和空文件
        position_path = os.path.dirname(os.path.realpath(robot_cartopose_param_url))
        POSITION_PATH = os.path.join(os.path.dirname(position_path), 'carto')
        if not os.path.exists(POSITION_PATH):
            os.mkdir(POSITION_PATH)
        positionfile = os.path.join(POSITION_PATH, 'carto_config.yaml')
        if not os.path.exists(positionfile):
            os.system(r"touch {}".format(positionfile))

        CARTOGRAPHER_PATH = spirit_dir+"cartographer_ros"
        if not os.path.exists(CARTOGRAPHER_PATH):
            os.mkdir(CARTOGRAPHER_PATH)
            os.system("cp -r /home/spirit/cartographer_ws/install_isolated/share/cartographer_ros/launch /home/spirit/.ros/cartographer_ros/")
            os.system("cp -r /home/spirit/cartographer_ws/install_isolated/share/cartographer_ros/configuration_files /home/spirit/.ros/cartographer_ros/")

        try:
            if self.carto_build_launch_flag:
                rospy.loginfo("停止cartographer建图，然后重新加载cartographer！")
                self.carto_build_launch.shutdown()
                time.sleep(3)
            self.carto_build_launch_flag = False
        except:
            rospy.loginfo("尝试停止cartographer建图，当前不处于建图模式！")

        try:
            if self.carto_local_launch_flag:
                rospy.loginfo("停止cartographer导航定位，然后重新加载cartographer！")
                self.carto_local_launch.shutdown()
                time.sleep(3)
            self.carto_local_launch_flag = False
        except:
            rospy.loginfo("尝试停止cartographer导航定位，当前不处于导航定位模式！")

        try:
            if self.carto_rebuild_launch_flag:
                rospy.loginfo("停止cartographer重建图，然后重新加载cartographer！")
                self.carto_rebuild_launch.shutdown()
                time.sleep(3)
            self.carto_rebuild_launch_flag = False
        except:
            rospy.loginfo("尝试停止cartographer重建图，当前不处于重建图模式！")

        if self.carto_local_launch_flag == False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            # filename = spirit_dir+'maps/zhengmaps3.pbstream'
            filename = filename.strip('.pbstream')
            cli_args = ['/home/spirit/.ros/cartographer_ros/launch/spirit_localization.launch','load_state_filename:='+filename,'load_cartographer_env:='+carto_env]
            # cli_args = ['cartographer_ros spirit_localization.launch','load_state_filename:='+filename]
            roslaunch_args = cli_args[1:]
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
            self.carto_local_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
            self.carto_local_launch.start()
            self.carto_local_launch_flag = True

        if self.publishlocation_miss == 3:
            self.publishlocation_miss = 0
            self.locationstatus_pub.publish(3)
            rospy.loginfo("机器人保存地图后加载cartographer，不进行重新定位！！！！")
        elif self.publishlocation_miss == 1:
            self.publishlocation_miss = 0
            self.locationstatus_pub.publish(1)
            rospy.loginfo("机器人切换地图，需要进行重新定位！！！！")

        rospy.loginfo("cartographer导航定位模式重启成功！")

    # rosbrige模块
    def loading_rosbridge(self):
        try:
            rospy.loginfo("启动 rosbridge_server")
            if self.rosbridge_launch_flag == False:
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)
                self.rosbridge_launch = roslaunch.parent.ROSLaunchParent(uuid, ["/opt/ros/melodic/share/rosbridge_server/launch/rosbridge_websocket.launch"])
                # self.rosbridge_launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/spirit/deliverRos/src/rosbridge_suite/rosbridge_server/launch/rosbridge_websocket.launch"])
                if not self.slaveMachine:
                    self.rosbridge_launch.start()
                self.rosbridge_launch_flag = True
                rospy.loginfo("启动rosbrdge server成功！")
        except Exception as e:
            rospy.loginfo("尝试启动rosbridge server 失败!%s"%str(e))
    
    def shutdown_rosbridge(self):
        try:
            if self.rosbridge_launch_flag:
                rospy.loginfo("关闭rosbridge server")
                self.rosbridge_launch.shutdown()
                time.sleep(2.0)
                rospy.loginfo("关闭rosbridge server成功")
            self.rosbridge_launch_flag = False
        except:
            rospy.loginfo("关闭rosbridge失败!")

    # Movebase模块
    def loading_movebase(self):
        try:
            rospy.loginfo("开始启动Movebase")
            if self.movebase_launch_flag == False:
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)
                if not self.slaveMachine:
                    roslaunch_file = ['/opt/ros/melodic/share/spirit_main/launch/robot_navigation.launch']
                else:
                    roslaunch_file = ['/home/spirit/deliverRos/src/spirit_main/launch/robot_navigation.launch']

                self.movebase_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
                self.movebase_launch.start()
                self.movebase_launch_flag = True
                rospy.loginfo("启动Movebase成功！")
        except Exception as e:
            rospy.loginfo("尝试启动movebase失败!%s"%str(e))

    def shutdown_movebase(self):
        self.movebase_ready = False
        try:
            if self.movebase_launch_flag:
                rospy.loginfo("关闭 Movebase")
                self.movebase_launch.shutdown()
                time.sleep(1)
                rospy.loginfo("关闭 Movebase成功！")
            self.movebase_launch_flag = False
        except:
            rospy.loginfo("尝试关闭movebase失败!")

    # 双目摄像头
    def loading_camera(self):
        if (self.startcamera == False):
            return
        
        try:
            if self.camera_launch_flag == False:
                rospy.loginfo("启动双目摄像头")
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)
                roslaunch_file = ['/opt/ros/melodic/share/spirit_main/launch/multi_dabai.launch']
                self.camera_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
                self.camera_launch.start()
                rospy.loginfo("启动双目摄像头成功！")

            self.camera_launch_flag = True
        except:
            rospy.loginfo("尝试启动双目摄像头失败!")

    def shutdown_camera(self):
        try:
            if self.camera_launch_flag:
                rospy.loginfo("关闭双目摄像头")
                self.camera_launch.shutdown()
                time.sleep(0.5)
                rospy.loginfo("关闭双目摄像头成功！")
            self.camera_launch_flag = False
        except:
            rospy.loginfo("尝试关闭双目摄像头失败!")
    

    # 多机避障
    def loading_multirobotsAvoid(self):
        if (self.startMultiRobotsAvoid == 0):
            return
        
        try:
            if self.multirobotsAvoid_launch_flag == False:
                rospy.loginfo("启动多机避障")
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)
                if self.startMultiRobotsAvoid == 1:
                    rospy.loginfo("启动多机避障esp")
                    roslaunch_file = ['/opt/ros/melodic/share/spirit_main/launch/spirit_esp_now.launch']
                    self.multirobotsAvoid_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
                if self.startMultiRobotsAvoid == 2:
                    rospy.loginfo("启动多机避障wlan")
                    roslaunch_file = ['/opt/ros/melodic/share/spirit_main/launch/spirit_wlansocket.launch']
                    self.multirobotsAvoid_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

                self.multirobotsAvoid_launch.start()
                # 广播给movebase，启动了多机避障
                self.espload_pub.publish(1)
                self.esp_status = True
                rospy.loginfo("启动多机避障成功！")

            self.multirobotsAvoid_launch_flag = True
        except:
            rospy.loginfo("尝试启动多机避障失败!")

    def shutdown_multirobotsAvoid(self):
        try:
            if self.multirobotsAvoid_launch_flag:
                rospy.loginfo("关闭多机避障")
                self.espload_pub(0)
                self.multirobotsAvoid_launch.shutdown()
                time.sleep(0.5)
                self.esp_status = False
                rospy.loginfo("关闭多机避障成功！")
            self.multirobotsAvoid_launch_flag = False
        except:
            rospy.loginfo("尝试关闭多机避障失败!")

    # 激光
    def loading_scan(self):
        return self.lidar_owner.loading()

    def shutdown_scan(self):
        return self.lidar_owner.shutdown()

    def vendor_loading_scan(self):
        if self.startScan == False:
            return
    
        try:
            if self.scan_launch_flag == False:
                rospy.loginfo("启动激光")
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)
                roslaunch_file = ['/opt/ros/melodic/share/spirit_main/launch/lidar/lidar.launch']
                self.scan_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
                self.scan_launch.start()
                rospy.loginfo("启动激光成功！")

            self.scan_launch_flag = True
        except:
            rospy.loginfo("尝试启动激光失败!")

    def vendor_shutdown_scan(self):
        try:
            if self.scan_launch_flag:
                rospy.loginfo("关闭激光")
                self.scan_launch.shutdown()
                time.sleep(0.5)
                rospy.loginfo("关闭激光成功！")
            self.scan_launch_flag = False
        except:
            rospy.loginfo("尝试关闭激光失败!")
    
    # IMU
    def loading_imu(self):
        if self.startImu == False:
            return
        
        try:
            if self.imu_launch_flag == False:
                rospy.loginfo("启动IMU")
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)                
                roslaunch_file = ['/opt/ros/melodic/share/spirit_main/launch/imu/imu.launch']
                self.imu_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
                self.imu_launch.start()
                rospy.loginfo("启动IMU成功！")

            self.imu_launch_flag = True
        except:
            rospy.loginfo("尝试启动IMU失败!")

    def shutdown_imu(self):
        try:
            if self.imu_launch_flag:
                rospy.loginfo("关闭IMU")
                self.imu_launch.shutdown()
                time.sleep(0.5)
                rospy.loginfo("关闭IMU成功！")
            self.imu_launch_flag = False
        except:
            rospy.loginfo("尝试关闭IMU失败!")
    
    # ODOM 驱动
    def loading_driver(self):
        if self.startDriver == False:
            return
        
        try:
            if self.driver_launch_flag == False:
                rospy.loginfo("启动ODOM驱动")
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)                
                roslaunch_file = ['/opt/ros/melodic/share/spirit_main/launch/robot_driver.launch']
                self.driver_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
                self.driver_launch.start()
                rospy.loginfo("启动ODOM驱动成功！")

            self.driver_launch_flag = True
        except:
            rospy.loginfo("尝试启动ODOM驱动失败!")

    def shutdown_driver(self):
        try:
            if self.driver_launch_flag:
                rospy.loginfo("关闭ODOM驱动")
                self.driver_launch.shutdown()
                time.sleep(1.0)
                rospy.loginfo("关闭ODOM驱动成功！")
            self.driver_launch_flag = False
        except:
            rospy.loginfo("尝试关闭ODOM驱动失败!")
    
    # waypoint 
    def loading_waypoint(self):
        if self.startWaypoint == False:
            return
        
        try:
            if self.waypoint_launch_flag == False:
                rospy.loginfo("启动waypoint")
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)                
                roslaunch_file = ['/opt/ros/melodic/share/spirit_main/launch/waypoint.launch']
                self.waypoit_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
                self.waypoit_launch.start()
                rospy.loginfo("启动waypoint成功！")

            self.waypoint_launch_flag = True
        except:
            rospy.loginfo("尝试启动waypoint失败!")

    def shutdown_waypoint(self):
        try:
            if self.waypoint_launch_flag:
                rospy.loginfo("关闭waypoint")
                self.waypoit_launch.shutdown()
                time.sleep(0.5)
                rospy.loginfo("关闭waypoint成功！")
            self.waypoint_launch_flag = False
        except:
            rospy.loginfo("尝试关闭waypoint失败!")
    
    # bag
    def loading_bag(self):
        if self.startBag == False:
            return
        
        try:
            if self.bag_launch_flag == False:
                rospy.loginfo("启动bag")
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)                
                roslaunch_file = ['/opt/ros/melodic/share/spirit_main/launch/bag.launch']
                self.bag_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
                self.bag_launch.start()
                rospy.loginfo("启动bag成功！")

            self.bag_launch_flag = True
        except:
            rospy.loginfo("尝试启动bag失败!")

    def shutdown_bag(self):
        try:
            if self.bag_launch_flag:
                rospy.loginfo("关闭bag")
                self.bag_launch.shutdown()
                time.sleep(0.5)
                rospy.loginfo("关闭bag成功！")
            self.bag_launch_flag = False
        except:
            rospy.loginfo("尝试关闭bag失败!")    
   
    # backend
    def loading_backend(self):
        if self.startBackend == False:
            return
        
        try:
            if self.backend_launch_flag == False:
                rospy.loginfo("启动backend")
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)                
                roslaunch_file = ['/opt/ros/melodic/share/spirit_main/launch/backend.launch']
                self.backend_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
                self.backend_launch.start()
                rospy.loginfo("启动backend成功！")

            self.backend_launch_flag = True
        except:
            rospy.loginfo("尝试启动backend失败!")

    def shutdown_backend(self):
        try:
            if self.backend_launch_flag:
                rospy.loginfo("关闭backend")
                self.backend_launch.shutdown()
                time.sleep(1.0)
                rospy.loginfo("关闭backend成功！")
            self.backend_launch_flag = False
        except:
            rospy.loginfo("尝试关闭backend失败!")
    
    # autoupgrade
    def loading_autoupgrade(self):
        if self.startAutoupgrade == False:
            return
        
        try:
            if self.autoupgrade_launch_flag == False:
                rospy.loginfo("启动autoupgrade")
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)                
                roslaunch_file = ['/opt/ros/melodic/share/spirit_main/launch/autoupgrade.launch']
                self.autoupgrade_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
                self.autoupgrade_launch.start()
                rospy.loginfo("启动autoupgrade成功！")

            self.autoupgrade_launch_flag = True
        except:
            rospy.loginfo("尝试启动autoupgrade失败!")

    def shutdown_autoupgrade(self):
        try:
            if self.autoupgrade_launch_flag:
                rospy.loginfo("关闭autoupgrade")
                self.autoupgrade_launch.shutdown()
                time.sleep(0.2)
                rospy.loginfo("关闭autoupgrade成功！")
            self.autoupgrade_launch_flag = False
        except:
            rospy.loginfo("尝试关闭autoupgrade失败!")
    
    # lora
    def loading_lora(self):
        if self.startLora== False:
            return
        
        try:
            if self.lora_launch_flag == False:
                rospy.loginfo("启动lora")
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)                
                roslaunch_file = ['/opt/ros/melodic/share/spirit_main/launch/spirit_lora_manager.launch']
                self.lora_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
                self.lora_launch.start()
                self.lora_status = True
                rospy.loginfo("启动lora成功！")

            self.lora_launch_flag = True
        except:
            rospy.loginfo("尝试启动lora失败!")

    def shutdown_lora(self):
        try:
            if self.lora_launch_flag:
                rospy.loginfo("关闭rola")
                self.lora_launch.shutdown()
                time.sleep(0.5)
                self.lora_status = False
                rospy.loginfo("关闭rola成功！")
            self.lora_launch_flag = False
        except:
            rospy.loginfo("尝试关闭rola失败!")
    
    # ekf
    def loading_ekf(self):
        if self.startEkf== False:
            return
        
        try:
            if self.ekf_launch_flag == False:
                rospy.loginfo("启动ekf")
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)                
                roslaunch_file = ['/opt/ros/melodic/share/spirit_main/launch/ekf.launch']
                self.ekf_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
                self.ekf_launch.start()
                rospy.loginfo("启动ekf成功！")
            self.ekf_launch_flag = True
        except:
            rospy.loginfo("尝试启动ekf失败!")

    def shutdown_ekf(self):
        try:
            if self.ekf_launch_flag:
                rospy.loginfo("关闭ekf")
                self.ekf_launch.shutdown()
                time.sleep(0.5)
                rospy.loginfo("关闭ekf成功！")
            self.ekf_launch_flag = False
        except:
            rospy.loginfo("尝试关闭ekf失败!")
    
    # mapserver
    def loading_mapserver_loadMap(self, filename):
        try:
            if self.mapserver_launch_flag:
                rospy.loginfo("停止mapserver，然后重新加载mapserver！")
                self.mapserver_launch.shutdown()
                time.sleep(3)
            self.mapserver_launch_flag = False
        except:
            rospy.loginfo("尝试停止mapserver失败！")

        if self.mapserver_launch_flag == False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)

            filename = filename.strip('.pbstream')
            cli_args = ['/opt/ros/melodic/share/spirit_main/launch/map_server.launch','load_map_filename:='+filename]
            roslaunch_args = cli_args[1:]
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
            self.mapserver_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
            self.mapserver_launch.start()
            self.mapserver_launch_flag = True

        rospy.loginfo("mapserver加载地图成功！")

    def check_map(self, isloadingMap=False):
        self.local_point_list = []  # check
        self.local_systempoint_list = [] # check
        self.local_globalplans_list = [] # check
        self.local_json_map = "" # type: str
        self.local_json_nglobalPlan = "" # type: str

        self.map_isEmpty = 1
        if not self.odom_ready:
            rospy.logwarn("check_map fail, odom is not ready, waiting...")
            return
        if not self.scan_ready:
            rospy.logwarn("check_map fail, scan is not ready, waiting...")
            return
        if not self.imu_ready:
            rospy.logwarn("check_map fail, imu is not ready, waiting...")
            return

        # 这个会检查文件及其路径是否存在，否则自动创建路径和空文件
        cur_path = os.path.dirname(os.path.realpath(spirit_map_url))
        MAP_PATH = os.path.join(os.path.dirname(cur_path), 'maps')
        if not os.path.exists(MAP_PATH):
            os.mkdir(MAP_PATH)
        mapfile = os.path.join(MAP_PATH, 'spiritMaps.json')
        if not os.path.exists(mapfile):
            os.system(r"touch {}".format(mapfile))

        self.allMap = None
        try:
            self.allMap = self.read_json(spirit_map_url)  # get map
        except:
            if (self.AndroidAlarm.androidmsg_mapstatus != 61):
                self.androidMapStatus_pub.publish(61)
                #self.AndroidmsgReportPub.publish(61)
                self.AndroidAlarm.androidmsg_mapstatus = 61
            rospy.logwarn("check_map fail, allMap is Nont Find")
        if self.allMap is None:
            if isloadingMap:
                rospy.loginfo("check_map成功，开始启动新建地图模式！")
                self.map_status = 1
                self.mapStatus_pub.publish(1)
                self.map_loading = True  # map none
                # self.loading_cartographer_buildMap() #getmap is none
            return
        mapSetting_bean = MapSetting(self.allMap)
        # rospy.loginfo("DefuatfloorId %d", mapSetting_bean.getDefaultFloor())

        for floor in mapSetting_bean.floors:

            floorbean = Floor(floor)
            if int(mapSetting_bean.getDefaultFloor()) == int(floorbean.getFloorId()):
                rospy.loginfo("check_map floorid = %s floorName = %s defaultmapId = %s ",str(floorbean.getFloorId()), str(floorbean.getFloorName()), str(floorbean.getDefaultmap()))
                self.current_floor_num = int(floorbean.getFloorId())
                self.current_floor_name = str(floorbean.getFloorName())
                if floorbean.getMaps():
                    for map in floorbean.getMaps():
                        mapbean = Map(map)

                        # 填充地图列表 开始
                        if int(floorbean.getDefaultmap()) == int(mapbean.getMapId()):
                            self.current_map_id = int(mapbean.getMapId())
                            self.current_map_name = str(mapbean.getMapName())
                            self.map_isEmpty = 0
                            # 是否加载地图
                            if isloadingMap:
                                if os.access(mapbean.getMapFile(), os.F_OK):
                                    # 指定的文件或目录存在
                                    rospy.loginfo("loading map %s", mapbean.getMapFile())
                                    self.map_file = str(mapbean.getMapFile())
                                    self.map_status = 0
                                    self.mapStatus_pub.publish(0)

                                    rospy.loginfo("机器人处于加载地图模式，可以正常导航。")
                                    #self.AndroidmsgReportPub.publish(70)
                                    self.androidMapBuildStatus_pub.publish(70)
                                    self.AndroidAlarm.androidmsg_mapbuildstatus = 70
                                    self.map_loading = True  # map loading
                                else:
                                    if (self.AndroidAlarm.androidmsg_mapstatus != 61):
                                        #self.AndroidmsgReportPub.publish(61)
                                        self.androidMapStatus_pub.publish(61)
                                        self.AndroidAlarm.androidmsg_mapstatus = 61
                                    self.map_ready = False
                                    self.localmap_ready = False
                                    rospy.logerr("cartographer地图文件（%s）没有找到！！！！", mapbean.getMapFile())

                            self.local_json_map = map
                            if "nglobalPlans" in map:
                                self.local_json_nglobalPlan = map["nglobalPlans"]
                            if mapbean.getPoints():
                                point_list = []
                                for point in mapbean.getPoints():
                                    pointbean = MapsBeanPoint(point)

                                    pointList = PointListEntry()
                                    pointList.point_id = str(pointbean.getPointId())
                                    pointList.point_name = str(pointbean.getPointName())
                                    pointList.x = (pointbean.getPositionX())
                                    pointList.y = (pointbean.getPositionY())
                                    pointList.z = (pointbean.getPositionYaw())
                                    point_list.append(pointList)
                                self.local_point_list = point_list

                            if mapbean.getSystemPoints():
                                point_list = []
                                for point in mapbean.getSystemPoints():
                                    pointbean = MapsBeanPoint(point)

                                    pointList = PointListEntry()
                                    pointList.point_id = str(pointbean.getPointId())
                                    pointList.point_name = str(pointbean.getPointName())
                                    pointList.x = (pointbean.getPositionX())
                                    pointList.y = (pointbean.getPositionY())
                                    pointList.z = (pointbean.getPositionYaw())
                                    point_list.append(pointList)
                                    if (pointList.point_name == "桩位置"):
                                        # 在配置文件里发现了桩位置，说鸣充电桩修正过
                                        self.fix_chargepoint_flag = True
                                        chargepoint = Point32()
                                        chargepoint.x = pointList.x
                                        chargepoint.y = pointList.y
                                        chargepoint.z = pointList.z
                                        self.charge_point_pub.publish(chargepoint)

                                self.local_systempoint_list = point_list

                            if mapbean.getGlobalPlans():
                                globalPlans_list = []
                                for globalplan in mapbean.getGlobalPlans():
                                    globalplanbean = MapsBeanGlobalplan(globalplan)

                                    one_globalplan = GlobalPlanEntry()
                                    one_globalplan.globalplanId = globalplanbean.getGlobalplanId()
                                    one_globalplan.globalplanName = str(globalplanbean.getGlobalplanName())
                                    one_globalplan.dir = globalplanbean.getDir()
                                    one_globalplan.lines = globalplanbean.getLines()
                                    one_globalplan.addPoints = globalplanbean.getAddPoints()
                                    globalPlans_list.append(one_globalplan)
                                self.local_globalplans_list = globalPlans_list

                            if mapbean.getCleanAreas() is not None:
                                self.cleanArea_json = mapbean.getCleanAreas()

                        # 填充地图列表 结束
        self.pubPointsPose() #放在這裡，所有更新操作都會讓發布的導航點更新
        
        robot_floormap = RobotFloorMap()
        robot_floormap.floorId = self.current_floor_num
        robot_floormap.floorName = self.current_floor_name
        robot_floormap.mapId = self.current_map_id
        robot_floormap.mapName = self.current_map_name
        self.current_floormap_pub.publish(robot_floormap)

    # 传入 地图 高速路线 和 目标点 id 
    # 返回高速路段 TargetGoalEntry[] 和 这段高速路段的 路线长度 total_length
    def check_gaosugonglu(self, floor_id, local_amap, startid, endid):
        targetGoalEntryList = []
        total_length = 0.0
        point = None
        
        a = local_amap
        b = local_amap["nglobalPlans"]
        if not b:
            return (targetGoalEntryList, total_length)
        
        temp_endid =int(endid)
        mapbean = Map(a)

        shortest_path = self.dijkstra(b,a, mapbean, startid,temp_endid)
        temp_endbean = self.get_point_by_id(mapbean, temp_endid)
        print(shortest_path)
        if shortest_path:
            path_with_speed = []
            # 遍历最短路径中的节点
            for i in range(len(shortest_path)-1): # 防止下标越界，最后一个点不需要设置速度
                point_id = shortest_path[i]
                next_point_id = shortest_path[i+1]

                # 在线路信息中查找对应的线路. startNid ,endNid 可能在多条路线上使用，保证采集的速度，在这条线上
                for line in b["nLines"]:
                    # 判断线路的方向和起点终点是否匹配
                    if line["direction"] == 2 and ((line["startNid"] == point_id and line["endNid"] == next_point_id) or (line["startNid"] == next_point_id and line["endNid"] == point_id)):
                        speed = line["speed"]
                        path_with_speed.append((point_id, speed))
                        if i+1 == len(shortest_path)-1:# 添加终点节点和速度信息
                            path_with_speed.append((next_point_id, speed))
                        break
                    # 判断线路的方向和起点终点是否匹配
                    elif line["direction"] == 1 and line["startNid"] == point_id and line["endNid"] == next_point_id:
                        speed = line["speed"]
                        path_with_speed.append((point_id, speed))
                        if i+1 == len(shortest_path)-1:# 添加终点节点和速度信息
                            path_with_speed.append((next_point_id, speed))
                        break
                    # 判断线路的方向和起点终点是否匹配
                    elif line["direction"] == 2 and line["startNid"] == next_point_id and line["endNid"] == point_id:
                        speed = line["speed"]
                        path_with_speed.append((point_id, speed))
                        if i+1 == len(shortest_path)-1: # 添加终点节点和速度信息
                            path_with_speed.append((next_point_id, speed))
                        break

            # 添加终点节点和速度信息
            #path_with_speed.append((shortest_path[-1], 0.2))
            #print(path_with_speed)
                                # eg: path_with_speed  [(1709686116, 0.4000000059604645), (1709686118, 0.5), (1709686120, 0.4000000059604645),
                                # (1709686123, 0.4000000059604645), (1709686125, 0.4000000059604645), (1709685924, 0.4000000059604645),
                                # (1709685927, 0.4000000059604645), (1709685930, 0.4000000059604645), (1709685839, 0.4000000059604645),
                                # (1709617957, 0.4000000059604645)]
            if len(path_with_speed) >1:
                # m =PoseEntryList()
                p1 = None    
                prev_speed = 0     
                custom_datas=[]   
                short_count = 0
                for short in path_with_speed:
                    findpoint = False
                    for point in mapbean.getPoints() :                        
                        if short[0] == point["pointId"]:
                            #print (short[0])
                            findpoint = True
                            if p1 != None:
                                distance = sqrt((point["positionX"] - p1["positionX"]) ** 2 + (point["positionY"] - p1["positionY"]) ** 2)
                                total_length += distance
                            p1 = point

                            q = transformations.quaternion_from_euler(0.0, 0.0, temp_endbean.getPositionYaw()/180.0*pi)
                            if (short[1] != prev_speed and prev_speed !=0):
                                te = TargetGoalEntry()
                                rospy.loginfo("yaw: %f", temp_endbean.getPositionYaw())
                                
                                # 默认为高速公路 默认没有区域绑定, 需要遍历区域时候 统一设置
                                # 等待点也是统一添加 
                                te.point_id = int(point["pointId"])
                                te.point_name = str(point["pointName"])
                                te.device_id = str(point["deviceID"])
                                te.floor_id = floor_id
                                if point["ledWarnID"]:
                                    te.led_warn_id =  int(point["ledWarnID"])
                                te.map_id =mapbean.getMapId()
                                te.goal.header.frame_id = "map"
                                te.goal.pose.position.x = temp_endbean.getPositionX()
                                te.goal.pose.position.y = temp_endbean.getPositionY()
                                te.goal.pose.position.z = 0
                                te.goal.pose.orientation.x = q[0]
                                te.goal.pose.orientation.y = q[1]
                                te.goal.pose.orientation.z = q[2]
                                te.goal.pose.orientation.w = q[3]
                                te.custom_path.data = custom_datas
                                te.type = 10   #高速公路
                                te.point_line_speed = short[1]
                                te.point_angular_speed = short[1]
                                te.enter_elevator = False

                                # 在路线中 高速路路段补充 经过区域类型
                                if mapbean.getRegionList():
                                    for regionn in mapbean.getRegionList(): # 遍历区域 高速公路经过的区域
                                        region = Region(regionn)
                                        if short[0] in (region.getStartPointId(), region.getEndPointId(), region.getWaitPointId()):
                                            te.type = region.getType()
                                            break

                                # 在路线中 给每个点 补充等待点
                                if point["waitPoints"]:
                                    for waitpoin in point["waitPoints"]:
                                        waitpoint = Position(waitpoin)
                                        wp = geometry_msgs.msg.Point()
                                        wp.x = waitpoint.getX()
                                        wp.y = waitpoint.getY()
                                        te.waitgoal_list.append(wp)
                                targetGoalEntryList.append(te)
                                rospy.loginfo("path submitted, custom_datas size: %d", len(custom_datas))

                                custom_datas = []
                                point_data = PoseStamped()
                                point_data.header.frame_id = "map"
                                point_data.pose.position.x = point["positionX"]
                                point_data.pose.position.y = point["positionY"]
                                point_data.pose.position.z = 0
                                point_data.pose.orientation.x = q[0]
                                point_data.pose.orientation.y = q[1]
                                point_data.pose.orientation.z = q[2]
                                point_data.pose.orientation.w = q[3]
                                custom_datas.append(point_data)
                                prev_speed = short[1]                                 
                            else:
                                point_data = PoseStamped()
                                point_data.header.frame_id = "map"
                                point_data.pose.position.x = point["positionX"]
                                point_data.pose.position.y = point["positionY"]
                                point_data.pose.position.z = 0
                                point_data.pose.orientation.x = q[0]
                                point_data.pose.orientation.y = q[1]
                                point_data.pose.orientation.z = q[2]
                                point_data.pose.orientation.w = q[3]
                                custom_datas.append(point_data)

                    if findpoint == False:
                        if short_count !=0:
                            for point in mapbean.getSystemPoints() :
                                # print("catalpa short[0]: "+short[0]+" , short[1]: "+short[1])
                                if short[0] == point["pointId"]:
                                    if p1 != None:
                                        distance = sqrt((point["positionX"] - p1["positionX"]) ** 2 + (point["positionY"] - p1["positionY"]) ** 2)
                                        total_length += distance
                                    p1 = point

                                    q = transformations.quaternion_from_euler(0.0, 0.0, temp_endbean.getPositionYaw()/180.0*pi)
                                    if (short[1] != prev_speed and prev_speed !=0):
                                        te = TargetGoalEntry()
                                        rospy.loginfo("yaw: %f", temp_endbean.getPositionYaw())
                                        
                                        # 默认为高速公路 默认没有区域绑定, 需要遍历区域时候 统一设置
                                        # 等待点也是统一添加 
                                        te.point_id = int(point["pointId"])
                                        te.point_name = str(point["pointName"])
                                        te.device_id = str(point["deviceID"])
                                        te.floor_id = floor_id
                                        if point["ledWarnID"]:
                                            te.led_warn_id =  int(point["ledWarnID"])
                                        te.map_id =mapbean.getMapId()
                                        te.goal.header.frame_id = "map"
                                        te.goal.pose.position.x = temp_endbean.getPositionX()
                                        te.goal.pose.position.y = temp_endbean.getPositionY()
                                        te.goal.pose.position.z = 0
                                        te.goal.pose.orientation.x = q[0]
                                        te.goal.pose.orientation.y = q[1]
                                        te.goal.pose.orientation.z = q[2]
                                        te.goal.pose.orientation.w = q[3]
                                        te.custom_path.data = custom_datas
                                        te.type = 10   #高速公路
                                        te.point_line_speed = short[1]
                                        te.point_angular_speed = short[1]
                                        te.enter_elevator = False

                                        # 在路线中 高速路路段补充 经过区域类型
                                        if mapbean.getRegionList():
                                            for regionn in mapbean.getRegionList(): # 遍历区域 高速公路经过的区域
                                                region = Region(regionn)
                                                if short[0] in (region.getStartPointId(), region.getEndPointId(), region.getWaitPointId()):
                                                    te.type = region.getType()
                                                    break

                                        targetGoalEntryList.append(te)
                                        rospy.loginfo("path submitted, custom_datas size: %d", len(custom_datas))

                                        custom_datas = []
                                        point_data = PoseStamped()
                                        point_data.header.frame_id = "map"
                                        point_data.pose.position.x = point["positionX"]
                                        point_data.pose.position.y = point["positionY"]
                                        point_data.pose.position.z = 0
                                        point_data.pose.orientation.x = q[0]
                                        point_data.pose.orientation.y = q[1]
                                        point_data.pose.orientation.z = q[2]
                                        point_data.pose.orientation.w = q[3]
                                        custom_datas.append(point_data)
                                        prev_speed = short[1]                                 
                                    else:
                                        point_data = PoseStamped()
                                        point_data.header.frame_id = "map"
                                        point_data.pose.position.x = point["positionX"]
                                        point_data.pose.position.y = point["positionY"]
                                        point_data.pose.position.z = 0
                                        point_data.pose.orientation.x = q[0]
                                        point_data.pose.orientation.y = q[1]
                                        point_data.pose.orientation.z = q[2]
                                        point_data.pose.orientation.w = q[3]
                                        custom_datas.append(point_data)
                    short_count = short_count + 1


                te = TargetGoalEntry()
                #rospy.loginfo("yaw: %f", temp_endbean.getPositionYaw())
                q = transformations.quaternion_from_euler(0.0, 0.0, temp_endbean.getPositionYaw()/180.0*pi)
                # 默认为高速公路 默认没有区域绑定, 需要遍历区域时候 统一设置
                # 等待点也是统一添加 
                te.point_id = int(point["pointId"])
                te.point_name = str(point["pointName"])
                te.device_id = str(point["deviceID"])
                te.floor_id = floor_id
                if point["ledWarnID"]:
                    te.led_warn_id =  int(point["ledWarnID"])
                te.map_id =mapbean.getMapId()
                te.goal.header.frame_id = "map"
                te.goal.pose.position.x = temp_endbean.getPositionX()
                te.goal.pose.position.y = temp_endbean.getPositionY()
                te.goal.pose.position.z = 0
                te.goal.pose.orientation.x = q[0]
                te.goal.pose.orientation.y = q[1]
                te.goal.pose.orientation.z = q[2]
                te.goal.pose.orientation.w = q[3]
                te.custom_path.data = custom_datas
                te.type = 10   #高速公路
                try:
                    te.lora_id = int(temp_endbean.getDeviceID())
                except Exception as e:
                    rospy.logwarn("temp_endbean.getDeviceID()  %s"%str(e))
                te.device_type = temp_endbean.getDeviceType()
                te.point_line_speed = short[1]
                te.point_angular_speed = short[1]
                te.enter_elevator = False

                # 在路线中 高速路路段补充 经过区域类型
                #if mapbean.getRegionList():
                #    for regionn in mapbean.getRegionList(): # 遍历区域 高速公路经过的区域
                #        region = Region(regionn)
                #        if short[0] in (region.getStartPointId(), region.getEndPointId(), region.getWaitPointId()):
                #            te.type = region.getType()
                #            break

                # 在路线中 给每个点 补充等待点
                if point["waitPoints"]:
                    for waitpoin in point["waitPoints"]:
                        waitpoint = Position(waitpoin)
                        wp = geometry_msgs.msg.Point()
                        wp.x = waitpoint.getX()
                        wp.y = waitpoint.getY()
                        te.waitgoal_list.append(wp)                             
                rospy.loginfo("path submitted, custom_datas size: %d", len(custom_datas)) 

                targetGoalEntryList.append(te)
                # 计算当前点离高速公路最近的点，从这里上高速
                print (custom_datas)

                currentpose = self.get_pos()     
                vx3 = currentpose.x  - targetGoalEntryList[0].custom_path.data[0].pose.position.x  # 向量v3 = current_pose - point0 
                vy3 = currentpose.y  - targetGoalEntryList[0].custom_path.data[0].pose.position.y
                if startid !=0:
                    current_bean = self.get_point_by_id(mapbean, startid)
                    vx3 = current_bean.getPositionX()  - targetGoalEntryList[0].custom_path.data[0].pose.position.x  # 向量v3 = current_pose - point0 
                    vy3 = current_bean.getPositionY() - targetGoalEntryList[0].custom_path.data[0].pose.position.y

                vx2 = targetGoalEntryList[0].custom_path.data[1].pose.position.x - targetGoalEntryList[0].custom_path.data[0].pose.position.x  #2.向量v2 = point1 - point0
                vy2 = targetGoalEntryList[0].custom_path.data[1].pose.position.y - targetGoalEntryList[0].custom_path.data[0].pose.position.y
                

                #线段 A->B 的长度平方
                v2_len_sq = vx2**2 + vy2**2  
                # 计算投影系数 t = (v3 · v2) / |v2|^2
                dot_v3_v2 = vx3 * vx2 + vy3 * vy2
                t = dot_v3_v2 / v2_len_sq

                # 3.计算点积 dot = (vx3*vx2 + vy3*vy2)
                dot = vx3 * vx2 + vy3 * vy2    
                #4.计算两个向量模
                norm1 = math.sqrt(vx3**2 + vy3**2)  
                norm2 = math.sqrt(vx2**2 + vy2**2)
                # 若其中一个向量长度为0（极少数情况下可能重合），可根据需要特殊处理
                if norm1 < 0.15 or norm2 < 0.15:
                    return (targetGoalEntryList, total_length)  # 或者直接返回 0，具体处理见业务需求
                # 5. 计算余弦值
                cos_value = dot / (norm1 * norm2) 
                # 6. 数值稳定性处理，确保 cos_value 不会超出 [-1, 1]
                if cos_value > 1.0:
                    cos_value = 1.0
                elif cos_value < -1.0:
                    cos_value = -1.0
                # 7.计算出角度 弧度制
                angle = math.acos(cos_value)
                #rospy.loginfo("1111111111111111111111  angle = %f ",angle)
                # 钝角说明距离[0]更近一点，反之距离[1]更近
                if angle <= (math.pi / 2): 
                    targetGoalEntryList[0].custom_path.data[0].pose.position.x = targetGoalEntryList[0].custom_path.data[0].pose.position.x + t * vx2
                    targetGoalEntryList[0].custom_path.data[0].pose.position.y = targetGoalEntryList[0].custom_path.data[0].pose.position.y + t * vy2
                rospy.loginfo("targetGoalEntryList[0].custom_path.data[0]: %f, %f", targetGoalEntryList[0].custom_path.data[0].pose.position.x, targetGoalEntryList[0].custom_path.data[0].pose.position.y)
                   
        return (targetGoalEntryList, total_length)        

    def get_point_by_id(self, mapbean, point_id):
        """
        根据点的 id 从 mapbean 中获取 MapsBeanPoint 对象
        :param mapbean: 地图对象
        :param point_id: 点的 id
        :return: MapsBeanPoint 对象
        """
        for po in mapbean.getPoints():
            pointb = MapsBeanPoint(po)
            if pointb.getPointId() == point_id:
                return pointb
        for syspo in mapbean.getSystemPoints():
            pointy = MapsBeanPoint(syspo)
            if pointy.getPointId() == point_id:
                return pointy
        return None
    
    def create_target_goal_entry(self, pointb, region, entry_point):
        """
        此函数用于创建并设置 TargetGoalEntry 对象的属性
        :param pointb: MapsBeanPoint 对象
        :param region: Region 对象
        :param entry_point: 入口点
        :return: TargetGoalEntry 对象和计算得到的距离
        """
        q = transformations.quaternion_from_euler(0.0, 0.0, pointb.getPositionYaw() / 180.0 * math.pi)
        tg = TargetGoalEntry()
        tg.type = region.getType()
        tg.device_type = pointb.getDeviceType()

        if pointb.getWaitPoints():
            for waitpoin in pointb.getWaitPoints():
                waitpoint = Position(waitpoin)
                wp = geometry_msgs.msg.Point()
                wp.x = waitpoint.getX()
                wp.y = waitpoint.getY()
                tg.waitgoal_list.append(wp)
        tg.device_id = pointb.getDeviceID()
        if pointb.getLedWarnID():
            tg.led_warn_id =  int(pointb.getLedWarnID())
        tg.floor_num = self.across_elevator_floor
        rospy.loginfo("type: %d, device_id: %s", tg.type, tg.device_id)
        if pointb.getDeviceID():
            try:
                tg.lora_id = int(pointb.getDeviceID())
                rospy.loginfo("lora_id: %d", tg.lora_id)
            except:
                rospy.loginfo("get lora_id except ...")
        else:
            tg.lora_id = 0
        tg.goal.header.frame_id = "map"
        tg.goal.pose.position.x = pointb.getPositionX()
        tg.goal.pose.position.y = pointb.getPositionY()
        tg.goal.pose.position.z = 0
        tg.goal.pose.orientation.x = q[0]
        tg.goal.pose.orientation.y = q[1]
        tg.goal.pose.orientation.z = q[2]
        tg.goal.pose.orientation.w = q[3]
        tg.point_id = pointb.getPointId()
        tg.point_name = pointb.getPointName()
        tg.enter_elevator = False
        distance = math.hypot(pointb.getPositionX() - entry_point.x, pointb.getPositionY() - entry_point.y)
        return tg, distance
    
    def find_closest_index_in_mpath(self, point, mpath):
        """
        找到点在 mpath 中的最近索引
        :param point: 输入的点
        :param mpath: 路径
        :return: 最近的索引
        """
        min_distance = float('inf')
        closest_index = 0
        for i, pose in enumerate(mpath):
            dx = point.x - pose.pose.position.x
            dy = point.y - pose.pose.position.y
            distance = math.sqrt(dx ** 2 + dy ** 2)
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        return closest_index
    
    def fill_targetfloor(self):
        # 先取得当前地图   
        rospy.loginfo("进入fill_targetfloor....") 
        mapSetting_bean = MapSetting(self.allMap) # 获取所有地图
        for floor in mapSetting_bean.floors:  #寻找当前点所在的楼层，所在的导航点
            floorbean = Floor(floor)         
            if floorbean.getMaps():
                for map in floorbean.getMaps():
                    mapbean = Map(map)                    
                    if int(floorbean.getDefaultmap()) ==  int(mapbean.getMapId()): # 找到当前地图
                        floor_num = mapbean.getMapFloorNum()
                        if floor_num != self.across_elevator_entry.floor_num:
                            continue

                        if mapbean.getRegionList():
                            for regionn in mapbean.getRegionList(): # 遍历区域 找到电梯
                                region = Region(regionn)
                                if region.getType() == 1:
                                    #找到中间楼层电梯   
                                    if region.getRegionProperty1():    
                                        elevator_id = region.getRegionProperty1()
                                        if int(elevator_id) == self.across_elevator_entry.elevator_id:
                                            self.start_elevator_pointid = region.getStartPointId()   # 记录出电梯的外面这个点，地图切换后可以导航
                                            self.start_innerelevator_pointid = region.getWaitPointId()
                                            self.across_elevator_entry.floor_id = floorbean.getFloorId()
                                            self.across_elevator_entry.map_id = mapbean.getMapId()
                                            rospy.loginfo("找到切换楼层的楼层号是：self.across_elevator_floor %d， floor_id: %d， start_elevator_pointid: %d", self.across_elevator_floor, self.across_elevator_entry.floor_id, self.start_elevator_pointid)  
                                            return  
    
    def split_targetGoal(self, type): # 进入检查搬运模式
        isNavigation_result = False
        distance = 0.0

        rospy.loginfo("enter spirit_targetGoal...")
        rospy.loginfo("type = %d",type)
        rospy.loginfo("self.map_status: %d",self.map_status)
        rospy.loginfo("要去的目的地信息 goal type: %d floorid = %d mapid = %d , pointid = %d, ",self.target_goal.type, self.target_goal.floor_id,self.target_goal.map_id,self.target_goal.point_id)

        if self.map_status == 0 and self.movebase_ready:
            if self.allMap is None:
                rospy.logerr("exit spirit_targetGoal. 机器人导航失败，地图文件没有加载成功！")
                return (False, 0)
            
            # 先取得当前地图    
            mapSetting_bean = MapSetting(self.allMap) # 获取所有地图
            self.is_across_floor= False # 是否跨楼层,  默认不需要
            self.is_across_map = False  # 是否跨地图,  默认不需要
            self.across_elevator_floor = 0 # 按的电梯号， 默认0
            
            # 检查是否进入电梯模式： 当前导航点楼层跟当前地图的默认楼层不一致，则认为跨楼层
            if(mapSetting_bean.getDefaultFloor() != self.target_goal.floor_id):
                self.is_across_floor = True   

            rospy.loginfo("是否跨楼层 is_across_floor: %d, current floor_id: %d, self.target_goal.floor_id: %d", self.is_across_floor, mapSetting_bean.getDefaultFloor(),self.target_goal.floor_id)         
            
            target_elevators = []               # 目标楼层电梯信息
            current_elevator = ElevatorInfo()   # 当前楼层选择正确电梯信息

            if mapSetting_bean.getFloors():  
                for floor in mapSetting_bean.getFloors():  # 寻找目标导航点所在楼层，所在电梯，所在绑定点等信息
                    floorbean = Floor(floor)
                    if int(floorbean.getFloorId()) == int(self.target_goal.floor_id):
                        for map in floorbean.getMaps():
                            mapbean = Map(map)             # 只寻找各个楼层中默认地图
                            if int(mapbean.getMapId()) == int(self.target_goal.map_id):
                                floor_num = mapbean.getMapFloorNum()
                                point = None
                                pointbean = None
                                findTargetpoint = False
                                for point in mapbean.getSystemPoints():
                                    tmpsys_pointbean = MapsBeanPoint(point)
                                    if int(tmpsys_pointbean.getPointId()) == int(self.target_goal.point_id):
                                        pointbean = tmpsys_pointbean
                                        findTargetpoint = True
                                        break
                                if findTargetpoint == False:
                                    for point in mapbean.getPoints():
                                        tmppointbean = MapsBeanPoint(point)
                                        if int(tmppointbean.getPointId()) == int(self.target_goal.point_id):
                                            pointbean = tmppointbean
                                            findTargetpoint = True
                                            break

                                if (findTargetpoint == False):
                                    rospy.loginfo("没有找到目标点，配置错误！")
                                    return (False, 0)
                                
                                rospy.loginfo("pointbean info..., findTargetpoint: %d", findTargetpoint)
                                # 找到目标导航点ID号以及目标导航点所在的楼层，所在的地图
                                # 判断目标点电梯区域
                                if self.is_across_floor:
                                    # 找到目标楼层的电梯配置
                                    target_inner_elevator_pointid = 0
                                    if mapbean.getRegionList():
                                        for regionn in mapbean.getRegionList(): # 遍历区域 找到电梯
                                            region = Region(regionn)
                                            if region.getType() == 1:
                                                # 找到电梯了
                                                target_inner_elevator_pointid = region.getWaitPointId()

                                                target_elevator = ElevatorInfo()
                                                target_elevator.point_id = int(target_inner_elevator_pointid)
                                                target_elevator.region_id = int(region.getRegionId())
                                                elevator_id = region.getRegionProperty1()  # 存放电梯编号
                                                if elevator_id:
                                                    target_elevator.elevator_id = elevator_id
                                                else:
                                                    target_elevator.elevator_id = 0
                                                try:
                                                    target_elevator.floor_num = int(floor_num)  
                                                    self.across_elevator_floor = int(floor_num)
                                                except:
                                                    target_elevator.floor_num = 0 
                                                    self.across_elevator_floor = 0
                                                    rospy.loginfo("目标点楼层号设置不正确，except...")

                                                # 根据电梯ID找到要去的电梯点
                                                for point in mapbean.getPoints():
                                                    pointbean = MapsBeanPoint(point)
                                                    if target_inner_elevator_pointid == int(pointbean.getPointId()): #找到点    
                                                        if (pointbean.getDeviceID()):                                                            
                                                            target_elevator.device_id = "513&&" + (pointbean.getDeviceID())
                                                            try:
                                                                target_elevator.lora_id = int(pointbean.getDeviceID())
                                                            except:
                                                                target_elevator.lora_id = 0
                                                                rospy.loginfo("目标楼层电梯lora_id设置不正确，except...")
                                                        
                                                rospy.loginfo("目标楼层电梯信息如下：")
                                                print (target_elevator)
                                                target_elevators.append(target_elevator) 
                                            
                                break                                
                        break                   

                for floor in mapSetting_bean.floors:  #寻找当前点所在的楼层，所在的导航点
                    floorbean = Floor(floor)
                    current_floor = mapSetting_bean.getDefaultFloor()
                    if int(current_floor) == int(floorbean.getFloorId()): # 找到当前楼层
                        rospy.loginfo("找到当前楼层 floorid = %d floorName = %s defaultmapId = %d ", floorbean.getFloorId(), floorbean.getFloorName(), floorbean.getDefaultmap())
                        if floorbean.getMaps():
                            for map in floorbean.getMaps():
                                mapbean = Map(map)
                                current_floor_num = mapbean.getMapFloorNum()
                                pointbean = None
                                if int(floorbean.getDefaultmap()) ==  int(mapbean.getMapId()): # 找到当前地图
                                    if self.is_across_floor == False:
                                        # 如果没有跨楼层，检查一下同一楼层是否跨地图
                                        if (floorbean.getDefaultmap() != self.target_goal.map_id): # 当楼层内 判断默认地图是否和目标地图不一样
                                            self.is_across_map = True  

                                    # 先判定是否跨楼层，跨楼层，找到电梯点                                    
                                    target_goal_pointid = self.target_goal.point_id    # 记录下一步要去的目标导航点，如果是多楼层，那就是当前楼层电梯点
                                    target_innerelevator_pointid = 0                   # 拆分后到达电梯内点 
                                    start_outof_elevator_id = 0                        # 出电梯电梯外点
                                    start_inner_elevator_id = 0                        # 出电梯电梯内点
                                    enter_elevator = False                             # 进电梯标记
                                    exit_elevator = False                              # 出电梯标记
                                    is_elevator_arrived = False                        # 是否已经在选中的电梯内了

                                    if self.change_map_flag:
                                        # 刚做的切换地图,一定要先出电梯
                                        self.change_map_flag = False
                                        exit_elevator = True

                                        # 缓存一下电梯数据，下面会覆盖
                                        start_outof_elevator_id = self.start_elevator_pointid
                                        start_inner_elevator_id = self.start_innerelevator_pointid  

                                    # 先判断是否在电梯中
                                    if exit_elevator == False:
                                        for regionn in mapbean.getRegionList(): # 遍历区域 找到电梯
                                            region = Region(regionn)
                                            if region.getType() == 1:  
                                                check_inner_elevator_point_id = region.getWaitPointId()  # 当前电梯电梯内导航点
                                                check_inner_elevator_pointbean = self.get_point_by_id(mapbean, check_inner_elevator_point_id) 
                                                check_elevator_distance =  math.hypot(self.current_pose.pose.position.x - check_inner_elevator_pointbean.getPositionX(), self.current_pose.pose.position.y - check_inner_elevator_pointbean.getPositionY())                                   
                                                if (check_elevator_distance < 1.0):
                                                    exit_elevator = True
                                                    start_inner_elevator_id = region.getWaitPointId()
                                                    start_outof_elevator_id = region.getStartPointId()
                                                rospy.loginfo("check_elevator_distance: %d, start_inner_elevator_id: %d, start_outof_elevator_id: %d, exit_elevator: %d", check_elevator_distance, start_outof_elevator_id, start_outof_elevator_id, exit_elevator)
                                    
                                    # 需要跨地图就遍历区域找到楼梯
                                    find_currentfloor_elevator = False
                                    if self.is_across_floor:
                                        elevator_distance = 9999.0
                                        tmp_across_elevator_id = self.across_elevator_floor
                                        if mapbean.getRegionList():
                                            find_mul_elevator = False
                                            for regionn in mapbean.getRegionList(): # 遍历区域 找到电梯
                                                region = Region(regionn)
                                                if region.getType() == 1:
                                                    # 先遍历当前楼层所有电梯，电梯去的楼层能够到达目标楼层，则选取
                                                    find_currentfloor_elevator = True                                                   
                                                    c_inner_elevator_point_id = region.getWaitPointId()  # 当前电梯电梯内导航点
                                                    c_inner_elevator_pointbean = self.get_point_by_id(mapbean, c_inner_elevator_point_id)  

                                                    try:
                                                        current_elevator.floor_num = int(current_floor_num)
                                                    except Exception as e:
                                                        rospy.logwarn("current_floor_num 楼层号设置不正确! %s"%str(e))

                                                    one_elevator_distance =  math.hypot(self.current_pose.pose.position.x - c_inner_elevator_pointbean.getPositionX(), self.current_pose.pose.position.y - c_inner_elevator_pointbean.getPositionY())
                                                    rospy.loginfo("one_elevator_distance: %f, elevator_distance: %f, c_inner_elevator_point_id: %d", one_elevator_distance, elevator_distance, c_inner_elevator_point_id)
                                                                                                        
                                                    if region.getRegionProperty1() == None:
                                                        rospy.loginfo("当前楼层可去的电梯为默认，直接到达目标层，电梯编号为0")
                                                        # 判断到电梯的距离，多部电梯，选择距离近的进入电梯                                                        
                                                        if one_elevator_distance < elevator_distance:
                                                            elevator_distance = one_elevator_distance
                                                            current_elevator.elevator_id = 0

                                                            self.across_elevator_entry.elevator_id = 0       # 填充中间楼层点数据
                                                            current_elevator.region_id = region.getRegionId()
                                                            current_elevator.device_id = c_inner_elevator_pointbean.getDeviceID()
                                                            if c_inner_elevator_pointbean.getDeviceID():
                                                                current_elevator.lora_id = int(c_inner_elevator_pointbean.getDeviceID())
                                                            else:
                                                                current_elevator.lora_id = 0   
                                                                rospy.loginfo("当前楼层电梯lora_id设置不正确，except...")                                                               
                                                                
                                                            current_elevator.status = True
                                                            target_goal_pointid = region.getStartPointId()           # 电梯外点，用来做预路线
                                                            target_innerelevator_pointid = region.getWaitPointId()   # 保留电梯内这个点，方便后续导航
                                                            current_elevator.point_id = target_innerelevator_pointid
                                                            find_mul_elevator = True
                                                            if one_elevator_distance < 1.0:
                                                                is_elevator_arrived = True
                                                                rospy.loginfo("当前小车已经在目标电梯内，直接切换地图即可...")
                                                    else:

                                                        current_elevator_id = region.getRegionProperty1()   # 电梯ID

                                                        # 根据目标电梯楼层，判断选择哪部电梯
                                                        for target_elevator in target_elevators:
                                                            if current_elevator_id == int(target_elevator.elevator_id):
                                                                rospy.loginfo("当前楼层可去的电梯，直接到达目标层") 
                                                                if one_elevator_distance < elevator_distance:
                                                                    # 直达电梯，找到了电梯口点
                                                                    elevator_distance = one_elevator_distance
                                                                    target_innerelevator_pointid = region.getWaitPointId()   # 保留电梯内这个点，方便后续导航
                                                                    current_elevator.elevator_id = current_elevator_id
                                                                    current_elevator.point_id = target_innerelevator_pointid
                                                                    current_elevator.region_id = region.getRegionId()
                                                                    current_elevator.device_id = c_inner_elevator_pointbean.getDeviceID()
                                                                    if c_inner_elevator_pointbean.getDeviceID():
                                                                        current_elevator.lora_id = int(c_inner_elevator_pointbean.getDeviceID())
                                                                    else:
                                                                        current_elevator.lora_id = 0   
                                                                        rospy.loginfo("当前楼层电梯lora_id设置不正确，except...")                                                                 
                                                                    current_elevator.status = True
                                                                    target_goal_pointid = region.getStartPointId()           # 电梯外点，用来做预路线
                                                                    target_innerelevator_pointid = region.getWaitPointId()   # 保留电梯内这个点，方便后续导航                                                                    
                                                                    find_mul_elevator = True
                                                                    if one_elevator_distance < 1.0:
                                                                        is_elevator_arrived = True
                                                                        rospy.loginfo("当前小车已经在目标电梯内，直接切换地图即可...")
                                                        
                                                        if find_mul_elevator == False:
                                                            #没有找到直达电梯，则寻找过渡电梯
                                                            for target_elevator in target_elevators:                                                           
                                                                target_floor_num = target_elevator.floor_num
                                                                rospy.loginfo("target_floor_num: %d, current_floor_num: %d", target_floor_num, current_floor_num)
                                                                rospy.loginfo("target_elevator.elevator_id: %d, current_elevator.elevator_id: %d", target_elevator.elevator_id, current_elevator_id)
                                                                # 在整个楼层里寻找过渡的中间层
                                                                tmpmapSetting_bean = MapSetting(self.allMap) # 获取所有地图
                                                                for tmpfloor in tmpmapSetting_bean.floors:  #寻找当前点所在的楼层，所在的导航点
                                                                    tmpfloorbean = Floor(tmpfloor)  
                                                                    if tmpfloorbean.getMaps():
                                                                        for tmpmap in tmpfloorbean.getMaps():
                                                                            tmpmapbean = Map(tmpmap)
                                                                            if int(tmpfloorbean.getDefaultmap()) ==  int(tmpmapbean.getMapId()): # 找到当前地图
                                                                                tmp_floor_num = tmpmapbean.getMapFloorNum()
                                                                                if int(tmp_floor_num) !=  target_floor_num and (int(tmp_floor_num) !=  current_floor_num): 
                                                                                    # 寻找中间楼层，中间楼层有多部电梯，并且覆盖了当前楼层和目标楼层电梯号
                                                                                    find_current_elevator = False
                                                                                    find_target_elevator = False                                                                                
                                                                                    if tmpmapbean.getRegionList():
                                                                                        for tmpregionn in tmpmapbean.getRegionList(): # 遍历区域 找到电梯
                                                                                            tmpregion = Region(tmpregionn)
                                                                                            if tmpregion.getType() == 1:

                                                                                                if tmpregion.getRegionProperty1():
                                                                                                    elevator_id = tmpregion.getRegionProperty1()   # 存放电梯编号
                                                                                                    if elevator_id == int(target_elevator.elevator_id):
                                                                                                        find_target_elevator = True
                                                                                                    if elevator_id == current_elevator_id:
                                                                                                        find_current_elevator = True
                                                        
                                                                                    if (find_current_elevator and find_target_elevator):
                                                                                        rospy.loginfo("在中间层找到了电梯信息：中间楼层是：%d", int(tmp_floor_num))                                                                                                        
                                                                                        if one_elevator_distance < elevator_distance:
                                                                                            elevator_distance = one_elevator_distance
                                                                                            target_innerelevator_pointid = region.getWaitPointId()   # 保留电梯内这个点，方便后续导航
                                                                                            current_elevator.point_id = target_innerelevator_pointid
                                                                                            current_elevator.region_id = region.getEndPointId()
                                                                                            current_elevator.device_id = c_inner_elevator_pointbean.getDeviceID()
                                                                                            if c_inner_elevator_pointbean.getDeviceID():
                                                                                                current_elevator.lora_id = int(c_inner_elevator_pointbean.getDeviceID())
                                                                                            else:
                                                                                                current_elevator.lora_id = 0
                                                                                                rospy.loginfo("当前楼层电梯lora_id设置不正确，except...")     

                                                                                            current_elevator.floor_num = tmp_floor_num
                                                                                            current_elevator.elevator_id = current_elevator_id
                                                                                            current_elevator.status = True
                                                                                            target_goal_pointid = region.getStartPointId()    #如果有电梯就到达电梯外这个点
                                                                                            
                                                                                            find_mul_elevator = True                                                                            
                                                                                            tmp_across_elevator_id = tmp_floor_num 
                                                                                            if one_elevator_distance < 1.0:
                                                                                                is_elevator_arrived = True
                                                                                                rospy.loginfo("当前小车已经在目标电梯内，直接切换地图即可...")
                                                                                            
                                            if find_mul_elevator == False:
                                                rospy.logerr("split_targetGoal 找不到合适的电梯，无法跨地图导航")   
                                                return (False, 0)
                                            else:
                                                self.across_elevator_floor = tmp_across_elevator_id
                                                enter_elevator = True
                                                rospy.loginfo("当前楼层电梯信息如下：")
                                                print (current_elevator)
                                                rospy.loginfo("new set  self.across_elevator_floor: %d",  self.across_elevator_floor)

                                            # 填充中间楼层点数据
                                            self.across_elevator_entry.lora_id = current_elevator.lora_id
                                            self.across_elevator_entry.floor_num = self.across_elevator_floor
                                            self.across_elevator_entry.device_id = current_elevator.device_id
                                            self.across_elevator_entry.elevator_id = current_elevator.elevator_id
                                            self.fill_targetfloor()
                                            if is_elevator_arrived:
                                                # 已经在电梯内，直接切换地图即可
                                                self.is_across_floor = False                
                                                addoneGoal = TargetGoalEntry()
                                                addoneGoal.floor_id = int(self.across_elevator_entry.floor_id)
                                                addoneGoal.map_id = int(self.across_elevator_entry.map_id)
                                                addoneGoal.point_id = int(self.across_elevator_entry.point_id)
                                                addoneGoal.type = 25

                                                self.target_goal_lock.acquire()
                                                self.subtask_targetgoals.insert(0, addoneGoal)
                                                self.target_goal_lock.release()
                                                if self.task_executing == 0:
                                                    self.task_executing = 1
                                                self.get_nexttask = True
                                                self.task_runs_status = False
                                                return (True, 0)
                                                            
                                        else:
                                            rospy.logerr("exit split_targetGoal 找不到当前楼层区域，无法跨地图导航")   
                                            return (False, 0)  
                                    else:
                                        # 不是跨楼层，但目标点在电梯内
                                        if mapbean.getRegionList():
                                            for regionn in mapbean.getRegionList(): # 遍历区域 找到电梯
                                                region = Region(regionn)
                                                if region.getType() == 1:
                                                    # 先遍历当前楼层所有电梯，电梯去的楼层能够到达目标楼层，则选取                                              
                                                    c_inner_elevator_point_id = region.getWaitPointId()  # 当前电梯电梯内导航点
                                                    if c_inner_elevator_point_id == int(self.target_goal.point_id):
                                                        target_innerelevator_pointid = c_inner_elevator_point_id
                                                        target_goal_pointid = region.getStartPointId()
                                                        enter_elevator = True 
                                                        rospy.loginfo("不是跨楼层，但目标点在电梯内, c_inner_elevator_point_id: %d, target_goal_pointid: %d", c_inner_elevator_point_id, target_goal_pointid)
                                                                                                        

                                    if self.is_across_floor and (not find_currentfloor_elevator): # 需要跨地图 并且找不到电梯
                                        rospy.logerr("exit split_targetGoal, 找不到电梯点，无法跨地图导航")
                                        return (False, 0) 

                                    # 找到地图点或 开启电梯导航
                                    findCurrentpoint = False
                                    if mapbean.getSystemPoints():
                                        for point in mapbean.getSystemPoints():
                                            syspointbean = MapsBeanPoint(point)  
                                            if target_goal_pointid == int(syspointbean.getPointId()): 
                                                pointbean = syspointbean 
                                                findCurrentpoint = True
                                                break                                           
                                    if findCurrentpoint == False:
                                        if mapbean.getPoints():
                                            for point in mapbean.getPoints():
                                                tmppointbean = MapsBeanPoint(point)  
                                                if target_goal_pointid == int(tmppointbean.getPointId()): 
                                                    pointbean = tmppointbean 
                                                    findCurrentpoint = True
                                                    break
                                    rospy.loginfo("findCurrentpoint: %d", findCurrentpoint)
                                    # 找到目标导航点，如果不是跨楼层，则为目标点，如果是跨楼层，则是电梯外这个点
                                    target_innerelevator_pointbean = pointbean
                                    start_outelevator_pointbean = pointbean
                                    start_innerelevator_pointbean = pointbean

                                    if (target_innerelevator_pointid !=0):
                                        target_innerelevator_pointbean = self.get_point_by_id(mapbean, target_innerelevator_pointid) 
                                    
                                    if (start_outof_elevator_id !=0):
                                        start_outelevator_pointbean = self.get_point_by_id(mapbean, start_outof_elevator_id) 
                                    if (start_inner_elevator_id!=0):
                                        start_innerelevator_pointbean = self.get_point_by_id(mapbean, start_inner_elevator_id) 
                                    
                                    rospy.loginfo("start_outof_elevator_id: %d start_innerelevator_pointid: %d, mapbenid: %d", start_outof_elevator_id, start_inner_elevator_id, mapbean.getMapId())
                                    
                                    #找到当楼层要去的导航点
                                    rospy.loginfo("找到导航点 %d, 导航点名： %s 准备预路线。。。", pointbean.getPointId(), pointbean.getPointName())


                                    tl  = TargetGoalEntryList()
                                    targetgoals = TargetGoalEntryList()

                                    # 先做路径分析，然后再做高速公路判断
                                    mpath = []
                                    mpath_length = 0
                                    try:
                                        if exit_elevator:
                                            (mpath, mpath_length) = self.getMovebasePlan(start_outelevator_pointbean, pointbean)
                                        else:    
                                            (mpath, mpath_length) = self.getMovebasePlan(None, pointbean)
                                    except:
                                        rospy.loginfo("exit split_targetGoal getMovebasePlan fail")
                                        return (False, 0)                                                
                                    
                                    send_movebase_points = []
                                    robot_in_region_flag = False
                                    if mpath:
                                        rospy.loginfo( "初步计算路线成功,初步计算路径长度： %f 米",mpath_length)
                                    else:
                                        # 预路线规划失败，重新规划
                                        rospy.loginfo( "exit split_targetGoal 预路线规划失败，重新再规划 %f",mpath_length)
                                        return (False, 0)
                                    
                                    region_entries = []  # 存储 (index, tg)
                                    if mpath and mapbean.getRegionList(): # 有路线有区域才重构                                                    
                                        for polygon in mapbean.getRegionList(): # 遍历区域 找到穿过区域的点并重组 
                                            # 这里 算法 只计算 起点到穿入区域点 和  穿入区域点到穿出区域点 所以缺 最后一个区域出到目标点
                                            # 查找与当前路径相交的区域                                                         
                                            intersection_points = self.find_intersection_points(mpath, polygon) 
                                            rospy.loginfo( "进入intersection_points 检测到路线有穿过区域点数 %d",len(intersection_points))
                                            if len(intersection_points)==1: # ps: 遇到穿过区域的是一个点，这里默认以为机器当前位置为区域内
                                                robot_in_region_flag = True

                                            if intersection_points:  # 判断是否穿过该区域
                                                rospy.loginfo( "进入intersection_points 检测到路线有穿过区域 ")
                                                entry_point = intersection_points[0][0]   # 入口  
                                                exit_point = intersection_points[-1][1]   # 出口
                                                if (len(intersection_points) > 1):
                                                    rospy.loginfo( "穿过区域2个点信息： entry_point (%f,%f), exit_point (%f,%f) ",entry_point.x,entry_point.y,exit_point.x,exit_point.y)
                                                elif (len(intersection_points) == 1):
                                                    rospy.loginfo( "穿过区域1个点信息： entry_point (%f,%f) ",entry_point.x,entry_point.y)

                                                # 根据区域的出入口选择最近的入口
                                                region = Region(polygon)
                                                if region.getType() == 1: # 在电梯内，需要进行出电梯
                                                    rospy.logerr("电梯这块目前进不来，不用考虑")

                                                else :    
                                                    # 先找到区域的出入口点 对应的是路线入口还是出口
                                                    # 如果 distance_a< b 那么 入口是 区域入口
                                                    # 如果 distance_a> b 那么 出口是 区域入口
                                                    tga, distance_a = self.create_target_goal_entry(self.get_point_by_id(mapbean, region.getStartPointId()), region, entry_point)
                                                    tgb, distance_b = self.create_target_goal_entry(self.get_point_by_id(mapbean, region.getStartPointId()), region, exit_point)
                                                    
                                                    tg1, distance1 = self.create_target_goal_entry(self.get_point_by_id(mapbean, region.getStartPointId()), region, entry_point)
                                                    tg2, distance2 = self.create_target_goal_entry(self.get_point_by_id(mapbean, region.getEndPointId()), region, entry_point)
                                                    rospy.loginfo("split_targetGoal   distance1 %f, distance2 %f", distance1, distance2)
                                                    if tg1.device_id == "" and tg2.device_id!= "":
                                                        tg1.device_id = tg2.device_id
                                                        tg1.lora_id = tg2.lora_id
                                                        tg1.device_type = tg2.device_type
                                                    elif tg2.device_id == "" and tg1.device_id!= "":
                                                        tg2.device_id = tg1.device_id
                                                        tg2.lora_id = tg1.lora_id
                                                        tg2.device_type = tg1.device_type
                                                    
                                                    # 找到 entry_point 和 exit_point 在 mpath 中的最近索引
                                                    entry_index = self.find_closest_index_in_mpath(entry_point, mpath)
                                                    exit_index = self.find_closest_index_in_mpath(exit_point, mpath)

                                                    # 看哪个点最先穿过区域 就先加哪个数据
                                                    if distance_a <= distance_b:
                                                        region_entries.append((entry_index, tg1))
                                                        region_entries.append((exit_index, tg2))                                                                    
                                                    else:
                                                        region_entries.append((entry_index, tg2))
                                                        region_entries.append((exit_index, tg1))  

                                        # 按照 entry_index 对区域进行排序
                                        region_entries.sort(key=lambda x: x[0])

                                    # 增加当前点进入队列
                                    firstte = TargetGoalEntry()
                                    q = transformations.quaternion_from_euler(0.0, 0.0, 1.0/180.0*pi)
                                    firstte.point_id = 0
                                    firstte.point_name = "robot_pose"
                                    firstte.floor_id = current_floor
                                    firstte.map_id = mapbean.getMapId()
                                    firstte.device_id = ""
                                    firstte.goal.header.frame_id = "map"
                                    firstte.goal.pose.position.x = self.current_pose.pose.position.x
                                    firstte.goal.pose.position.y = self.current_pose.pose.position.y
                                    firstte.goal.pose.position.z = 0
                                    firstte.goal.pose.orientation.x = q[0]
                                    firstte.goal.pose.orientation.y = q[1]
                                    firstte.goal.pose.orientation.z = q[2]
                                    firstte.goal.pose.orientation.w = q[3]
                                    firstte.type = 0 
                                    firstte.enter_elevator = False 

                                    send_movebase_points.append(firstte)
                                    if len(region_entries)> 0:
                                        for index, tg in region_entries:
                                            send_movebase_points.append(tg)

                                    te = TargetGoalEntry()
                                    q = transformations.quaternion_from_euler(0.0, 0.0, pointbean.getPositionYaw()/180.0*pi)
                                    te.point_id = pointbean.getPointId()
                                    te.point_name = str(pointbean.getPointName())
                                    te.floor_id = current_floor
                                    te.map_id = mapbean.getMapId()
                                    te.device_id = pointbean.getDeviceID()
                                    te.goal.header.frame_id = "map"
                                    te.goal.pose.position.x = pointbean.getPositionX()
                                    te.goal.pose.position.y = pointbean.getPositionY()
                                    te.goal.pose.position.z = 0
                                    te.goal.pose.orientation.x = q[0]
                                    te.goal.pose.orientation.y = q[1]
                                    te.goal.pose.orientation.z = q[2]
                                    te.goal.pose.orientation.w = q[3]
                                    te.type = 0 
                                    te.enter_elevator = False
                                    send_movebase_points.append(te)                                                    
                                    
                                    rospy.loginfo("send_movebase_points: ")
                                    print (send_movebase_points)                                               


                                    # 对每一个预路线分析的导航点进行高速公路分析，判断是否走高速
                                    isGaosugonglu = False 
                                    check_point_gaosugonglu = 0
                                    single_count = 1
                                    start_point_id = 0
                                    if robot_in_region_flag:
                                        # 在区域内，需要先出区域                                                    
                                        for mid_point in send_movebase_points:
                                            rospy.loginfo("single_count: %d", single_count)
                                            if single_count == 1:
                                                continue
                                            if single_count == 2:
                                                #增加一个区域导航出去
                                                targetgoals.goal_point_list.append(mid_point)

                                            if (int(single_count) % 2) == 0:
                                                single_count = single_count + 1
                                                start_point_id = mid_point.point_id
                                                start_point = mid_point
                                                rospy.loginfo("skipped!")
                                                continue
                                            single_count = single_count + 1

                                            mid_point_id = mid_point.point_id
                                            rospy.loginfo("start_point_id: %d mid_point_id: %d", start_point_id, mid_point_id) 
                                                                                        
                                            (gplan_pathpoints, gplan_length) = self.check_gaosugonglu(current_floor, map, start_point_id, mid_point_id)                                              

                                            if gplan_pathpoints and len(gplan_pathpoints) > 0:
                                                # 有高速公路 直接走，不需要计算自由路线    
                                                if check_point_gaosugonglu == 0:                                                
                                                    targetgoals.goal_point_list = gplan_pathpoints
                                                else:
                                                    targetgoals.goal_point_list.append(start_point)
                                                    for gpoint in gplan_pathpoints:
                                                        targetgoals.goal_point_list.append(gpoint)  
                                                targetgoals.goal_point_list[-1].lora_id = mid_point.lora_id
                                                                                                                        
                                                rospy.loginfo("高速, 机器人检查导航到目的地: %s", pointbean.getPointName()) 
                                                isGaosugonglu = True   
                                            else:
                                                targetgoals.goal_point_list.append(start_point)
                                                # 把mid_point的type设定为直接导航
                                                mid_point.type = 0
                                                targetgoals.goal_point_list.append(mid_point)  
                                            check_point_gaosugonglu = check_point_gaosugonglu + 1    
                                    else:
                                        for mid_point in send_movebase_points:
                                            rospy.loginfo("single_count: %d", single_count)
                                            if (int(single_count) % 2) == 1:
                                                single_count = single_count + 1
                                                start_point_id = mid_point.point_id
                                                start_point = mid_point
                                                rospy.loginfo("skipped!")
                                                continue
                                            single_count = single_count + 1

                                            mid_point_id = mid_point.point_id
                                            rospy.loginfo("start_point_id: %d mid_point_id: %d", start_point_id, mid_point_id) 
                                                                                        
                                            (gplan_pathpoints, gplan_length) = self.check_gaosugonglu(current_floor, map, start_point_id, mid_point_id)                                              

                                            if gplan_pathpoints and len(gplan_pathpoints) > 0:
                                                # 有高速公路 直接走，不需要计算自由路线    
                                                if check_point_gaosugonglu == 0:                                                
                                                    targetgoals.goal_point_list = gplan_pathpoints
                                                else:
                                                    targetgoals.goal_point_list.append(start_point)
                                                    for gpoint in gplan_pathpoints:
                                                        targetgoals.goal_point_list.append(gpoint)                                                            
                                                rospy.loginfo("高速, 机器人检查导航到目的地: %s", str(pointbean.getPointName())) 
                                                isGaosugonglu = True   
                                            else:
                                                targetgoals.goal_point_list.append(start_point)
                                                # 把mid_point的type设定为直接导航
                                                mid_point.type = 0
                                                targetgoals.goal_point_list.append(mid_point)  
                                            check_point_gaosugonglu = check_point_gaosugonglu + 1    
                                    rospy.loginfo("check_point_gaosugonglu: %d, isGaosugonglu: %d, targetgoals.goal_point_list size: %d", check_point_gaosugonglu, isGaosugonglu, len(targetgoals.goal_point_list))                                      

                                    # 如果是路线经过高速公路，则最后一个点必定是高速公路追加处理过的，这里不能追加判断，
                                    # 因为 有过高速路，自由路线目标点 pointbean 会变成 临时高速公路起点，
                                    # 哪怕到高速的自由路线穿过了区域，穿过区域后不追加目标点，高速公路起点，会与之衔接上，所以这种情况不需要考虑追加  
                                    # 结束 给 mpath 重新计算路线 并且添加区域节点属性
                                    if isGaosugonglu: #如果是高速公路过来的 那么把这段自由路线要优先走
                                        rospy.loginfo("split_targetGoal 正在填充上高速路")                                                   
                                        tl.goal_point_list = (targetgoals.goal_point_list)
                                        # 如果是高速路导航 那么长度需要追加 到高速路起点的距离
                                        
                                    else:                                    
                                        tl.goal_point_list = send_movebase_points
                                        # 如果是普通路线则 直接是
                                        distance = mpath_length


                                    if enter_elevator:                                                   
                                        # 如果是进电梯，增加电梯点
                                        te = TargetGoalEntry()
                                        q = transformations.quaternion_from_euler(0.0, 0.0, target_innerelevator_pointbean.getPositionYaw()/180.0*pi)
                                        te.point_id = target_innerelevator_pointbean.getPointId()
                                        te.point_name = str(target_innerelevator_pointbean.getPointName())
                                        te.floor_id = current_floor
                                        te.map_id = mapbean.getMapId()
                                        te.device_id = "513&&" + target_innerelevator_pointbean.getDeviceID()
                                        te.floor_num = current_floor_num
                                        if target_innerelevator_pointbean.getDeviceID():
                                            te.lora_id = int(target_innerelevator_pointbean.getDeviceID())
                                        te.goal.header.frame_id = "map"
                                        te.goal.pose.position.x = target_innerelevator_pointbean.getPositionX()
                                        te.goal.pose.position.y = target_innerelevator_pointbean.getPositionY()
                                        te.goal.pose.position.z = 0
                                        te.goal.pose.orientation.x = q[0]
                                        te.goal.pose.orientation.y = q[1]
                                        te.goal.pose.orientation.z = q[2]
                                        te.goal.pose.orientation.w = q[3]
                                        te.type = 1 
                                        
                                        te.enter_elevator = True
                                        # 在路线中 给目标点 补充等待点
                                        if point["waitPoints"]:
                                            for waitpoin in point["waitPoints"]:
                                                waitpoint = Position(waitpoin)
                                                wp = geometry_msgs.msg.Point()
                                                wp.x = waitpoint.getX()
                                                wp.y = waitpoint.getY()
                                                te.waitgoal_list.append(wp) 
                                        #tl.enter_elevator = True
                                        rospy.loginfo("进入电梯信息te： ")
                                        print (te)
                                        tl.goal_point_list.append(te)    

                                    if exit_elevator:
                                        # 如果是出电梯，增加电梯点到第一个
                                        te = TargetGoalEntry()
                                        q = transformations.quaternion_from_euler(0.0, 0.0, start_outelevator_pointbean.getPositionYaw()/180.0*pi)
                                        te.point_id = start_outelevator_pointbean.getPointId()
                                        te.point_name = str(start_outelevator_pointbean.getPointName())
                                        te.floor_id = current_floor
                                        te.map_id = mapbean.getMapId()
                                        te.device_id = "513&&" + start_outelevator_pointbean.getDeviceID()
                                        te.floor_num = current_floor_num
                                        if start_innerelevator_pointbean.getDeviceID():
                                            te.lora_id = int(start_innerelevator_pointbean.getDeviceID())
                                        te.goal.header.frame_id = "map"
                                        te.goal.pose.position.x = start_outelevator_pointbean.getPositionX()
                                        te.goal.pose.position.y = start_outelevator_pointbean.getPositionY()
                                        te.goal.pose.position.z = 0
                                        te.goal.pose.orientation.x = q[0]
                                        te.goal.pose.orientation.y = q[1]
                                        te.goal.pose.orientation.z = q[2]
                                        te.goal.pose.orientation.w = q[3]
                                        te.type = 1 

                                        te.enter_elevator = False

                                        #tl.enter_elevator = True
                                        rospy.loginfo("出电梯信息te： ")
                                        print (te)
                                        tl.goal_point_list.insert(0, te)                                                    
                                    
                                    if type == 1:
                                        isNavigation_result = True
                                        rospy.loginfo("split_targetGoal 只计算距离不导航: %s, 长度 %f ", str(tl.goal_point_list[-1].point_name), distance)
                                        return (True, distance)

                                    # 发布导航点
                                    self.target_goal_list_pub.publish(tl)  
                                    rospy.loginfo("打印t1信息如下：")
                                    print (tl)  

                                    rospy.loginfo("tl.goal_point_list, size: %d", len(tl.goal_point_list))
                                    isNavigation_result = True
                                    
                                    
                                    break # 结束地图遍历
                        break # 结束楼层遍历
            else:
                rospy.loginfo("split_targetGoal getfloors fail!")
        rospy.loginfo("split_targetGoal end status: %d distance: %f",isNavigation_result, distance)
        return (isNavigation_result, distance)

    def CurrentJackingStatuslCallback(self,msg):

        self.current_jacking_status=msg.data

    def scan_CB(self, msg):
        # 复位scan状态标志
        self.scan_status = 0
#         self.current_scan = msg
        if self.scan_ready:
            return
        self.scan_ready = True
        rospy.loginfo("侦测到激光已经启动！")
        # 激光已经起来
        #self.AndroidmsgReportPub.publish(14)
        self.androidLidarStatus_pub.publish(14)
        self.AndroidAlarm.androidmsg_lidarstatus = 14
        self.check_map(True)  # scan_ready
        self.check_navigation_isOK()

    def odom_CB(self, msg):
        # 复位odom状态标志
        self.driver_status = 0
        if self.odom_ready:
            return

        self.odom_ready = True
        rospy.loginfo("侦测到里程计已经启动！")
        # 里程计已经起来
        #self.AndroidmsgReportPub.publish(20)
        self.androidOdomStatus_pub.publish(20)
        self.AndroidAlarm.androidmsg_odomstatus = 20
        self.check_map(True)  # odom_ready
        self.check_navigation_isOK()
        try:
            self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(3.0))

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logerr("Map manager : initialize odom fail, TF can't create.")

    def localmap_CB(self, msg):
        if self.localmap_ready:
            return

        self.sendmapdata.header = msg.header
        self.sendmapdata.info = msg.info
        self.sendmapdata.data = msg.data
        self.mapupdate_pub.publish(self.sendmapdata)

        self.localmap_ready = True
        rospy.loginfo("mapserver读取地图数据成功！")

    def map_CB(self, msg):
#         self.current_map = msg
        if self.map_ready:
            return

        self.map_ready = True
        rospy.loginfo("侦测到地图已经启动！")
        # 地图已经起来
        #self.AndroidmsgReportPub.publish(60)
        self.androidMapStatus_pub.publish(60)
        self.AndroidAlarm.androidmsg_mapstatus = 60
        self.wall_update_pub.publish(0) #map update
        self.check_navigation_isOK()
        try:
            self.tf_listener.waitForTransform(self.map_frame, self.base_frame, rospy.Time(), rospy.Duration(3.0))

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logerr("map_CB异常，从map到base_linkTF没有找到！！！")

    def imu_CB(self, msg):
        # 复位imu状态标志
        self.imu_status = 0
        if self.imu_ready:
            # check IMU
            return
        self.imu_ready = True
        rospy.loginfo("侦测到IMU已经启动！")
        # IMU已经起来
        #self.AndroidmsgReportPub.publish(16)
        self.androidImuStatus_pub.publish(16)
        self.AndroidAlarm.androidmsg_imustatus = 16
        self.check_map(True)  # imu_ready
        self.check_navigation_isOK()
    
    def imu_angle_CB(self, msg):
        self.imu_angle.x = msg.x
        self.imu_angle.y = msg.y
        self.imu_angle.z = msg.z

    def navicmd_CB(self, msg):        
        if self.mode == 1:
            return
        
        if self.laser_switch==1:
            return

        self.laser_switch = 1
        self.laserswitch_pub.publish(1)
        rospy.loginfo("监测到机器人运动，驱动底盘开启激光！")
    
    def charge_fail(self, msg):
        # 充电失败
        # result
        # 0 成功
        # 2 失败
        if msg.data == 5:
            self.mode = 2
            self.waitfornavi = False
            self.motionmode_pub.publish(2)
            rospy.loginfo("机器人充电失败！")

            if self.start_chargecheck == False:
                self.lowpowerExecute = True
                self.lowpowerChargepoint_add = False
                self.task_runs_status = False
            return 


    def chargepoint_set(self, msg):
        # 修改充电桩
        # result 0 添加成功

        rospy.loginfo("chargepoint_set (%f, %f, %f)-(%f,%f,%f,%f)", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)

        if self.fix_chargepoint_flag:
            return 

        rospy.loginfo("开始修正充电桩位置。。。")    
        if not self.map_ready:
            rospy.logwarn("map tf can not find")
            return 

        local_time = self.getId()
        local_time = int(local_time)

        (euler_r,euler_p,euler_y) = transformations.euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
        poseset = Point32()
        poseset.x = msg.pose.position.x
        poseset.y = msg.pose.position.y
        poseset.z = euler_y / pi * 180

        if self.map_status == 0:
            point = SystemPoint()
            point.setPointType(4)
            point.setPointId(local_time)
            point.setPositionX(poseset.x)
            point.setPositionY(poseset.y)
            point.setPositionYaw(poseset.z)
            point.setPointName("桩位置")

            rospy.loginfo("添加桩位置成功， 位置：%f, %f, %f", poseset.x, poseset.y, poseset.z)
            self.build_systempoints.append(point)

        self.check_map()  # point set

        if self.allMap is None:
            return 

        mapSetting = MapSetting(self.allMap)

        # 逐条取出保存，方便后面
        floors = []
        floorindex = -1
        isaddfloorindex = True
        maps = []

        for fll in mapSetting.getFloors():
            if isaddfloorindex:
                floorindex= floorindex+1
            floor = Floor(fll)
            floors.append(floor)
            if (floor.getFloorId() == mapSetting.getDefaultFloor()):
                # 找到默认楼层, 结束记录下标, 只记录默认地图的下标
                isaddfloorindex = False

            
                mapindex = -1
                isaddindex = True
                for uu in floor.getMaps():
                    if (isaddindex):
                        mapindex = mapindex + 1
                    map = Map(uu)
                    maps.append(map)
                    # 逐条取出保存
                    if map.getMapId() == floor.getDefaultmap():
                        isaddindex = False
                rospy.loginfo("chargepoint_set map size = %d, defatmapId = %d", len(maps), mapindex)

                points = []
                if maps[mapindex].getPoints():
                    for pointname in maps[mapindex].getPoints():
                        point = MapsBeanPoint(pointname)
                        points.append(point)

                systemPoints = []
                if maps[mapindex].getSystemPoints():
                    for pointname in maps[mapindex].getSystemPoints():
                        systempoint = MapsBeanPoint(pointname)
                        systemPoints.append(systempoint)
                    
                    # 新增加桩位置点
                    systempoint = MapsBeanPoint()
                    systempoint.setPointId(local_time)
                    systempoint.setPositionX(poseset.x)
                    systempoint.setPositionY(poseset.y)
                    systempoint.setPositionYaw(poseset.z)
                    systempoint.setPointName("桩位置")
                    systemPoints.append(systempoint)
                # 回填map的points
                maps[mapindex].setPoints(points)
                maps[mapindex].setSystemPoints(systemPoints)



        floors[floorindex].setMaps(maps)
        mapSetting.setFloors(floors)
        self.write_json(mapSetting, spirit_map_url)  # point set
        self.wall_update_pub.publish(0)

        self.check_map() #map null point add end
        self.fix_chargepoint_flag = True
        rospy.loginfo("chargepoint_set success!")
        return 

    def initialpose_set(self):
        rospy.loginfo("在建图状态进入充电模式，记录初始位置值，写入systempoints中！")

        if not self.map_ready:
            rospy.logwarn("地图没有加载成功，退出吸入systempoints！")
            return

        local_time = self.getId()
        local_time = int(local_time)
        init_local_time = int(local_time)
        charge_local_time = int(local_time)+1

        poseset = self.get_pos()

        if self.map_status == 1 or self.map_status == 2:
            if self.build_systempoints:
                local_points = []
                InitialpointFind = False  # type: bool
                chargepointFind = False

                for point1 in self.build_systempoints:
                    # point1 = SystemPoint(poin)
                    if (point1.getPointName() == "初始点"):
                        InitialpointFind = True
                        point1.setPointType(2)
                        point1.setPositionX(poseset.x)
                        point1.setPositionY(poseset.y)
                        point1.setPositionYaw(poseset.z)
                        rospy.loginfo("机器人在充电，修改初始点位置为: %f, %f, %f", point1.getPositionX(), point1.getPositionY(), point1.getPositionYaw())
                    elif (point1.getPointName() == "充电桩"):
                        chargepointFind = True

                    local_points.append(point1)
                if InitialpointFind:
                    if (chargepointFind == False):
                        point = SystemPoint()
                        point.setPointType(1)
                        point.setPointId(charge_local_time)
                        point.setPositionYaw(poseset.z + 180.0)
                        if poseset.z < 0:
                            poseset.z = poseset.z + 360
                        point.setPositionX(poseset.x + 0.3*cos(poseset.z *pi /180.0))
                        point.setPositionY(poseset.y + 0.3*sin(poseset.z *pi /180.0))
                        point.setPointName("充电桩")
                        self.fix_chargepoint_flag = False
                        rospy.loginfo("机器人在充电，添加充电桩位置为: %f, %f, %f", point.getPositionX(), point.getPositionY(), point.getPositionYaw())
                        local_points.append(point)
                    self.build_systempoints = local_points
                    self.wall_update_pub.publish(0)
                    return
                else:
                    point = SystemPoint()
                    point.setPointType(2)
                    point.setPointId(init_local_time)
                    point.setPositionX(poseset.x)
                    point.setPositionY(poseset.y)
                    point.setPositionYaw(poseset.z)
                    point.setPointName("初始点")
                    rospy.loginfo("机器人在充电，添加初始点位置为: %f, %f, %f", point.getPositionX(), point.getPositionY(), point.getPositionYaw())
                    local_points.append(point)
                    if (chargepointFind == False):
                        point = SystemPoint()
                        point.setPointType(1)
                        point.setPointId(charge_local_time)
                        point.setPositionYaw(poseset.z + 180.0)
                        if poseset.z < 0:
                            poseset.z = poseset.z + 360
                        point.setPositionX(poseset.x + 0.3*cos(poseset.z *pi /180.0))
                        point.setPositionY(poseset.y + 0.3*sin(poseset.z *pi /180.0))
                        point.setPointName("充电桩")
                        self.fix_chargepoint_flag = False
                        rospy.loginfo("机器人在充电，添加充电桩位置为: %f, %f, %f", point.getPositionX(), point.getPositionY(), point.getPositionYaw())
                        local_points.append(point)
                    self.build_systempoints = local_points
                    self.wall_update_pub.publish(0)
                    return

            point = SystemPoint()
            point.setPointType(1)
            point.setPointId(init_local_time)
            point.setPositionX(poseset.x)
            point.setPositionY(poseset.y)
            point.setPositionYaw(poseset.z)
            point.setPointName("初始点")
            rospy.loginfo("机器人在充电，保存初始点位置为: %f, %f, %f", point.getPositionX(), point.getPositionY(), point.getPositionYaw())
            self.build_systempoints.append(point)

            point = SystemPoint()
            point.setPointType(1)
            point.setPointId(charge_local_time)
            point.setPositionYaw(poseset.z + 180.0)
            if poseset.z < 0:
                poseset.z = poseset.z + 360
            point.setPositionX(poseset.x + 0.3*cos(poseset.z *pi /180.0))
            point.setPositionY(poseset.y + 0.3*sin(poseset.z *pi /180.0))
            point.setPointName("充电桩")
            self.fix_chargepoint_flag = False
            rospy.loginfo("机器人在充电，保存充电桩位置为: %f, %f, %f", point.getPositionX(), point.getPositionY(), point.getPositionYaw())
            self.build_systempoints.append(point)
            self.wall_update_pub.publish(0)
            return

    def relocation_set(self, relocationType=0):
        if (relocationType == 1):
            if (self.map_status == 0):
                chargeFloorId = 0
                rospy.loginfo("在加载地图模式下，进入充电模式，正在进行自动重定位...")
                # 判断楼层是否一致
                isSameFloor = False
                if self.allMap:
                    mapSetting = MapSetting(self.allMap)                    
                    if mapSetting.getChargingPrioritys():
                        chargeFloorId = mapSetting.getChargingPrioritys()[0]
                        rospy.loginfo("findChargeTargetGoalEntry  chargeFloorId = %d",chargeFloorId)
                    
                    # 如果没有找到跨楼层充电，则在本楼层充电
                    if chargeFloorId == 0:
                        chargeFloorId = mapSetting.getDefaultFloor()

                    if mapSetting.getDefaultFloor() == chargeFloorId:
                        isSameFloor = True

                if isSameFloor:
                    # 重定位，如果机器跟充电桩在同一个楼层
                    for point in self.local_systempoint_list:
                        if str(point.point_name) == "初始点":
                            rospy.loginfo("在同一楼层，机器人重定位到初始点，位置为: %f, %f, %f", point.x, point.y, point.z)

                            pose = PoseWithCovarianceStamped()
                            pose.header.stamp = rospy.Time.now()
                            pose.header.frame_id = 'map'
                            pose.pose.pose.position.x = point.x
                            pose.pose.pose.position.y = point.y

                            rot = transformations.quaternion_from_euler(0.0, 0.0, point.z / 180.0 * pi)
                            pose.pose.pose.orientation.x = rot[0]
                            pose.pose.pose.orientation.y = rot[1]
                            pose.pose.pose.orientation.z = rot[2]
                            pose.pose.pose.orientation.w = rot[3]
                            self.initpose_pub.publish(pose)
                else:
                    # 不在同一楼层，先切换地图到充电桩楼层，然后再做重定位
                    addoneGoal = TargetGoalEntry()
                    addoneGoal.floor_id = chargeFloorId
                    addoneGoal.type = 26

                    self.target_goal_lock.acquire()
                    self.subtask_targetgoals.insert(0, addoneGoal)
                    self.target_goal_lock.release()
                    if self.task_executing == 0:
                        self.task_executing = 1
                    self.get_nexttask = True
                    self.task_runs_status = False
                    rospy.loginfo("充电桩不在同一个楼层，开始切换地图")

        else:
            if (self.map_status == 0):
                rospy.loginfo("在加载地图模式下，正在进行手动重定位...")
                self.initpose_pub.publish(self.initial_pose)
        rospy.loginfo("在导航状态进入重定位成功，发送新位置给cartographer成功！")

    def pubPointsPose(self):
        points_posearray = PointPoseArray()
        points_posearray.header.stamp = rospy.Time.now()
        points_posearray.header.frame_id = "/map"
        for point in self.local_point_list:
            # Add the map info to our result list.
            poseadd = PointPoseListEntry()
            poseadd.name = point.point_name
            poseadd.pose.position.x = point.x
            poseadd.pose.position.y = point.y
            q = transformations.quaternion_from_euler(0.0, 0.0, point.z / 180.0 * pi)
            poseadd.pose.orientation.x = q[0]
            poseadd.pose.orientation.y = q[1]
            poseadd.pose.orientation.z = q[2]
            poseadd.pose.orientation.w = q[3]
            points_posearray.poseslist.append(poseadd)
        self.PointsPose_Pub.publish(points_posearray)
    
    def changeTargetMap(self, type=1):
        #优先跨地图，再充电
        rospy.loginfo("navigationStatusCallback 到楼梯点切换地图, self.target_goal.floor_id: %d, self.target_goal.map_id: %d, type: %d", self.across_elevator_entry.floor_id, self.across_elevator_entry.map_id, type)

        if type == 1:
            # 开始按楼层
            target_floor_num = UInt16MultiArray()
            target_floor_num.data.append(self.across_elevator_entry.lora_id)
            target_floor_num.data.append(self.across_elevator_floor)
            self.elevator_press_button_pub_.publish(target_floor_num)                

            # 1 准备切换地图
            if self.allMap is not None:
                mapSetting = MapSetting(self.allMap)
                # 逐条取出保存，方便后面
                floors = []
                for fll in mapSetting.getFloors():
                    floor = Floor(fll)
                    if (floor.getFloorId() == self.across_elevator_entry.floor_id):
                        # 找到要更新的楼层, 
                        for uu in floor.getMaps():
                            map = Map(uu)
                            if map.getMapId() == self.across_elevator_entry.map_id:
                                floor.setDefaultmap(self.across_elevator_entry.map_id)
                                inner_elevator_point_id = 0
                                # 先找到切换楼层后 电梯内的点 start
                                if map.getRegionList():
                                    find_elevator = False
                                    for regionn in map.getRegionList(): # 遍历区域 找到电梯
                                        region = Region(regionn)
                                        if region.getType() == 1:
                                            elevator_id = region.getRegionProperty1()
                                            if (int(elevator_id) == self.across_elevator_entry.elevator_id):
                                                # 找到电梯号                                            
                                                inner_elevator_point_id = region.getWaitPointId()
                                                rospy.logwarn("找到目标楼层 电梯内的点id %d inner_elevator_point_id: %d", self.across_elevator_entry.elevator_id, inner_elevator_point_id)                            
                                                for point in map.getPoints():
                                                    pointbean = MapsBeanPoint(point)
                                                    if int(pointbean.getPointId()) == inner_elevator_point_id:
                                                        rospy.logwarn("找到目标楼层 电梯内的点详情 ")
                                                        self.across_elevator_point = pointbean
                                                        rospy.loginfo("self.across_elevator_point: %f, %f,%f", self.across_elevator_point.getPositionX(), self.across_elevator_point.getPositionY(),self.across_elevator_point.getPositionYaw())
                                                        self.publishlocation_miss = 10
                                                        self.save_position_flag = 2
                                                        break
                                break
                    floors.append(floor)
                mapSetting.setFloors(floors)
                mapSetting.setDefaultFloor(self.across_elevator_entry.floor_id)
                rospy.loginfo("is_across_floor mappseting %s", self.bean_to_dict(mapSetting))
                self.write_json(mapSetting, spirit_map_url)  # 跨地图 准备切换地图 map
                
                #self.movebase_ready = False     
                self.is_across_loadingMap = True     
                self.check_map(True)  #  is_across_floor 切换地图 change map
                return # 这里要返回，不然 会触发重定位 
        else:
            # 充电重定位切换地图
            # 1 准备切换地图            
            if self.allMap is not None:
                mapSetting = MapSetting(self.allMap)
                # 逐条取出保存，方便后面
                floors = []
                chargefloor = 0
                if mapSetting.getChargingPrioritys():
                    chargefloor = mapSetting.getChargingPrioritys()[0]
                    rospy.loginfo("changeTargetMap chargeFloorId = %d",chargefloor)

                for fll in mapSetting.getFloors():
                    floor = Floor(fll)
                    floors.append(floor)

                mapSetting.setFloors(floors)
                rospy.loginfo("chargefloor: %d", chargefloor)
                mapSetting.setDefaultFloor(chargefloor)
                rospy.loginfo("is_across_floor mappseting %s", self.bean_to_dict(mapSetting))
                self.write_json(mapSetting, spirit_map_url)  # 跨地图 准备切换地图 map
                
                #self.movebase_ready = False     
                self.is_across_loadingMap = True     
                self.check_map(True)  #  is_across_floor 切换地图 change map
                return # 这里要返回，不然 会触发重定位 

    def navigationStatusCallback(self, msg):
        self.navigation_status = msg.status           
        if (self.navigation_status == 2):
            rospy.loginfo("navigationStatusCallback, navigation status (2), Goal Reached!")
            # 到达导航点，进行到达处理

            # 保存当前位置
            if self.save_position_flag !=2:
                self.save_position_flag = 1

            if self.target_goal.type ==24 and self.start_chargecheck:
                # 如果已经在充电了，不能跟充电桩导航点进行比较，相差会过大，这里单独进行处理
                if (self.map_driverinfo.ChargeStatus > 0 and self.map_driverinfo.ChargeStatus <5):
                    rospy.loginfo("自动充电中，进入下一个导航")
                    #self.task_current_wait_time = self.task_executing_wait_time
                    #self.get_nexttask = True
                    #self.task_runs_status = False
                    #self.AndroidAlarm.androidmsg_navigationstatus = 86  #对桩成功
                else:
                    # 对桩没有对上， 重新对桩
                    # 充电模式下导航到导航点成功，开始下发自动充电流程，feck_dock模块开始工作
                    rospy.loginfo("充电失败，再来一次。。。")
                    self.start_chargecheck = False
                    if self.task_executing == 0:
                        self.task_executing = 1
                    self.get_nexttask = True
                    self.task_runs_status = False
                    self.AndroidAlarm.androidmsg_navigationstatus = 87  # 对桩失败
                return
            
            if self.target_goal.type == 22 and self.start_liftcheck:
                # 顶升结束判断
                self.start_liftcheck = False
                rospy.loginfo("顶升结束，进入下一个导航")
                self.task_current_wait_time = -1
                self.get_nexttask = True
                self.task_runs_status = False
                self.AndroidAlarm.androidmsg_navigationstatus = 90 # 顶升完成
                return
            
            if self.target_goal.type == 23 and self.start_putdowncheck:
                # 卸货结束判断
                self.start_putdowncheck = False
                rospy.loginfo("卸货结束，进入下一个导航")
                self.task_current_wait_time = -1
                self.get_nexttask = True
                self.task_runs_status = False
                self.AndroidAlarm.androidmsg_navigationstatus = 93 # 卸货完成
                return
            
            # 先判定当前位置是否就是导航点的位置，如果不是，说明这个到达消息不正确，再次发起导航
            current_goal = self.target_goal
            if self.is_across_floor:
                current_goal.floor_id = self.across_elevator_entry.floor_id
                current_goal.map_id = self.across_elevator_entry.map_id
                current_goal.point_id = self.across_elevator_entry.point_id
            rospy.loginfo("current_goal: %d, %d, %d", current_goal.floor_id, current_goal.map_id, current_goal.point_id)

            # 判断当前点跟要到达的导航点进行比较，如果距离过远大于0.15米，说明未达到，需要再次发起导航
            arrived_goal = False
            max_distance = 0.15
            if self.allMap:
                mapSetting = MapSetting(self.allMap)
                for fll in mapSetting.getFloors():
                    floor = Floor(fll)
                    if (int(floor.getFloorId()) == int(current_goal.floor_id)):
                        for uu in floor.getMaps():
                            map = Map(uu)
                            if map.getMapId() == floor.getDefaultmap() and int(map.getMapId()) == int(current_goal.map_id):
                                point = None
                                findpoint = False
                                for pin in map.getPoints():
                                    tmppoint = MapsBeanPoint(pin)
                                    if int(tmppoint.getPointId()) == int(current_goal.point_id):
                                        point = tmppoint
                                        findpoint = True
                                        break
                                if findpoint == False:
                                    for pin1 in map.getSystemPoints():
                                        tmpsyspoint = MapsBeanPoint(pin1)
                                        rospy.loginfo("int(tmpsyspoint.getPointId()): %d int(current_goal.point_id: %d", int(tmpsyspoint.getPointId()), int(current_goal.point_id))
                                        if int(tmpsyspoint.getPointId()) == int(current_goal.point_id):
                                            point = tmpsyspoint
                                            findpoint = True
                                            break
                                if findpoint:
                                    distancegoal = sqrt(pow((point.getPositionX() - self.current_pose.pose.position.x), 2) + pow((point.getPositionY() - self.current_pose.pose.position.y), 2))
                                    rospy.loginfo("distancegoal: %f currentpose: %f, %f, targetpoint: %f, %f", distancegoal, self.current_pose.pose.position.x, self.current_pose.pose.position.y, point.getPositionX(), point.getPositionY())
                                    if (abs(distancegoal) < max_distance):
                                        arrived_goal = True
                                else:
                                    rospy.loginfo("not find target point...")
                                break
                        break

            if arrived_goal:
                rospy.loginfo("当前点位置跟目标导航点位置一致，到达目标点...")
                self.navi_retry_times = 0
                if self.first_targetjump == True:
                    self.androidStartGoal_pub.publish(self.target_goal)  
                    self.first_targetjump = False

                #优先跨地图，再充电
                if self.is_across_floor:  # 如果是跨地图
                    rospy.loginfo("navigationStatusCallback 到楼梯点切换地图")
                    self.navigation_status = 0 # 避免自动测试中 认为到点
                    self.is_across_floor = False
                    
                    addoneGoal = TargetGoalEntry()
                    addoneGoal.floor_id = int(self.across_elevator_entry.floor_id)
                    addoneGoal.map_id = int(self.across_elevator_entry.map_id)
                    addoneGoal.point_id = int(self.across_elevator_entry.point_id)
                    addoneGoal.type = 25

                    self.target_goal_lock.acquire()
                    self.subtask_targetgoals.insert(0, addoneGoal)
                    self.target_goal_lock.release()
                    if self.task_executing == 0:
                        self.task_executing = 1
                    self.get_nexttask = True
                    self.task_runs_status = False
                    return 
                
                if (self.waitfornavi == True):
                    self.navigation_status = 0 # 避免自动测试中 认为到点
                    self.waitfornavi = False
                    # 充电模式下导航到导航点成功，开始下发自动充电流程，feck_dock模块开始工作
                    rospy.loginfo("navigationStatusCallback, 在充电模式下，导航到充电桩成功!")
                    addoneGoal = TargetGoalEntry()
                    addoneGoal.type = 24
                    self.target_goal_lock.acquire()
                    self.subtask_targetgoals.insert(0, addoneGoal)
                    self.target_goal_lock.release()
                    if self.task_executing == 0:
                        self.task_executing = 1
                    self.get_nexttask = True
                    self.task_runs_status = False
                    return
                
                if (self.target_goal.type == 22 and self.start_liftcheck == False):
                    self.navigation_status = 0 # 避免
                    # 顶升到达导航点，开始开启顶升任务
                    rospy.loginfo("navigationStatusCallback, 在顶升模式下，导航到顶升导航点成功!")
                    addoneGoal = TargetGoalEntry()
                    addoneGoal = copy.deepcopy(self.target_goal)
                    addoneGoal.type = 22

                    self.target_goal_lock.acquire()
                    self.subtask_targetgoals.insert(0, addoneGoal)
                    self.target_goal_lock.release()
                    if self.task_executing == 0:
                        self.task_executing = 1
                    self.get_nexttask = True
                    self.task_runs_status = False
                
                if (self.target_goal.type == 23 and self.start_putdowncheck == False):
                    self.navigation_status = 0 # 避免
                    # 顶升到达导航点，开始开启顶升任务
                    rospy.loginfo("navigationStatusCallback, 在顶升模式下，导航到卸货点导航点成功!")
                    addoneGoal = TargetGoalEntry()
                    addoneGoal.type = 23

                    self.target_goal_lock.acquire()
                    self.subtask_targetgoals.insert(0, addoneGoal)
                    self.target_goal_lock.release()
                    if self.task_executing == 0:
                        self.task_executing = 1
                    self.get_nexttask = True
                    self.task_runs_status = False
        
                if self.lora_call_id != 0:
                    lora_page = UInt16MultiArray()
                    lora_page.data.append(self.lora_call_id)
                    lora_page.data.append(3)
                    self.page_status_pub.publish(lora_page)
                    self.lora_call_id = 0
                
                # 导航成功 恢复字段，避免任务结束不掉
                self.target_goal.floor_id = 0
                self.target_goal.map_id = 0
                self.target_goal.point_id = 0
                self.is_across_floor = False

                self.target_goal_lock.acquire()
                if self.high_targetgoals and self.high_targetgoals[0].status == 1:
                    self.high_targetgoals[0].status = 2
                    self.operated_targetGoals.append(self.high_targetgoals[0])
                    if self.high_targetgoals[0].led_warn_id:
                        led_page = UInt16MultiArray()
                        led_page.data.append(self.high_targetgoals[0].led_warn_id)
                        led_page.data.append(self.ledWarnTime)
                        self.ledWarn_call_pub.publish(led_page)
                    self.high_targetgoals.pop(0)
                elif self.low_targetgoals and self.low_targetgoals[0].status == 1:
                    self.low_targetgoals[0].status = 2
                    self.operated_targetGoals.append(self.low_targetgoals[0])
                    if self.low_targetgoals[0].led_warn_id:
                        led_page = UInt16MultiArray()
                        led_page.data.append(self.low_targetgoals[0].led_warn_id)
                        led_page.data.append(self.ledWarnTime)
                        self.ledWarn_call_pub.publish(led_page)
                    self.low_targetgoals.pop(0)
                self.target_goal_lock.release()           

                # 在外面不能给他重置数据，不然 首次导航会收到影响
                self.task_current_wait_time = self.task_executing_wait_time
                self.get_nexttask = True
                self.task_runs_status = False

                #self.androidNavigationStatus_pub.publish(2)
                self.AndroidAlarm.androidmsg_navigationstatus = 2    
            else:                    
                if self.subtask_targetgoals or self.low_targetgoals or self.high_targetgoals:
                    rospy.loginfo("当前点位置跟目标导航点位置不一致，继续导航...")
                    #优先跨地图，再充电
                    if self.is_across_floor:  # 如果是跨地图
                        rospy.loginfo("navigationStatusCallback 到楼梯点切换地图")
                        self.navigation_status = 0 # 避免自动测试中 认为到点
                        self.is_across_floor = False
                        
                        addoneGoal = TargetGoalEntry()
                        addoneGoal.floor_id = int(self.across_elevator_entry.floor_id)
                        addoneGoal.map_id = int(self.across_elevator_entry.map_id)
                        addoneGoal.point_id = int(self.across_elevator_entry.point_id)
                        addoneGoal.type = 25

                        self.target_goal_lock.acquire()
                        self.subtask_targetgoals.insert(0, addoneGoal)
                        self.target_goal_lock.release()

                    if self.task_executing == 0:
                        self.task_executing = 1
                    self.get_nexttask = True   
                    self.task_runs_status = False

        elif (self.navigation_status == 3):
            # 失败重试
            if self.first_targetjump == True:
                self.androidStartGoal_pub.publish(self.target_goal)  
                self.first_targetjump = False

            self.navi_retry_times = self.navi_retry_times + 1
            rospy.logwarn("按照队列中, 再一次导航")
            #self.androidNavigationStatus_pub.publish(3)
            self.AndroidAlarm.androidmsg_navigationstatus = 3
            if self.target_goal.type == 22 and self.start_liftcheck:
                # 如果是顶升失败，那么不能走卸货流程
                rospy.loginfo("顶升失败，删除队列中的卸货点")
                self.start_liftcheck = False
                self.target_goal_lock.acquire()
                if self.low_targetgoals:
                    if self.low_targetgoals[0].type == 23:
                        self.low_targetgoals.pop(0)
                self.target_goal_lock.release()
            self.get_nexttask = True
            self.task_runs_status = False
            return

        else:
            rospy.loginfo("navigationStatusCallback, navigation status (%d)", self.navigation_status)      
            if self.navigation_status == 1 and self.AndroidAlarm.androidmsg_navigationstatus != 1:
                # 重新给安卓发一次导航点
                self.androidStartGoal_pub.publish(self.target_goal)  
                self.first_targetjump = False
                self.AndroidAlarm.androidmsg_navigationstatus = 1
                
            if self.navigation_status == 8 or self.navigation_status == 1 or self.navigation_status == 83 or self.navigation_status == 85 or self.navigation_status == 86 \
               or self.navigation_status == 87 or self.navigation_status == 88 or self.navigation_status == 89 or self.navigation_status == 90 or self.navigation_status == 91 \
               or self.navigation_status == 92 or self.navigation_status == 93 or self.navigation_status == 94 or self.navigation_status == 95 or self.navigation_status == 96 \
               or self.navigation_status == 97 or self.navigation_status == 98 or self.navigation_status == 99 or self.navigation_status == 100:
                self.AndroidAlarm.androidmsg_navigationstatus = self.navigation_status 

            if self.navigation_status == 72 and self.AndroidAlarm.androidmsg_navigationstatus == 8:
                self.AndroidAlarm.androidmsg_navigationstatus = self.navigation_status 

    def Sonar_acturalCallback(self, msg):
        if (self.sonar_flag == 0):
            # 超声波数据接收正常
            if (self.AndroidAlarm.androidmsg_sonarstatus != 106):
                rospy.loginfo("机器人超声波板正常。")
                #self.AndroidmsgReportPub.publish(106)
                self.androidSonarStatus_pub.publish(106)
                self.AndroidAlarm.androidmsg_sonarstatus = 106
            self.sonar_flag = 1
        elif (self.sonar_flag == 2):
            self.sonar_flag = 1

        self.sonardata[0] = msg.data[0] # 正前
        self.sonardata[1] = msg.data[1] # 正上
        self.sonardata[2] = msg.data[2] # 左侧
        self.sonardata[3] = msg.data[3] # 右侧
        self.sonardata[4] = msg.data[4] # 左前
        self.sonardata[5] = msg.data[5] # 右前
        self.sonardata[6] = msg.data[6] # 后

    def ImuInitSetCallback(self, msg):
        # 校准IMU初始位置
        if (msg.data == 1):
            # 修正IMU初始化位置
            tmp_imuInitsetting = dict()
            tmp_imuInitsetting.clear()
            tmp_imuInitsetting = self.read_param(robot_driver_param_url)
            imuInitsetting = dict()
            imuInitsetting.clear()
            if not bool(tmp_imuInitsetting):
                rospy.loginfo("robot driver setting is empty")
                return
            else:
                imuInitsetting = tmp_imuInitsetting.copy()

            imuInitsetting["imuAnglex"] = self.imu_angle.x
            imuInitsetting["imuAngley"] = self.imu_angle.y
            imuInitsetting["imuAnglez"] = self.imu_angle.z
            self.init_imu_angle.x = self.imu_angle.x
            self.init_imu_angle.y = self.imu_angle.y
            self.init_imu_angle.z = self.imu_angle.z
            self.writ_patam(robot_driver_param_url, imuInitsetting)
            self.imu_initxyz_pub.publish(self.init_imu_angle)
            rospy.loginfo("save initial IMU position success!")


    def LocationStatusCallback(self, msg):
        # 复位cartographer标志
        self.cartographer_status = -1
        if ((msg.data == 1) and (msg.data != self.location_missing)):
            # 全局定位丢失
            self.location_missing = msg.data
            if (self.AndroidAlarm.androidmsg_locationstatus !=9):
                #self.AndroidmsgReportPub.publish(9)
                self.androidLocationStatus_pub.publish(9)
                self.AndroidAlarm.androidmsg_locationstatus = 9
                rospy.loginfo("机器人位置丢失！请修复！")

        elif ((msg.data == 3 or msg.data == 13) and (msg.data != self.location_missing)):
            # 位置正常
            self.location_missing = msg.data
            if (self.AndroidAlarm.androidmsg_locationstatus !=10):
                #self.AndroidmsgReportPub.publish(10)
                self.androidLocationStatus_pub.publish(10)
                self.AndroidAlarm.androidmsg_locationstatus = 10
                rospy.loginfo("机器人重新定位成功！")

        if (msg.data == 3 or msg.data == 13):
            self.need_navigation = True
            self.neednavigation_pub.publish(1)
    
    def SendMsgreport_CB(self, msg):
        if (msg.data == 201):
            # cartographer is OK
            if (self.slam_ready == True):
                return
            self.slam_ready = True
            rospy.loginfo("机器人cartographer启动正常。")
            self.check_navigation_isOK()

    def MovebaseMsgreport_CB(self, msg):
        if (msg.data == 200):
            # movebase is OK
            self.movebase_ready = True
            self.sendSpeed_pub.publish(self.dwaspeed)
            rospy.loginfo("重新发送dwa速度模式成功！")
            rospy.loginfo("机器人Movebase模块工作正常。")
            self.check_navigation_isOK()

    def VirtualwallMsgreport_CB(self, msg):
        if (msg.data == 63):
            # 虚拟墙解析出错
            if (self.AndroidAlarm.androidmsg_virtualmapstatus !=63):
                rospy.loginfo("机器人虚拟墙加载失败，导航模块不工作！")
                #self.AndroidmsgReportPub.publish(63)
                self.androidVirtualmapStatus_pub.publish(63)
                self.AndroidAlarm.androidmsg_virtualmapstatus = 63
        elif (msg.data == 62):
            # 虚拟墙解析成功
            if (self.AndroidAlarm.androidmsg_virtualmapstatus !=62):
                rospy.loginfo("机器人虚拟墙加载成功。")
                #self.AndroidmsgReportPub.publish(62)
                self.androidVirtualmapStatus_pub.publish(62)
                self.AndroidAlarm.androidmsg_virtualmapstatus = 62

    def elevatorStatus_CB(self, msg):
        self.elevator_floor = msg.data[1]
        return
    
    def resetParams_CB(self, msg):
        rospy.loginfo("resetParams_CB start resetParames!")        
        if self.car_type == 300:
            os.system('chmod +x /home/spirit/spiritdeb/reset300params.sh')
            os.system('/home/spirit/spiritdeb/reset300params.sh')
        else:
            os.system('chmod +x /home/spirit/spiritdeb/resetparams.sh')
            os.system('/home/spirit/spiritdeb/resetparams.sh')


        os.system("rm -rf "+spirit_dir+"maps/*")
        self.local_point_list = []  # del allmap
        self.local_systempoint_list = []
        self.local_globalplans_list = []
        self.map_isEmpty = 1

        BAG_PATH = bag_param
        if not os.path.exists(BAG_PATH):
            os.mkdir(BAG_PATH)

        self.check_map()  # delete All
        rospy.loginfo("resetParams_CB 删除所有地图成功。")
        self.wall_update_pub.publish(0)


        os.system('reboot')
        return

    def NaviStop_CB(self, msg):
        if (msg.data == 5):
            # cancel navigation
            self.task_stop(True)

            self.mode = 2
            self.waitfornavi = False
            self.motionmode_pub.publish(2)
            #self.AndroidmsgReportPub.publish(83)
            #self.androidNavigationStatus_pub.publish(83)
            self.AndroidAlarm.androidmsg_navigationstatus = 83
            rospy.loginfo("机器人导航取消！")

            if self.testodom_switch:
                #取消odom测试
                self.testodom_pub.publish(10000)
                self.testodom_switch = False

    def AutoCharge_CB(self, msg):
        # 1: autocharge, 2: manumode
        if not self.odom_ready:
            return
        if self.current_jacking_status==1:
            rospy.loginfo("机器人处于顶升状态！")
            return
        
        if not self.local_systempoint_list:
            rospy.loginfo("机器人没有设置充电桩！")
            self.AndroidAlarm.androidmsg_navigationstatus = 52
            #self.AndroidmsgReportPub.publish(52)
            #self.androidNavigationStatus_pub.publish(52)  # 找不到充电桩
            return

        if self.need_navigation == False:
            # 上报一个告警，在安全模式下导航，报定位已丢失
            self.AndroidAlarm.androidmsg_navigationstatus = 84
            #self.androidNavigationStatus_pub.publish(84)
            rospy.loginfo("机器人初始化后没有进行重定位成功，进入旋转模式！")

        if msg.data == 1:
            rospy.loginfo("机器人设置为自动回充模式成功。")
            if (self.map_driverinfo.ChargeStatus > 0 and self.map_driverinfo.ChargeStatus <5):
                rospy.loginfo("机器人已经在充电中！！！")
                return
            (isFind, changeTarget) = self.findChargeTargetGoalEntry() # AutoCharge_CB
            if isFind:
                #  清空list
                self.task_stop(True)

                self.target_goal_lock.acquire()
                self.low_targetgoals.append(changeTarget)
                self.low_targetgoals_backup = self.low_targetgoals
                self.target_goal_lock.release()
                if self.task_executing == 0:
                    self.task_executing = 1
                    self.get_nexttask = True  
                    self.task_runs_status = False              

            else: # 如果没找到 没有充电桩
                rospy.loginfo("机器人没有设置充电桩！")
                self.AndroidAlarm.androidmsg_navigationstatus = 52
                #self.androidNavigationStatus_pub.publish(52)  # 找不到充电桩

        elif msg.data == 2:
            rospy.loginfo("机器人设置为MANU模式成功。")
            # manu mode
            self.mode = 2
            self.waitfornavi = False
            self.motionmode_pub.publish(2)
            self.task_stop(True)   

    def findChargeTargetGoalEntry(self):
        isFind = False
        if self.allMap:
            mapSetting = MapSetting(self.allMap)
            chargeFloorId = 0
            if mapSetting.getChargingPrioritys():
                chargeFloorId = mapSetting.getChargingPrioritys()[0]
                rospy.loginfo("findChargeTargetGoalEntry  chargeFloorId = %d",chargeFloorId)
            
            # 如果没有找到跨楼层充电，则在本楼层充电
            if chargeFloorId == 0:
                chargeFloorId = mapSetting.getDefaultFloor()

            for fll in mapSetting.getFloors():
                floor = Floor(fll)
                if (floor.getFloorId() == chargeFloorId):
                    for uu in floor.getMaps():
                        map = Map(uu)
                        if map.getMapId() == floor.getDefaultmap():
                            for pin in map.getPoints():
                                point = MapsBeanPoint(pin)
                                if point.getPointName() == str("充电桩"):
                                    # 先找全局充电桩
                                    rospy.loginfo("AutoCharge_CB 找到普通充电桩")
                                    addchargeGoal = TargetGoalEntry()
                                    addchargeGoal.floor_id = int(floor.getFloorId())
                                    addchargeGoal.floor_name = str(floor.getFloorName())
                                    addchargeGoal.map_id = int(map.getMapId())
                                    addchargeGoal.map_name = str(map.getMapName())
                                    addchargeGoal.point_id = int(point.getPointId())
                                    addchargeGoal.point_name = str(point.getPointName())
                                    addchargeGoal.type = 24
                                    rospy.loginfo("机器人开始导航到全局普通充电桩: %f, %f, %f", point.getPositionX(), point.getPositionY(), point.getPositionYaw())
                                    isFind = True
                                    break
                            if isFind == False:
                                # 没有找到，寻找当楼层系统充电桩
                                for cpin in map.getSystemPoints():
                                    chargepoint = MapsBeanPoint(cpin)
                                    if chargepoint.getPointName() == str("充电桩"):
                                        addchargeGoal = TargetGoalEntry()
                                        addchargeGoal.floor_id = int(chargeFloorId)
                                        addchargeGoal.floor_name = str(floor.getFloorName())
                                        addchargeGoal.map_id = int(floor.getDefaultmap())
                                        addchargeGoal.map_name = str(map.getMapName())
                                        addchargeGoal.point_id = int(chargepoint.getPointId())
                                        addchargeGoal.point_name = str(chargepoint.getPointName())
                                        addchargeGoal.type = 24
                                        rospy.loginfo("机器人开始导航到全局系统充电桩: %f, %f, %f", chargepoint.getPositionX(), chargepoint.getPositionY(), chargepoint.getPositionYaw())
                                        isFind = True
                            break
                    break
               
        return (isFind, addchargeGoal)
    
    def addChargepointTargetGoalEntry(self):
        self.mode = 1
        self.motionmode_pub.publish(1)  # 1: autocharge mode

        dock_pose = PoseStamped()
        dock_pose.header.frame_id = 'map'
        dock_pose.header.stamp = rospy.Time.now()                
        dockx = 0.0
        docky = 0.0

        chargepoing_found = False
        dockz = 0
        for point in self.local_systempoint_list:
            if str("桩位置") == str(point.point_name):
                dock_pose.pose.position.x = point.x
                dock_pose.pose.position.y = point.y
                q = transformations.quaternion_from_euler(0.0, 0.0, point.z/180.0*pi)
                dock_pose.pose.orientation.x = q[0]
                dock_pose.pose.orientation.y = q[1]
                dock_pose.pose.orientation.z = q[2]
                dock_pose.pose.orientation.w = q[3]
                dockz = point.z
                self.fix_chargepoint_flag = True
            else:
                self.fix_chargepoint_flag = False
                if str("充电桩") == str(point.point_name):
                    q = transformations.quaternion_from_euler(0.0, 0.0, point.z/180.0*pi)
                    dock_pose.pose.orientation.x = q[0]
                    dock_pose.pose.orientation.y = q[1]
                    dock_pose.pose.orientation.z = q[2]
                    dock_pose.pose.orientation.w = q[3]
                    dockx = point.x
                    docky = point.y
                    dockz = point.z
                    chargepoing_found = True
                if str("初始点") == str(point.point_name):
                    dock_pose.pose.position.x = point.x
                    dock_pose.pose.position.y = point.y                        
        
        if chargepoing_found:
            distanceCharge = sqrt(pow((dockx - dock_pose.pose.position.x), 2) + pow((docky - dock_pose.pose.position.y), 2))
            # 如果初始点离充电桩太远，以充电桩的点为参考点
            if (distanceCharge > 1.5):
                rospy.loginfo("初始点和充电桩相隔太远，选用充电桩来代替标志充电桩位置！！！！")
                dock_pose.pose.position.x = dockx
                dock_pose.pose.position.y = docky
        if self.fix_chargepoint_flag:
            dock_pose.pose.position.z = 0   # 借用这一个字节传递type类型
        else:
            dock_pose.pose.position.z = 1

        self.start_lidarcharge_pub.publish(dock_pose)

        rospy.loginfo("发送充电模式到驱动成功，启动驱动自动充电模式!，充电桩所在的标杆位置：%f, %f %f", dock_pose.pose.position.x, dock_pose.pose.position.y, dockz)
        self.AndroidAlarm.androidmsg_navigationstatus = 2
        return 
    
    def addLiftTargetGoalEntry(self, subgoal):
        addputdownGoal = TargetGoalEntry()
        addputdownGoal.floor_id = int(subgoal.floor_id_putdown)
        addputdownGoal.map_id = int(subgoal.map_id_putdown)
        addputdownGoal.point_id = int(subgoal.point_id_putdown)
        addputdownGoal.floor_name = subgoal.floor_name_putdown
        addputdownGoal.map_name = subgoal.map_name_putdown
        addputdownGoal.point_name = subgoal.point_name_putdown
        addputdownGoal.type = 23

        rospy.loginfo("addputdownGoal: floor_id: %d, map_id: %d, point_id: %d", int(subgoal.floor_id_putdown), int(subgoal.map_id_putdown), int(subgoal.point_id_putdown))
       
        self.target_goal_lock.acquire()
        self.low_targetgoals.insert(0, addputdownGoal)
        self.target_goal_lock.release()

        rospy.loginfo("low_targetgoals:")
        for goal in self.low_targetgoals:
            rospy.loginfo(" %d, %d, %d", goal.floor_id, goal.map_id, goal.point_id)

        rospy.logwarn("addLiftTargetGoalEntry 添加卸货子任务成功")        

        navigoal_pose = PoseStamped()
        navigoal_pose.header.frame_id = 'map'
        navigoal_pose.header.stamp = rospy.Time.now() 
        self.identify_and_lift_shelves_pub.publish(navigoal_pose)
        rospy.loginfo("启动顶升任务，导航点位置：%f, %f", navigoal_pose.pose.position.x, navigoal_pose.pose.position.y)
        return 
    
    def addPutdownTargetGoalEntry(self):

        navigoal_pose = PoseStamped()
        navigoal_pose.header.frame_id = 'map'
        navigoal_pose.header.stamp = rospy.Time.now()                

        self.arrive_and_place_shelves_pub.publish(navigoal_pose)
        rospy.loginfo("启动卸货任务，导航点位置：%f, %f", navigoal_pose.pose.position.x, navigoal_pose.pose.position.y)
        return 

    def AppToRobot_CB(self, msg):
        rospy.loginfo("APP进入新的页面：%d", msg.data)
        self.app_status = msg.data
        self.appStatus_pub(msg.data)

    def Updatestatus_CB(self, msg):
        rospy.loginfo("收到程序更新消息: %d", msg.data)
        self.update_status = msg.data

    def MovebaseReady_CB(self, msg):
        self.movebase_heart = msg.data

    def TrackedPose_CB(self, msg):
        # 复位cartographer标志
        self.cartographer_status = 0
        self.current_pose = msg    

    def ErasePose_CB(self, msg):
        #rospy.loginfo("receive pose: %f, %f", msg.pose.position.x, msg.pose.position.y)
        if not self.localmap_ready:
            return

        width = self.sendmapdata.info.width
        height = self.sendmapdata.info.height
        resolution = self.sendmapdata.info.resolution
        origin_x = self.sendmapdata.info.origin.position.x
        origin_y = self.sendmapdata.info.origin.position.y

        xdata = int((msg.x - origin_x) / resolution)
        ydata = int((msg.y - origin_y) / resolution)
        erasepoint = ydata * width + xdata
        maxpoint = width * height

        if 0 <= erasepoint < maxpoint:
            if self.erasermode == 1:
                # 擦除一个像素点
                self.sendmapdata.data[erasepoint] = 0

            elif self.erasermode == 3:
                # 擦除一个25个像素点的大正方形
                for dy in range(-2, 3):
                    for dx in range(-2, 3):
                        point = (ydata + dy) * width + (xdata + dx)
                        if 0 <= point < maxpoint:
                            self.sendmapdata.data[point] = 0

            else:
                # 擦除一个中等9个点正方形
                for dy in range(-1, 2):
                    for dx in range(-1, 2):
                        point = (ydata + dy) * width + (xdata + dx)
                        if 0 <= point < maxpoint:
                            self.sendmapdata.data[point] = 0

        self.mapupdate_pub.publish(self.sendmapdata)
        return
    

    def loraClientReceived_CB(self, msg):
        try:
            lora_id = msg.data[0]
            lora_type = msg.data[1]
            self.lora_call_id = lora_id
            rospy.loginfo("receive lora msg, lora_id: %d, lora_type: %d", lora_id, lora_type)
            
        except:
            rospy.loginfo("receive lora msg, data < 2")
            return

        if int(lora_type) == 4:
            # 取消呼叫
            lora_page = UInt16MultiArray()
            lora_page.data.append(lora_id)
            lora_page.data.append(4)
            self.page_status_pub.publish(lora_page)
            self.navi_stop_pub.publish(5)
            self.lora_call_id = 0
            return

        rospy.loginfo("lora_client 收到呼叫器 %s",msg.data[0])

        if self.allMap is None:
            rospy.logerr("lora_client 机器人导航失败，地图文件没有加载成功！")
            return
        
        self.need_gotodefaultPoint = True
        
        mapSetting_bean = MapSetting(self.allMap)
        for floor in mapSetting_bean.getFloors():
            floorbean = Floor(floor)
            if floorbean.getMaps():
                for map in floorbean.getMaps():
                    mapbean = Map(map)
                    # 只寻找各个楼层中默认地图
                    if mapbean.getMapId() == floorbean.getDefaultmap():
                        for point in mapbean.getPoints():
                            pointbean = MapsBeanPoint(point)
                            addoneGoal = TargetGoalEntry()

                            if int(pointbean.getDeviceType()) == 21 and int(pointbean.getDeviceID()) == int(lora_id):
                                lora_page = UInt16MultiArray()
                                lora_page.data.append(lora_id)
                                lora_page.data.append(1)
                                self.page_status_pub.publish(lora_page)
                                
                                addoneGoal.floor_id = int(floorbean.getFloorId())
                                addoneGoal.map_id = int(mapbean.getMapId())
                                addoneGoal.point_id = int(pointbean.getPointId())
                                addoneGoal.point_name = pointbean.getPointName()

                                addoneGoal.floor_id_putdown = pointbean.getBindFloorId()
                                addoneGoal.map_id_putdown = pointbean.getBindMapId()
                                addoneGoal.point_id_putdown = pointbean.getBindPointId()

                                if pointbean.getLedWarnID():
                                    addoneGoal.led_warn_id =  int(pointbean.getLedWarnID())
                                
                                if addoneGoal.point_id_putdown !=0:
                                    for bindpoint in mapbean.getPoints():
                                        bindpointBean = MapsBeanPoint(bindpoint)
                                        if addoneGoal.point_id_putdown == bindpointBean.getPointId():
                                            addoneGoal.point_name_putdown = bindpointBean.getPointName()
                                            break

                                if addoneGoal.point_id_putdown != 0:
                                    addoneGoal.type = 22
                                else:    
                                    addoneGoal.type = 21

                                self.target_goal_lock.acquire()
                                self.high_targetgoals.append(addoneGoal)
                                self.high_targetgoals_backup = copy.deepcopy(self.high_targetgoals)  
                                
                                self.target_goal_lock.release()
                                if self.task_executing == 0:
                                    self.task_executing = 1
                                    self.get_nexttask = True
                                    self.task_runs_status = False
                                rospy.loginfo("loraClientReceived_CB high_targetgoals:")
                                for goal in self.high_targetgoals:
                                    rospy.loginfo("loraClientReceived_CB high_target goal: floorid: %d floorname: %s mapid: %d mapname: %s pointid: %d pointname: %s", goal.floor_id, str(goal.floor_name), goal.map_id, str(goal.map_name), goal.point_id, str(goal.point_name))
                                return 
        rospy.logerr("lora_client 机器人导航失败，从默认地图中没找到呼叫器绑定的点")

    def TargetGoal_CB(self, msg):
        # 单点导航，app和屏幕操作，需要打断和清targetgoal list列表
        rospy.loginfo("TargetGoal_CB 收到topic消息 楼层ID %d , 地图ID %d , 导航点ID %d", msg.floor_id, msg.map_id, msg.point_id)

        if not self.odom_ready:
            rospy.loginfo("odom isnot ready, return")
            return
        
        if msg.floor_id == 0 or msg.map_id == 0 or msg.point_id == 0:
            rospy.logwarn("TargetGoal_CB 导航信息错误")
            return

        if self.need_navigation == False:
            # 上报一个告警，在安全模式下导航，报定位已丢失
            self.AndroidAlarm.androidmsg_navigationstatus = 84
            #self.androidNavigationStatus_pub.publish(84)
            rospy.loginfo("TargetGoal_CB 机器人初始化后没有进行重定位成功，进入旋转模式！")

        #  停止并清空list        
        self.task_stop(True)
        
        self.need_gotodefaultPoint = True

        addoneGoal = TargetGoalEntry()
        addoneGoal.floor_id = int(msg.floor_id)
        addoneGoal.map_id = int(msg.map_id)
        addoneGoal.point_id = int(msg.point_id)
        mapSetting_bean = MapSetting(self.allMap)
        for floor in mapSetting_bean.getFloors():   
            floorbean = Floor(floor)
            if addoneGoal.floor_id == floorbean.getFloorId():
                addoneGoal.floor_name = floorbean.getFloorName()
                for map in floorbean.getMaps():
                    mapbean = Map(map)
                    if addoneGoal.map_id == mapbean.getMapId():
                        addoneGoal.map_name = mapbean.getMapName()
                        for point in mapbean.getPoints():
                            pointBean = MapsBeanPoint(point)
                            if addoneGoal.point_id == pointBean.getPointId():
                                addoneGoal.point_name = pointBean.getPointName()
                                addoneGoal.floor_id_putdown = pointBean.getBindFloorId()
                                addoneGoal.map_id_putdown = pointBean.getBindMapId()
                                addoneGoal.point_id_putdown = pointBean.getBindPointId()
                                if pointBean.getLedWarnID():
                                    addoneGoal.led_warn_id = int(pointBean.getLedWarnID())
                                break
                        
                        if addoneGoal.point_id_putdown !=0:
                            for bindpoint in mapbean.getPoints():
                                bindpointBean = MapsBeanPoint(bindpoint)
                                if addoneGoal.point_id_putdown == bindpointBean.getPointId():
                                    addoneGoal.point_name_putdown = bindpointBean.getPointName()
                                    break

        if addoneGoal.point_id_putdown != 0:
            addoneGoal.type = 22
        else:    
            addoneGoal.type = 20

        self.target_goal_lock.acquire()
        self.low_targetgoals.append(addoneGoal)

        self.target_goal_lock.release()
        if self.task_executing == 0:
            self.task_executing = 1
            self.get_nexttask = True
            self.task_runs_status = False

        rospy.loginfo("TargetGoal_CB low_targetgoals:")
        for goal in self.low_targetgoals:
            rospy.loginfo("TargetGoal_CB low_target goal: floorid: %d floorname: %s mapid: %d mapname: %s pointid: %d pointname: %s", goal.floor_id, str(goal.floor_name), goal.map_id, str(goal.map_name), goal.point_id, str(goal.point_name))
        self.first_targetjump = True

    def naviMultiTargetGoal_CB(self, msg):
        # 多导航，app和屏幕操作，需要打断和清targetgoal list列表
        for goal in msg:
            rospy.loginfo("naviMultiTargetGoal_CB 收到topic消息 楼层ID %d , 地图ID %d , 导航点ID %d, type: %d", goal.floor_id, goal.map_id, goal.point_id, goal.type)

        rospy.loginfo("goal size: %d", msg.size())

        if not self.odom_ready:
            rospy.loginfo("odom isnot ready, return")
            return
        
        if self.need_navigation == False:
            # 上报一个告警，在安全模式下导航，报定位已丢失
            self.AndroidAlarm.androidmsg_navigationstatus = 84
            #self.androidNavigationStatus_pub.publish(84)
            rospy.loginfo("naviMultiTargetGoal_CB 机器人初始化后没有进行重定位成功，进入旋转模式！")

        #  需要停止并清空list
        self.task_stop(True)

        self.need_gotodefaultPoint = True

        self.target_goal_lock.acquire()
        for goal in msg:
            addoneGoal = TargetGoalEntry()
            addoneGoal.floor_id = int(goal.floor_id)
            addoneGoal.map_id = int(goal.map_id)
            addoneGoal.point_id = int(goal.point_id)
            addoneGoal.type = int(goal.type)

            mapSetting_bean = MapSetting(self.allMap)
            for floor in mapSetting_bean.getFloors():   
                floorbean = Floor(floor)
                if addoneGoal.floor_id == floorbean.getFloorId():
                    addoneGoal.floor_name = floorbean.getFloorName()
                    for map in floorbean.getMaps():
                        mapbean = Map(map)
                        if addoneGoal.map_id == mapbean.getMapId():
                            addoneGoal.map_name = mapbean.getMapName()
                            for point in mapbean.getPoints():
                                pointBean = MapsBeanPoint(point)
                                if addoneGoal.point_id == pointBean.getPointId():
                                    addoneGoal.point_name = pointBean.getPointName()
                                    addoneGoal.floor_id_putdown = pointBean.getBindFloorId()
                                    addoneGoal.map_id_putdown = pointBean.getBindMapId()
                                    addoneGoal.point_id_putdown = pointBean.getBindPointId()
                                    if pointBean.getLedWarnID():
                                        addoneGoal.led_warn_id = int(pointBean.getLedWarnID())
                                    break
                            if addoneGoal.point_id_putdown !=0:
                                for bindpoint in mapbean.getPoints():
                                    bindpointBean = MapsBeanPoint(bindpoint)
                                    if addoneGoal.point_id_putdown == bindpointBean.getPointId():
                                        addoneGoal.point_name_putdown = bindpointBean.getPointName()
                                        break
            
            if addoneGoal.type == 0:
                self.low_targetgoals.append(addoneGoal)    
            if addoneGoal.type == 1:
                self.high_targetgoals.append(addoneGoal)   

        self.low_targetgoals_backup = copy.deepcopy(self.low_targetgoals)  
        self.high_targetgoals_backup = copy.deepcopy(self.high_targetgoals)  

        self.target_goal_lock.release()

        if self.task_executing == 0:
            self.task_executing = 1
            self.get_nexttask = True
            self.task_runs_status = False

        rospy.loginfo("naviMultiTargetGoal_CB low_targetgoals:")
        for goal in self.low_targetgoals:
            rospy.loginfo("naviMultiTargetGoal_CB low_target goal: floorid: %d floorname: %s mapid: %d mapname: %s pointid: %d pointname: %s", goal.floor_id, str(goal.floor_name), goal.map_id, str(goal.map_name), goal.point_id, str(goal.point_name))
        self.first_targetjump = True

    def set_types_target_goal_CB(self,msg):
        rospy.loginfo("set_types_target_goal_CB, circle: %d, wait_time: %d", msg.circle, msg.wait_time)
        self.task_circle = msg.circle
        self.circle_times = 0
        self.task_executing_wait_time = msg.wait_time
        return
    
    def add_wait_time_target_goal_CB(self,msg):
        rospy.loginfo("add_wait_time_target_goal_CB, task_current_wait_time: %d, addtime: %d", self.task_current_wait_time, msg.data)
        self.task_current_wait_time = self.task_current_wait_time+ msg.data
        return
    
    def update_navi_target_lists_pub(self):
        targetGoalEntryRunList =TargetGoalEntryRunList()
        targetGoalEntryRunList.operated_goal_list = self.operated_targetGoals
        targetGoalEntryRunList.high_goal_list = self.high_targetgoals
        targetGoalEntryRunList.low_goal_list = self.low_targetgoals
        targetGoalEntryRunList.subtask_targetgoals = self.subtask_targetgoals
        targetGoalEntryRunList.backup_low_goals_list = self.low_targetgoals_backup
        targetGoalEntryRunList.backup_high_goals_list = self.high_targetgoals_backup
        targetGoalEntryRunList.task_circle = self.task_circle
        targetGoalEntryRunList.task_executing = self.task_executing
        targetGoalEntryRunList.task_executing_wait_time = self.task_executing_wait_time
        targetGoalEntryRunList.task_current_wait_time = self.task_current_wait_time
        targetGoalEntryRunList.task_runs_status = self.task_runs_status
        targetGoalEntryRunList.navi_retry_times = self.navi_retry_times
        targetGoalEntryRunList.lowpowerExecute = self.lowpowerExecute
        targetGoalEntryRunList.lowpowerChargepoint_add = self.lowpowerChargepoint_add
        targetGoalEntryRunList.init_status = self.init_status
        targetGoalEntryRunList.car_type = self.car_type
        targetGoalEntryRunList.androidmsg_navigationstatus = self.AndroidAlarm.androidmsg_navigationstatus
        targetGoalEntryRunList.get_nexttask = self.get_nexttask
        targetGoalEntryRunList.waitfornavi = self.waitfornavi
        targetGoalEntryRunList.start_liftcheck = self.start_liftcheck
        targetGoalEntryRunList.start_putdowncheck = self.start_putdowncheck
        targetGoalEntryRunList.is_across_floor = self.is_across_floor
        targetGoalEntryRunList.circle_times = self.circle_times
        targetGoalEntryRunList.start_chargecheck = self.start_chargecheck
        self.navi_targetgoallist_pub.publish(targetGoalEntryRunList)

    def add_target_goal_CB(self, msg):
        rospy.loginfo("add_target_goal_CB 收到topic消息 %d",len(msg.goal_point_list))

        if not self.odom_ready:
            rospy.loginfo("add_target_goal_CB odom isnot ready, return")
            return
        
        if self.need_navigation == False:
            # 上报一个告警，在安全模式下导航，报定位已丢失
            self.AndroidAlarm.androidmsg_navigationstatus = 84
            #self.androidNavigationStatus_pub.publish(84)
            rospy.loginfo("add_target_goal_CB 机器人初始化后没有进行重定位成功，进入旋转模式！")
      
        self.lowpowerExecute = False
        self.lowpowerChargepoint_add = False
        self.need_gotodefaultPoint = True
        self.target_goal_lock.acquire()
        if self.task_executing==0 and len(self.high_targetgoals) ==0 and len(self.low_targetgoals) ==0:
            rospy.logwarn("上次运行结束了，需要清除缓存")
            self.task_executing = 0
            self.task_runs_status = False
            self.task_current_wait_time = -1
            self.get_nexttask = False
            self.waitfornavi = False
            self.start_chargecheck = False
            self.start_liftcheck = False
            self.start_putdowncheck = False
            self.subtask_targetgoals = []
            self.high_targetgoals = []
            self.low_targetgoals = []
            self.high_targetgoals_backup = []
            self.low_targetgoals_backup = []
            self.operated_targetGoals = []

        for targetGoal in msg.goal_point_list:
            rospy.loginfo("add_target_goal_CB addnew 楼层(%d,%s), 地图 (%d,%s), 导航点 (%d,%s), type: %d , notes: %s"% \
                    (targetGoal.floor_id, targetGoal.floor_name, targetGoal.map_id, targetGoal.map_name, targetGoal.point_id, targetGoal.point_name, targetGoal.type ,str(targetGoal.notes)))
            # type: 0 导航点, 22：顶, 23：放, 24：充电 
            addoneGoal = TargetGoalEntry()
            #addBackupGoal = TargetGoalEntry()

            mapSetting_bean = MapSetting(self.allMap)
            for floor in mapSetting_bean.getFloors():
                floorbean = Floor(floor)
                if targetGoal.floor_id == floorbean.getFloorId():
                    addoneGoal.floor_name = floorbean.getFloorName()
                    #addBackupGoal.floor_name = floorbean.getFloorName()
                    for map in floorbean.getMaps():
                        mapbean = Map(map)
                        if targetGoal.map_id == mapbean.getMapId():
                            addoneGoal.map_name = mapbean.getMapName()
                            #addBackupGoal.map_name = mapbean.getMapName()
                            # 判断绑定点
                            for point in mapbean.getPoints():
                                pointBean = MapsBeanPoint(point)
                                if targetGoal.point_id == pointBean.getPointId():
                                    addoneGoal.point_name = pointBean.getPointName()
                                    addoneGoal.floor_id_putdown = pointBean.getBindFloorId()
                                    addoneGoal.map_id_putdown = pointBean.getBindMapId()
                                    addoneGoal.point_id_putdown = pointBean.getBindPointId()

                                    #addBackupGoal.point_name = pointBean.getPointName()
                                    #addBackupGoal.floor_id_putdown = pointBean.getBindFloorId()
                                    #addBackupGoal.map_id_putdown = pointBean.getBindMapId()
                                    #addBackupGoal.point_id_putdown = pointBean.getBindPointId()
                                    if pointBean.getLedWarnID():
                                        targetGoal.led_warn_id = int(pointBean.getLedWarnID())
                                    break
                            
                            if addoneGoal.point_id_putdown !=0:
                                targetGoal.type = 22
                                for bindpoint in mapbean.getPoints():
                                    bindpointBean = MapsBeanPoint(bindpoint)
                                    if addoneGoal.point_id_putdown == bindpointBean.getPointId():
                                        addoneGoal.point_name_putdown = bindpointBean.getPointName()
                                        #addBackupGoal.point_name_putdown = bindpointBean.getPointName()
                                        break            
            
            addoneGoal.floor_id = int(targetGoal.floor_id)
            addoneGoal.map_id = int(targetGoal.map_id)
            addoneGoal.point_id = int(targetGoal.point_id)
            if targetGoal.led_warn_id:
                addoneGoal.led_warn_id = targetGoal.led_warn_id

            if targetGoal.type == 22:
                # 顶升
                addoneGoal.type = 22
            else:
                addoneGoal.type = 0

            self.low_targetgoals.append(addoneGoal) # 新增
            
            '''
            addBackupGoal.floor_id = int(targetGoal.floor_id)
            addBackupGoal.map_id = int(targetGoal.map_id)
            addBackupGoal.point_id = int(targetGoal.point_id)
            if targetGoal.led_warn_id:
                addBackupGoal.led_warn_id = targetGoal.led_warn_id
            
            if targetGoal.type == 22:
                # 顶升
                addBackupGoal.floor_id_putdown = int(targetGoal.floor_id_putdown)
                addBackupGoal.map_id_putdown = int(targetGoal.map_id_putdown)
                addBackupGoal.point_id_putdown = int(targetGoal.point_id_putdown)
                addBackupGoal.type = 22
            else:
                addBackupGoal.type = 0

            self.low_targetgoals_backup.append(addBackupGoal)
            '''           

            if targetGoal.type == 24:
                rospy.loginfo("添加充电桩任务到队列...")
                (isfind, chtg) = self.findChargeTargetGoalEntry()
                if isfind: # 跨楼层跨区域充电
                    #tg1_change =copy.deepcopy(chtg)
                    #self.low_targetgoals_backup.append(tg1_change)
                    tg2_change =copy.deepcopy(chtg)
                    self.low_targetgoals.append(tg2_change) 
                else: # 如果没找到普通充电桩则走普通充电导航
                    rospy.loginfo("没有找到充电桩")       

            self.low_targetgoals_backup = copy.deepcopy(self.low_targetgoals)  

        self.target_goal_lock.release()

        rospy.loginfo("low_targetgoals:")
        for goal in self.low_targetgoals:
            rospy.loginfo("TargetGoal_CB low_target goal: floorid: %d floorname: %s mapid: %d mapname: %s pointid: %d pointname: %s", goal.floor_id, str(goal.floor_name), goal.map_id, str(goal.map_name), goal.point_id, str(goal.point_name))

        if self.task_executing == 0:
            self.task_executing = 1
            self.get_nexttask = True
            self.task_runs_status = False
        self.first_targetjump = True

        rospy.logwarn("add_target_goal_CB 添加成功")        
        return
    
    def del_target_goal_CB(self, msg):
        if msg.type == 0 or msg.index < 0:
            rospy.logwarn("del_target_goal_CB 删除不合法, type = %d, index:%d",msg.type, msg.index)
            return

        self.target_goal_lock.acquire()
        if msg.type == 1:
            if msg.index > len(self.operated_targetGoals)-1:
                rospy.logwarn("del_target_goal_CB operated_targetGoals 删除下标不合法 总长度%s",len(self.operated_targetGoals))
                self.target_goal_lock.release()
                return
            self.operated_targetGoals.pop(msg.index)
        if msg.type == 2:
            if msg.index > len(self.high_targetgoals)-1:
                rospy.logwarn("del_target_goal_CB high_targetgoals 删除下标不合法 总长度%s",len(self.high_targetgoals))
                self.target_goal_lock.release()
                return
            if msg.index == 0 and self.high_targetgoals[0].status == 1:
                self.task_stop(False)
                self.task_current_wait_time = -1
                self.high_targetgoals.pop(msg.index)
                self.get_nexttask = True
                self.task_runs_status = False
            else:
                self.high_targetgoals.pop(msg.index)
            
        if msg.type == 3:
            if msg.index > len(self.low_targetgoals)-1:
                rospy.logwarn("del_target_goal_CB low_targetgoals 删除下标不合法 总长度%s",len(self.low_targetgoals))
                self.target_goal_lock.release()
                return
            if msg.index == 0 and self.low_targetgoals[0].status == 1:
                self.task_stop(False)
                self.task_current_wait_time = -1
                self.low_targetgoals.pop(msg.index)
                self.get_nexttask = True
                self.task_runs_status = False
            else:
                self.low_targetgoals.pop(msg.index)
        
        self.target_goal_lock.release()  
        return

    def PathValidateCallback(self, msg):
        if (msg.status == 2):
            # 验证OK，不保存
            #if (self.AndroidAlarm.androidmsg_navigationstatus !=81):
                #self.AndroidmsgReportPub.publish(81)
            #self.androidNavigationStatus_pub.publish(81)
            self.AndroidAlarm.androidmsg_navigationstatus = 81
            rospy.loginfo("机器人导航验证成功！")
            return
        elif (msg.status == 1):
            # 验证失败
            #if (self.AndroidAlarm.androidmsg_navigationstatus !=82):
                #self.AndroidmsgReportPub.publish(82)
            #self.androidNavigationStatus_pub.publish(82)
            self.AndroidAlarm.androidmsg_navigationstatus = 82
            rospy.loginfo("机器人导航验证失败！")
            return
        else:
            # 验证OK，保存路线
            #if (self.AndroidAlarm.androidmsg_navigationstatus !=81):
                #self.AndroidmsgReportPub.publish(81)
            #self.androidNavigationStatus_pub.publish(81)
            self.AndroidAlarm.androidmsg_navigationstatus = 82
            rospy.loginfo("机器人导航路线验证成功，保存路线！")

            # 此代码操作数据集遵循，
            # 先遍历，后备份，
            # 遍历不操作数据，
            # 对备份数据集操作，之后把备份回填对象

            if not self.map_ready:
                rospy.logwarn("map tf can not find")
                return

            if self.map_status == 1 or self.map_status == 2:
                rospy.logwarn("map is build, disenable addShape")
                return

            self.check_map()  # addShap

            if self.allMap is None:
                return

            local_time = self.getId()
            local_time = int(local_time)

            mapSetting = MapSetting(self.allMap)

            # 逐条取出保存，方便后面
            floors = []
            floorindex = -1
            isaddfloorindex = True
            maps = []

            for fll in mapSetting.getFloors():
                if isaddfloorindex:
                    floorindex= floorindex+1
                floor = Floor(fll)
                floors.append(floor)
                
                if (floor.getFloorId() == mapSetting.getDefaultFloor()):
                    # 找到默认楼层, 结束记录下标, 只记录默认地图的下标
                    isaddfloorindex = False
                

                    mapindex = -1
                    isaddindex = True
                    for uu in floor.getMaps():
                        if (isaddindex):
                            mapindex = mapindex + 1
                        map = Map(uu)
                        maps.append(map)
                        # 逐条取出保存
                        if map.getMapId() == floor.getDefaultmap():
                            isaddindex = False
                    rospy.loginfo("globalPlans map size = %d, defatmapId = %d", len(maps), mapindex)
                    globalPlans = []
                    isdell = False
                    ordernum = 10000
                    lastordernum = 0
                    if maps[mapindex].getGlobalPlans():
                        for globalplanarr in maps[mapindex].getGlobalPlans():
                            globalplan = GlobalPlan(globalplanarr)
                            #如果有id,并且与库里相同
                            if msg.globalplanId and  msg.globalplanId == globalplan.getGlobalplanId():
                                isdell =True
                                ordernum = globalplan.getOrderNum()
                                continue
                            if (globalplan.getOrderNum == None):
                                lastordernum = lastordernum+1
                                globalplan.setOrderNum(lastordernum)
                            if (globalplan.getOrderNum() > ordernum):
                                globalplan.setOrderNum(globalplan.getOrderNum() - 1)
                            lastordernum = globalplan.getOrderNum()
                            globalPlans.append(globalplan)

                    #添加手动全局路径点
                    if not isdell:
                        globalplan = GlobalPlan()
                        globalplan.setGlobalplanId(local_time)
                        globalplan.setDir(msg.dir)
                        globalplan.setGlobalplanName(msg.globalplanName)
                        globalplan.setOrderNum(lastordernum+1)
                        positions = []
                        for point in msg.points:
                            position = Position()
                            position.setX(point.x)
                            position.setY(point.y)
                            positions.append(position)
                        globalplan.setLines(positions)
                        globalPlans.append(globalplan)

            # 回填map的globalplans
            maps[mapindex].setGlobalPlans(globalPlans)
            floors[floorindex].setMaps(maps)
            mapSetting.setFloors(floors)
            self.write_json(mapSetting, spirit_map_url)
            self.wall_update_pub.publish(0)
            rospy.loginfo("添加一条全局路线成功。")
            return

    def NPathAddOrDelCallback(self, msg):
            # 此代码操作数据集遵循，
            # 先遍历，后备份，
            # 遍历不操作数据，
            # 对备份数据集操作，之后把备份回填对象

        if not self.map_ready:
            rospy.logwarn("map tf can not find NPath")
            return

        if self.map_status == 1 or self.map_status == 2:
            rospy.logwarn("map is build, disenable NPath")
            return
        self.check_map()  # NPath
        if self.allMap is None:
            return
        local_time = self.getId()
        local_time = int(local_time)
        mapSetting = MapSetting(self.allMap)

        # 逐条取出保存，方便后面
        floors = []
        floorindex = -1
        isaddfloorindex = True
        maps = []

        for fll in mapSetting.getFloors():
            if isaddfloorindex:
                floorindex= floorindex+1
            floor = Floor(fll)
            floors.append(floor)
            if (floor.getFloorId() == mapSetting.getDefaultFloor()):
                # 找到默认楼层, 结束记录下标, 只记录默认地图的下标
                isaddfloorindex = False
                rospy.loginfo("NPathAddOrDelCallback 找到默认楼层")

                mapindex = -1
                isaddindex = True
                for uu in floor.getMaps():
                    if (isaddindex):
                        mapindex = mapindex + 1
                    map = Map(uu)
                    maps.append(map)
                    # 逐条取出保存
                    if map.getMapId() == floor.getDefaultmap():
                        isaddindex = False
                rospy.loginfo("NPath map size = %d, defatmapId = %d", len(maps), mapindex)
                nglobalPlan = NGlobalPlans()
                nLines =[]
                isdell = False
                isupdate = False
                if maps[mapindex].getNGlobalPlans():
                    nglobalPlan =maps[mapindex].getNGlobalPlans()
                    if nglobalPlan:
                        if nglobalPlan.getNLines():
                            for nLineLocal in nglobalPlan.getNLines():
                                nLineBean = NLine(nLineLocal)
                                #如果有id,并且与库里相同认为是删除或者修改，type不为空则为修改
                                if msg.id and  msg.id == nLineBean.getId():
                                    if msg.type != -1:
                                        isupdate = True
                                        nLineBean.setType(msg.type)
                                        nLineBean.setDirection(msg.direction)
                                        nLineBean.setSpeed(msg.speed)
                                        nLineBean.setWeight(msg.weight)
                                    else:
                                        isdell =True
                                        continue
                                nLines.append(nLineBean)
                            nglobalPlan.setNLines(nLines)
                if not isdell and not isupdate:
                    nLineBean = NLine()
                    nLineBean.setId(local_time)
                    nLineBean.setStartNid(msg.startNid)
                    nLineBean.setEndNid(msg.endNid)
                    nLineBean.setType(msg.type)
                    nLineBean.setDirection(msg.direction)
                    nLineBean.setSpeed(msg.speed)
                    nLineBean.setWeight(msg.weight)
                    nLines.append(nLineBean)
                    nglobalPlan.setNLines(nLines)
                # 回填时候如果被删除路线后的点孤立了，则把点也删掉
                if isdell:
                    if maps[mapindex].getPoints():
                        points = []
                        for pointname in maps[mapindex].getPoints():
                                point = MapsBeanPoint(pointname)
                                if point.getPointName() == CANCLEPOINTNAME:
                                    isdel2 = True
                                    for nLineLocal2 in nglobalPlan.getNLines():
                                        # nLineBean2 = NLine(nLineLomapserver_manager.py ,sid =  ")
                                        if point.getPointId() == nLineLocal2.getStartNid() or point.getPointId() == nLineLocal2.getEndNid():
                                            isdel2 = False
                                            break
                                    if isdel2:
                                        continue
                                points.append(point)
                        maps[mapindex].setPoints(points)


        # 回填map的globalplans
        maps[mapindex].setNGlobalPlans(nglobalPlan)
        floors[floorindex].setMaps(maps)
        mapSetting.setFloors(floors)
        self.write_json(mapSetting, spirit_map_url)
        self.wall_update_pub.publish(0)
        rospy.loginfo("添加一条nLine成功。")
        self.check_map()  # NPath 刷新算法中的 self.local_json*
        return

    def NetworkbeatCallback(self, msg):
        if (msg.data == 1):
            # 每10分钟调用一次
            if self.isMove == False:
                self.connectwifi()

        if (msg.data == 3):
            # 每2s检查一次
            self.heartbeat_pub.publish(self.map_driverinfo.ChargeStatus)

            if (self.scan_ready == False and self.network_report >= 6):
                # 激光数据接收异常
                if (self.AndroidAlarm.androidmsg_lidarstatus !=13):
                    #self.AndroidmsgReportPub.publish(13)
                    self.androidLidarStatus_pub.publish(13)
                    self.AndroidAlarm.androidmsg_lidarstatus = 13
                    rospy.loginfo("机器人激光模块发送数据异常，导航模块不工作！")
            if (self.imu_ready == False and self.network_report >= 6):
                # IMU数据接收异常
                if (self.AndroidAlarm.androidmsg_imustatus !=15):
                    #self.AndroidmsgReportPub.publish(15)
                    self.androidImuStatus_pub.publish(15)
                    self.AndroidAlarm.androidmsg_imustatus = 15
                    rospy.loginfo("机器人IMU模块发送数据异常，导航模块不工作！")
            if (self.map_ready == False and self.network_report >= 6):
                # 地图数据接收异常
                if (self.AndroidAlarm.androidmsg_mapstatus !=61):
                    #self.AndroidmsgReportPub.publish(61)
                    self.androidMapStatus_pub.publish(61)
                    self.AndroidAlarm.androidmsg_mapstatus = 61
                    rospy.loginfo("机器人地图加载失败。")
            if (self.odom_ready == False and self.network_report >= 6):
                # 里程计数据接收异常
                if (self.AndroidAlarm.androidmsg_odomstatus !=19):
                    #self.AndroidmsgReportPub.publish(19)
                    self.androidOdomStatus_pub.publish(19)
                    self.AndroidAlarm.androidmsg_odomstatus = 19
                    rospy.loginfo("机器人里程计发送数据失败，可能是通讯出现了问题或者配置文件丢失！")

            # 在运动中，进行位置保存
            if self.isMove:
                if self.save_position_flag !=2:
                    self.save_position_flag = 1      

            if (self.network_report < 6):
                self.network_report  = self.network_report  + 1      

        if (msg.data == 4):
            # 1分钟检查一次
            # 检查48小时未关机，在充电中重启
            if (self.map_driverinfo.ChargeStatus > 0) and (self.map_driverinfo.ChargeStatus < 5):
                if self.reboot_time >= 2:
                    rospy.loginfo("机器人48小时未关机，正在充电中，机器人开始重启！！！")
                    os.system("reboot")

            # 检查超声波是否数据正常
            if (self.sonar_flag == 2 and self.use_Sonar == True and self.network_report >= 6):
                # 超声波数据接收异常
                self.sonar_flag = 0
                if (self.AndroidAlarm.androidmsg_sonarstatus !=105):
                    #self.AndroidmsgReportPub.publish(105)
                    self.androidSonarStatus_pub.publish(105)
                    self.AndroidAlarm.androidmsg_sonarstatus = 105
                    rospy.loginfo("机器人超声波板出现故障，有可能是硬件故障或者连线出现问题！")
            elif (self.sonar_flag == 1):
                self.sonar_flag = 2

            # 检查通讯是否正常
            if (self.driver_ready == 2 and self.network_report >= 6):
                self.driver_ready = 0
                # 驱动STM32通讯异常
                if (self.AndroidAlarm.androidmsg_stm32status !=17):
                    #self.AndroidmsgReportPub.publish(17)
                    self.androidStm32Status_pub.publish(17)
                    self.AndroidAlarm.androidmsg_stm32status = 17
                    rospy.loginfo("机器人导航模块跟STM32底板通讯失败，可能是驱动线存在松动，请重启机器尝试恢复！")
            elif (self.driver_ready == 1):
                self.driver_ready = 2

        if (msg.data == 2):
            # 每24小时调用一次
            self.reboot_time = self.reboot_time + 1
            os.system("find /root/.ros/log -mtime +3 | xargs rm -rf")

    def RelocationPoseCallback(self, msg):
        rospy.loginfo("re-location start!")
        # 重定位
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = msg.point_X
        pose.pose.pose.position.y = msg.point_Y
        q = transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]
        self.initpose_pub.publish(pose)
        rospy.loginfo("机器人发送重定位消息给cartographer成功。")

    def child_thread_func(self):
        a = True
        while a:
            if self.isheart:
                self.isheart = False
                time.sleep(10)
            else:
                a = False
                if (self.record_bag_type != 4):  # 如果是主动退出则不关闭
                    rospy.logwarn("录制bag包心跳停止, bag包录制退出!")
                    self.record_bag_type = 4
                    self.recordbag.type = 4
                    self.sendToRecordbag_pub.publish(self.recordbag)
                # self.t1.end() # 子线程结束
        rospy.loginfo("bag包录制成功, 停止录制线程。")

    # int type;
    # string bag_name;
    # type:
    # =1:  开始录制
    # =2:  暂停录制
    # =3:  取消暂停
    # =4： 停止录制
    # =5： 心跳
    # =6： 获取bag包文件列表
    def RecordBag_CB(self, msg):
        rospy.loginfo("Recordbag msg received, type: %d", msg.type)
        self.recordbag = msg
        if self.recordbag.config is None or self.recordbag.config =="":
            self.recordbag.config = "spirit"

        if (msg.type == 1):
            # 开始录制
            rospy.loginfo("开始录制bag包...")
            BAG_PATH = bag_param
            if not os.path.exists(BAG_PATH):
                os.mkdir(BAG_PATH)

            self.record_bag_type = 1
            self.isheart = True
            self.sendToRecordbag_pub.publish(self.recordbag)
            # 通过 target=函数名 的方式定义子线程
            # 通过 target 指定子线程要执行的任务。可以通过 args=元组 来指定该函数的参数
            self.t1 = threading.Thread(target=self.child_thread_func)
            # start 之后，子线程开始执行，主线程也会继续执行，不会等待
            self.t1.start()  # 子线程执行
        elif (msg.type == 4):
            # 停止录制
            rospy.loginfo("停止录制bag包, bag录制完成, 保存在/home/spirit/.ros/maps/bag/目录下。")
            self.record_bag_type = 4
            self.isheart = False
            self.sendToRecordbag_pub.publish(self.recordbag)
        elif (msg.type == 5):
            # 心跳
            self.isheart = True
            self.record_bag_type = 5

    def DriverinfoCallback(self, msg):  # driver状态
        # Dipaninitializa signal
        # EmergencyStatus 紧停
        # MotorInitStatus 1,驱动就绪
        # charge status report

        # 关机 或者 重启
        if (msg.Shutdown ==1):
            self.shutdown = 1
            self.robot_reboot_pub.publish(3)
        elif (msg.Shutdown ==2):
            self.shutdown = 2
            self.robot_reboot_pub.publish(1)

        # check STM32 connection
        if (self.driver_ready == 0):
            # 驱动STM32通讯正常
            if (self.AndroidAlarm.androidmsg_stm32status != 18):
                #self.AndroidmsgReportPub.publish(18)
                self.androidStm32Status_pub.publish(18)
                self.AndroidAlarm.androidmsg_stm32status = 18
            self.driver_ready = 1
        if (self.driver_ready == 2):
            self.driver_ready = 1

        # power report
        if ((msg.Power != self.map_driverinfo.Power)):
            if (self.map_driverinfo.ChargeStatus == 0):
                #未充电模式
                if (self.map_driverinfo.Power > msg.Power):
                    self.map_driverinfo.Power = msg.Power
                    rospy.loginfo("power is: %d", msg.Power)
                    self.PowerReportPub.publish(msg.Power)
                elif (self.map_driverinfo.Power + 5 < msg.Power):
                    self.map_driverinfo.Power = msg.Power
                    rospy.loginfo("power is: %d", msg.Power)
                    self.PowerReportPub.publish(msg.Power)
            elif((self.map_driverinfo.ChargeStatus > 0) and (self.map_driverinfo.ChargeStatus < 5)):
                #充电模式
                if (self.map_driverinfo.Power < msg.Power):
                    self.map_driverinfo.Power = msg.Power
                    rospy.loginfo("power is: %d", msg.Power)
                    self.PowerReportPub.publish(msg.Power)
                elif (self.map_driverinfo.Power > (msg.Power + 5)):
                    self.map_driverinfo.Power = msg.Power
                    rospy.loginfo("power is: %d", msg.Power)
                    self.PowerReportPub.publish(msg.Power)

        # left motor alarm
        if ((msg.LeftAlarm != self.map_driverinfo.LeftAlarm)):
            self.map_driverinfo.LeftAlarm = msg.LeftAlarm
            rospy.loginfo("Left ALarm change to: %d", msg.LeftAlarm)
            if (msg.LeftAlarm == 0):
                rospy.loginfo("机器人左电机正常。")
                #self.AndroidmsgReportPub.publish(108)
                self.androidLeftmotorStatus_pub.publish(108)
                self.AndroidAlarm.androidmsg_leftmotorstatus = 108
            else:
                rospy.loginfo("机器人左电机出现故障，请检查左电机！")
                #self.AndroidmsgReportPub.publish(107)
                self.androidLeftmotorStatus_pub.publish(107)
                self.AndroidAlarm.androidmsg_leftmotorstatus = 107

        # right motor alarm
        if ((msg.RightAlarm != self.map_driverinfo.RightAlarm)):
            self.map_driverinfo.RightAlarm = msg.RightAlarm
            rospy.loginfo("Right ALarm change to: %d", msg.RightAlarm)
            if (msg.RightAlarm == 0):
                rospy.loginfo("机器人右电机出现故障，请检查右电机！")
                #self.AndroidmsgReportPub.publish(110)
                self.androidRightmotorStatus_pub.publish(110)
                self.AndroidAlarm.androidmsg_rightmotorstatus = 110
            else:
                rospy.loginfo("机器人右电机正常。")
                #self.AndroidmsgReportPub.publish(109)
                self.androidRightmotorStatus_pub.publish(109)
                self.AndroidAlarm.androidmsg_rightmotorstatus = 109

        # move alarm & stop alarm
        if ((msg.MoveAlarm != self.map_driverinfo.MoveAlarm)):
            self.map_driverinfo.MoveAlarm = msg.MoveAlarm
            rospy.loginfo("MoveAlarm change to: %d", msg.MoveAlarm)
            if (msg.MoveAlarm == 0):
                rospy.loginfo("机器人轮子响应正常。")
                #self.AndroidmsgReportPub.publish(120)
                self.androidMotor2Status_pub.publish(120)
                self.AndroidAlarm.androidmsg_motor2status = 120
            else:
                rospy.loginfo("机器人在有移动指令情况下，轮子静止不动，有可能轮子被卡住了，请检查轮子！")
                #self.AndroidmsgReportPub.publish(119)
                self.androidMotor2Status_pub.publish(119)
                self.AndroidAlarm.androidmsg_motor2status = 119
        if ((msg.StopAlarm != self.map_driverinfo.StopAlarm)):
            self.map_driverinfo.StopAlarm = msg.StopAlarm
            rospy.loginfo("StopAlarm change to: %d", msg.StopAlarm)
            if (msg.StopAlarm == 0):
                rospy.loginfo("机器人在没有指令情况下，轮子静止，状态正常。")
                #self.AndroidmsgReportPub.publish(102)
                self.androidMotor1Status_pub.publish(102)
                self.AndroidAlarm.androidmsg_motor1status = 102
            else:
                rospy.loginfo("机器人在没有指令情况下，轮子依然在转动，危险！")
                #self.AndroidmsgReportPub.publish(101)
                self.androidMotor1Status_pub.publish(101)
                self.AndroidAlarm.androidmsg_motor1status = 101

        # record speed and encoder
        self.map_driverinfo.Speedx = msg.Speedx
        self.map_driverinfo.Speedth = msg.Speedth
        self.map_driverinfo.LeftEncoder = msg.LeftEncoder
        self.map_driverinfo.RightEncoder = msg.RightEncoder

        if (abs(self.map_driverinfo.Speedx) > 2 or abs(self.map_driverinfo.Speedth) > 2):
            self.isMove = True
        else:
            if self.isMove:
                if self.save_position_flag !=2:
                    self.save_position_flag = 1
            self.isMove = False

        if ((self.initial_set_flag or self.charge_relocation_change_map_flag) and self.map_driverinfo.ChargeStatus > 0 and self.map_driverinfo.ChargeStatus < 5):
            #rospy.loginfo("enter initial set, timeout: %d", self.initial_set_timeout)
            if (self.initial_set_timeout > 300):
                self.initial_set_flag = False
                self.charge_relocation_change_map_flag = False
                self.initial_set_timeout = 0
                if (self.map_status == 0):
                    self.relocation_set(1)
                elif (self.map_status == 1 or self.map_status == 2):
                    self.initialpose_set()
            else:
                self.initial_set_timeout = self.initial_set_timeout + 1
        else:
            # 不进行重定位
            self.initial_set_flag = False
            self.initial_set_timeout = 0
        
        # charge status report
        if ((msg.ChargeStatus != self.map_driverinfo.ChargeStatus)):
            self.map_driverinfo.ChargeStatus = msg.ChargeStatus
            self.PowerReportPub.publish(msg.Power)
            #rospy.loginfo("Charge status change to: %d, power: %d", msg.ChargeStatus, msg.Power)

            #### 41:  没有充电 42:  充电中 43： 充电完成 44： 充电失败 45:  自动充电中 46： 手动充电中 47： 自动充电完成 48:  手动充电完成
            need_send_initial_pose = 0
            self.chargeStatus_pub.publish(msg.ChargeStatus)
            charge_mode = 0
            if (msg.ChargeStatus == 0):
                rospy.loginfo("机器人处于未充电模式")
                self.laser_switch_timeout = 0
                if (self.laser_switch == 0):
                    rospy.loginfo("底盘处于未充电模式，驱动开启激光！！！")
                    self.laser_switch = 1
                    self.laserswitch_pub.publish(1)

                #self.AndroidmsgReportPub.publish(41)
                self.androidChargeStatus_pub.publish(41)
                self.AndroidAlarm.androidmsg_chargestatus = 41
                charge_mode = 2
            elif ((msg.ChargeStatus == 1)):
                rospy.loginfo("机器人处于自动充电状态。")
                #self.AndroidmsgReportPub.publish(45)
                self.androidChargeStatus_pub.publish(45)
                self.AndroidAlarm.androidmsg_chargestatus = 45
                charge_mode = 50
                need_send_initial_pose = 1
            elif ((msg.ChargeStatus == 2)):
                rospy.loginfo("机器人处于手动充电状态。")
                #self.AndroidmsgReportPub.publish(46)
                self.androidChargeStatus_pub.publish(46)
                self.AndroidAlarm.androidmsg_chargestatus = 46
                charge_mode = 50
                need_send_initial_pose = 1
            elif ((msg.ChargeStatus == 3)):
                rospy.loginfo("机器人自动充电已经完成，电量100%。")
                #self.AndroidmsgReportPub.publish(45)
                self.androidChargeStatus_pub.publish(45)
                self.AndroidAlarm.androidmsg_chargestatus = 45
                #self.AndroidmsgReportPub.publish(47)
                self.androidChargeStatus_pub.publish(47)
                self.AndroidAlarm.androidmsg_chargestatus = 47
                charge_mode = 50
                need_send_initial_pose = 1
            elif ((msg.ChargeStatus == 4)):
                rospy.loginfo("机器人手动充电已经完成，电量100%。")
                #self.AndroidmsgReportPub.publish(46)
                self.androidChargeStatus_pub.publish(46)
                self.AndroidAlarm.androidmsg_chargestatus = 46
                #self.AndroidmsgReportPub.publish(48)
                self.androidChargeStatus_pub.publish(48)
                self.AndroidAlarm.androidmsg_chargestatus = 48
                charge_mode = 50
                need_send_initial_pose = 1
            elif (msg.ChargeStatus == 5):
                rospy.loginfo("机器人充电失败，未充电！请及时修复，以防低电关机！")
                #self.AndroidmsgReportPub.publish(44)
                self.androidChargeStatus_pub.publish(44)
                self.AndroidAlarm.androidmsg_chargestatus = 44
                charge_mode = 2

            if ((self.send_initial_pose != need_send_initial_pose) and (need_send_initial_pose == 1) and self.mode != 1):
                rospy.loginfo("robot is manuput into charge point set, need relocation...")
                self.initial_set_flag = True

            self.send_initial_pose = need_send_initial_pose
            self.mode = charge_mode

        if (msg.ChargeStatus > 0 and msg.ChargeStatus < 5):
            if (self.laser_switch == 1):
                self.laser_switch_timeout = self.laser_switch_timeout + 1
                if self.laser_switch_timeout > 3000:
                    # 延迟三十秒关闭激光
                    self.laser_switch = 0
                    self.laserswitch_pub.publish(0)
                    rospy.loginfo("机器人在充电中，已经延时：%d秒， 开始关闭激光！", self.laser_switch_timeout/100)
                    self.laser_switch_timeout = 0


        # mode change
        if (msg.Mode != self.map_driverinfo.Mode):
            if (msg.Mode ==2) and (self.map_driverinfo.ChargeStatus == 0):
                rospy.loginfo("机器人出桩成功！")
                #self.AndroidmsgReportPub.publish(50)
                self.androidOutofChargepoint_pub .publish(50)
                self.AndroidAlarm.androidmsg_outofchargepoint = 50

            #rospy.loginfo("driver.mode: %d, msg.mode: %d", self.map_driverinfo.Mode, msg.Mode)
            self.map_driverinfo.Mode = msg.Mode

        # Emergerncy signal
        if ((msg.EmergencyStatus != self.map_driverinfo.EmergencyStatus)):
            self.map_driverinfo.EmergencyStatus = msg.EmergencyStatus
            rospy.loginfo("Emergerncy status change to: %d", msg.EmergencyStatus)
            #### 31:  急停
            #### 32:  取消急停
            if (msg.EmergencyStatus == 1):
                #self.AndroidmsgReportPub.publish(31)
                self.androidEmergencyStatus_pub.publish(31)
                self.AndroidAlarm.androidmsg_emergencystatus = 31
                rospy.loginfo("机器人处于急停状态！机器人电机锁住不动！")
            else:
                #self.AndroidmsgReportPub.publish(32)
                self.androidEmergencyStatus_pub.publish(32)
                self.AndroidAlarm.androidmsg_emergencystatus = 32
                rospy.loginfo("机器人急停状态取消。")
        
        # MotorEnabled 
        if ((msg.MotorEnabled != self.map_driverinfo.MotorEnabled)):
            self.map_driverinfo.MotorEnabled = msg.MotorEnabled
            rospy.loginfo("MotorEnabled status change to: %d", msg.MotorEnabled)
            #### 33:  电机锁
            #### 34:  取消电机锁
            if (msg.MotorEnabled == 1):
                self.androidMotorEnabled_pub.publish(33)
                self.AndroidAlarm.androidmsg_motorenabledstatus = 31
                rospy.loginfo("机器人处于电机锁状态！机器人电机自由旋转！小心坡路安全")
            else:
                self.androidMotorEnabled_pub.publish(34)
                self.AndroidAlarm.androidmsg_motorenabledstatus = 32
                rospy.loginfo("机器人电机锁状态取消。上电抱紧")

        # initial_status
        if (msg.MotorInitStatus != self.map_driverinfo.MotorInitStatus ):
            rospy.loginfo("STM32板卡初始化正常。")
            rospy.loginfo("MotorInitStatus change to: %d", msg.MotorInitStatus)
            self.map_driverinfo.MotorInitStatus = msg.MotorInitStatus


    ##########################################################################
    # Services callbacks
    ##########################################################################

    def set_virtualwall_cb(self,request):
        # 此代码操作数据集遵循，
        # 先遍历，后备份，
        # 遍历不操作数据，
        # 对备份数据集操作，之后把备份回填对象
        rospy.loginfo("service call :virtual wall add type %s",request.type)
        shaperesponse = SetShapeResponse()
        shaperesponse.success =False

        if not self.map_ready:
            rospy.logwarn("map tf can not find")
            return shaperesponse

        if self.map_status == 1 or self.map_status == 2:
            rospy.logwarn("map is build, disenable addShape")
            return shaperesponse

        self.check_map()  # addShap

        if self.allMap is None:
            return shaperesponse

        local_time = self.getId()
        local_time = int(local_time)

        mapSetting = MapSetting(self.allMap)

        # 逐条取出保存，方便后面
        floors = []
        floorindex = -1
        isaddfloorindex = True
        maps = []
        
        for fll in mapSetting.getFloors():
            if isaddfloorindex:
                floorindex= floorindex+1
            floor = Floor(fll)
            floors.append(floor)
            if (floor.getFloorId() == mapSetting.getDefaultFloor()):
                # 找到默认楼层, 结束记录下标, 只记录默认地图的下标
                isaddfloorindex = False

                mapindex = -1
                isaddindex = True
                for uu in floor.getMaps():
                    if (isaddindex):
                        mapindex = mapindex + 1
                    map = Map(uu)
                    maps.append(map)
                    # 逐条取出保存
                    if map.getMapId() == floor.getDefaultmap():
                        isaddindex = False
                rospy.loginfo("set_virtualwall map size = %d, defatmapId = %d", len(maps), mapindex)
                shapeAreas = []
                isdell = False
                ordernum = 10000
                lastordernum = 0
                if maps[mapindex].getShapeAreas():
                    for shapearr in maps[mapindex].getShapeAreas():
                        shape = Shape(shapearr)
                        #如果有id,并且与库里相同，则认为是删除虚拟墙
                        if request.shape_id and  request.shape_id == shape.getShapeId():
                            isdell =True
                            ordernum = shape.getOrderNum()
                            continue
                        if (shape.getOrderNum == None):
                            lastordernum = lastordernum+1
                            shape.setOrderNum(lastordernum)
                        if (shape.getOrderNum() > ordernum):
                            shape.setOrderNum(shape.getOrderNum() - 1)
                        lastordernum = shape.getOrderNum()
                        shapeAreas.append(shape)

                #添加虚拟墙
                if not isdell:
                    shape = Shape()
                    shape.setShapeId(local_time)
                    shape.setType(request.type)
                    shape.setOrderNum(lastordernum+1)
                    shape.setClosed(request.closed)
                    if request.type =="lines":
                        positions = []
                        for point in request.points:
                            position = Position()
                            position.setX(point.x)
                            position.setY(point.y)
                            positions.append(position)
                        shape.setLines(positions)
                    if request.type =="circle":
                        circle = Circle()
                        circle.setRadius(request.radius)
                        position = Position()
                        position.setX(request.points[0].x)
                        position.setY(request.points[0].y)
                        circle.setPosition(position)
                        shape.setCircle(circle)
                    shapeAreas.append(shape)

                # 回填map的shapeAreas
                maps[mapindex].setShapeAreas(shapeAreas)
        floors[floorindex].setMaps(maps)
        mapSetting.setFloors(floors)
        self.write_json(mapSetting, spirit_map_url)  # shape set
        self.wall_update_pub.publish(0) #wall update
        shaperesponse.success = True
        rospy.loginfo("添加一条虚拟墙成功。")
        return shaperesponse

    def set_region_cb(self, request):
        rospy.loginfo("service call :set_region_cb add type %s",request.type)
        regionresponse = SetRegionResponse()
        regionresponse.success =False

        if not self.map_ready:
            rospy.logwarn("map tf can not find")
            return regionresponse

        if self.map_status == 1 or self.map_status == 2:
            rospy.logwarn("map is build, disenable addRegion")
            return regionresponse

        self.check_map()  # addRegion

        if self.allMap is None:
            return regionresponse

        local_time = self.getId()
        local_time = int(local_time)

        mapSetting = MapSetting(self.allMap)

        # 逐条取出保存，方便后面
        floors = []
        floorindex = -1
        isaddfloorindex = True
        maps = []
        
        for fll in mapSetting.getFloors():
            if isaddfloorindex:
                floorindex= floorindex+1
            floor = Floor(fll)
            floors.append(floor)
            if (floor.getFloorId() == mapSetting.getDefaultFloor()):
                # 找到默认楼层, 结束记录下标, 只记录默认地图的下标
                isaddfloorindex = False

                mapindex = -1
                isaddindex = True
                for uu in floor.getMaps():
                    if (isaddindex):
                        mapindex = mapindex + 1
                    map = Map(uu)
                    maps.append(map)
                    # 逐条取出保存
                    if map.getMapId() == floor.getDefaultmap():
                        isaddindex = False
                rospy.loginfo("set_region map size = %d, defatmapId = %d", len(maps), mapindex)
                regionList = []
                isdell = False
                ordernum = 10000
                lastordernum = 0
                if maps[mapindex].getRegionList():
                    for regionn in maps[mapindex].getRegionList():
                        region = Region(regionn)
                        #如果有id,并且与库里相同，则认为是删除虚拟墙
                        if request.region_id and  request.region_id == region.getRegionId():
                            isdell =True
                            ordernum = region.getOrderNum()
                            continue # 跳过添加缓存
                        if (region.getOrderNum == None):
                            lastordernum = lastordernum+1
                            region.setOrderNum(lastordernum)
                        if (region.getOrderNum() > ordernum):
                            region.setOrderNum(region.getOrderNum() - 1)
                        lastordernum = region.getOrderNum()
                        regionList.append(region)

                #添加区域
                if not isdell:
                    region = Region()
                    region.setRegionId(local_time)
                    region.setType(request.type)
                    region.setOrderNum(lastordernum+1)
                    
                    positions = [] # 多边形连线点
                    for point in request.points:
                        position = Position()
                        position.setX(point.x)
                        position.setY(point.y)
                        positions.append(position)
                    region.setPoints(positions)
                    region.setStartPointId(request.start_point_id)
                    region.setEndPointId(request.end_point_id)
                    region.setWaitPointId(request.wait_point_id)
                    region.setRegionProperty1(request.region_property1)
                    region.setRegionProperty2(request.region_property2)
                    regionList.append(region)

                # 回填map的 setRegionList
                maps[mapindex].setRegionList(regionList)
                if request.region_property2 != 0:
                    maps[mapindex].setMapFloorNum(request.region_property2) #注意，这里借用了一个属性，设置电梯楼层

        floors[floorindex].setMaps(maps)
        mapSetting.setFloors(floors)
        self.write_json(mapSetting, spirit_map_url)  # region set
        self.wall_update_pub.publish(0) # region update
        regionresponse.success = True
        rospy.loginfo("添加特殊区域成功。")



        return regionresponse

    def set_cleanArea_cb(self,request):
        # 此代码操作数据集遵循，
        # 先遍历，后备份，
        # 遍历不操作数据，
        # 对备份数据集操作，之后把备份回填对象
        rospy.loginfo("service call :cleanArea add type %s",request.type)
        cleanAreasponse = SetCleanAreaResponse()
        cleanAreasponse.success =False

        if not self.map_ready:
            rospy.logwarn("map moready, disenable cleanArea")
            return cleanAreasponse

        if self.map_status == 1 or self.map_status == 2:
            rospy.logwarn("map is build, disenable cleanArea")
            return cleanAreasponse

        self.check_map()  # cleanArea

        if self.allMap is None:
            return cleanAreasponse

        local_time = self.getId()
        local_time = int(local_time)

        mapSetting = MapSetting(self.allMap)

        # 逐条取出保存，方便后面
        floors = []
        floorindex = -1
        isaddfloorindex = True
        maps = []
        for fll in mapSetting.getFloors():
            if isaddfloorindex:
                floorindex= floorindex+1
            floor = Floor(fll)
            floors.append(floor)
            if (floor.getFloorId() == mapSetting.getDefaultFloor()):
                # 找到默认楼层, 结束记录下标, 只记录默认地图的下标
                isaddfloorindex = False

                mapindex = -1
                isaddindex = True
                for uu in floor.getMaps():
                    if (isaddindex):
                        mapindex = mapindex + 1
                    map = Map(uu)
                    maps.append(map)
                    # 逐条取出保存
                    if map.getMapId() == floor.getDefaultmap():
                        isaddindex = False
                rospy.loginfo("set_cleanArea map size = %d, defatmapId = %d", len(maps), mapindex)
                cleanAreas = []
                isdell = False
                ordernum = 10000
                lastordernum = 0
                if maps[mapindex].getCleanAreas():
                    for shapearr in maps[mapindex].getCleanAreas():
                        cleanArea = CleanArea(shapearr)
                        #如果有id,并且与库里相同，则认为是删除清洁区域
                        if request.cleanAreaId and  request.cleanAreaId == cleanArea.getCleanAreaId():
                            isdell =True
                            ordernum = cleanArea.getSequenceId()
                            continue
                        if (cleanArea.getSequenceId() == None):
                            lastordernum = lastordernum+1
                            cleanArea.setSequenceId(lastordernum)
                        if (cleanArea.getSequenceId() > ordernum):
                            cleanArea.setSequenceId(cleanArea.getSequenceId() - 1)
                        lastordernum = cleanArea.getSequenceId()
                        cleanAreas.append(cleanArea)
                #添加清洁区域
                if not isdell:
                    cleanArea = CleanArea()
                    cleanArea.setCleanAreaId(local_time)
                    cleanArea.setSequenceId(lastordernum+1)
                    cleanArea.setCleanAreaName(request.name)
                    positions = []
                    for point in request.points:
                        position = Position()
                        position.setX(point.x)
                        position.setY(point.y)
                        positions.append(position)
                    cleanArea.setLines(positions)
                    cleanAreas.append(cleanArea)

                # 回填 map 的 cleanAreas
                maps[mapindex].setCleanAreas(cleanAreas)
        floors[floorindex].setMaps(maps)
        mapSetting.setFloors(floors)
        self.write_json(mapSetting, spirit_map_url)  # cleanArea set
        self.wall_update_pub.publish(0) # cleanArea update
        cleanAreasponse.success = True
        rospy.loginfo("添加一清洁区域成功。")
        return cleanAreasponse

    def import_config_cb(self,request):
        rospy.loginfo("service call :import configs.")
        importconfigsresponse = ImportConfigsResponse()
        importconfigsresponse.status =False

        if not self.map_ready:
            rospy.logwarn("map tf can not find")
            return importconfigsresponse

        if self.map_status == 1 or self.map_status == 2:
            rospy.logwarn("map is build, disenable import configs")
            return importconfigsresponse
        #self.androidImportfilesStatus_pub.publish(1)
        rospy.loginfo("导入地图配置成功。")
        return importconfigsresponse

    def set_eraser_cb(self,request):
        rospy.loginfo("service call :set eraser is called!")
        seteraserresponse = SetEraserModeResponse()
        if (request.erasermode == 1):
            rospy.loginfo("设置橡皮檫为小圆点模式！")
            self.erasermode = 1
        elif (request.erasermode == 3):
            rospy.loginfo("设置橡皮檫为大圆点模式！")
            self.erasermode = 3
        else:
            rospy.loginfo("设置橡皮檫为中圆点模式！")
            self.erasermode = 2

        return seteraserresponse
    
    def addpoints_globalplan_cb(self,request):
        # 此代码操作数据集遵循，
        # 先遍历，后备份，
        # 遍历不操作数据，
        # 对备份数据集操作，之后把备份回填对象
        rospy.loginfo("service call :addpoints to globalplan!")
        addpointsGlobalplanresponse = AddpointsGlobalplanResponse()
        addpointsGlobalplanresponse.result = False

        if not self.map_ready:
            rospy.logwarn("map tf can not find")
            return addpointsGlobalplanresponse

        if self.map_status == 1 or self.map_status == 2:
            rospy.logwarn("map is build, disenable addShape")
            return addpointsGlobalplanresponse

        self.check_map()  # addShap

        if self.allMap is None:
            return addpointsGlobalplanresponse

        mapSetting = MapSetting(self.allMap)

        # 逐条取出保存，方便后面
        floors = []
        floorindex = -1
        isaddfloorindex = True
        maps = []

        for fll in mapSetting.getFloors():
            if isaddfloorindex:
                floorindex= floorindex+1
            floor = Floor(fll)
            floors.append(floor)
            if (floor.getFloorId() == mapSetting.getDefaultFloor()):
                # 找到默认楼层, 结束记录下标, 只记录默认地图的下标
                isaddfloorindex = False

                mapindex = -1
                isaddindex = True
                for uu in floor.getMaps():
                    if (isaddindex):
                        mapindex = mapindex + 1
                    map = Map(uu)
                    maps.append(map)
                    # 逐条取出保存
                    if map.getMapId() == floor.getDefaultmap():
                        isaddindex = False
                rospy.loginfo("globalPlans map size = %d, defatmapId = %d", len(maps), mapindex)

                globalPlans = []

                if maps[mapindex].getGlobalPlans():
                    for globalplanarr in maps[mapindex].getGlobalPlans():
                        globalplan = GlobalPlan(globalplanarr)
                        #如果有id,并且与库里相同
                        if request.globalplanId and  request.globalplanId == globalplan.getGlobalplanId():
                            positions = []
                            if globalplan.getAddPoints():
                                for addpoint in globalplan.getAddPoints():
                                    positionbean = Position(addpoint)
                                    positions.append(positionbean)
                            position = Position()
                            position.setX(request.addPoints.pointIdA)
                            position.setY(request.addPoints.pointIdB)
                            positions.append(position)
                            globalplan.setAddPoints(positions)
                        globalPlans.append(globalplan)

                # 回填map的globalplans
                maps[mapindex].setGlobalPlans(globalPlans)
        floors[floorindex].setMaps(maps)
        mapSetting.setFloors(floors)
        rospy.loginfo("mappseting %s", self.bean_to_dict(mapSetting))

        self.write_json(mapSetting, spirit_map_url)
        self.wall_update_pub.publish(0)
        addpointsGlobalplanresponse.result = True
        rospy.loginfo("为全局路线添加导航点数值对成功!")
        return addpointsGlobalplanresponse

    def deletepoints_globalplan_cb(self,request):
        # 此代码操作数据集遵循，
        # 先遍历，后备份，
        # 遍历不操作数据，
        # 对备份数据集操作，之后把备份回填对象
        rospy.loginfo("service call :deletepoints to globalplan!")
        addpointsGlobalplanresponse = AddpointsGlobalplanResponse()
        addpointsGlobalplanresponse.result = False

        if not self.map_ready:
            rospy.logwarn("map tf can not find")
            return addpointsGlobalplanresponse

        if self.map_status == 1 or self.map_status == 2:
            rospy.logwarn("map is build, disenable addShape")
            return addpointsGlobalplanresponse

        self.check_map()  # addShap

        if self.allMap is None:
            return addpointsGlobalplanresponse

        mapSetting = MapSetting(self.allMap)

        # 逐条取出保存，方便后面
        floors = []
        floorindex = -1
        isaddfloorindex = True
        maps = []

        for fll in mapSetting.getFloors():
            if isaddfloorindex:
                floorindex= floorindex+1
            floor = Floor(fll)
            floors.append(floor)
            if (floor.getFloorId() == mapSetting.getDefaultFloor()):
                # 找到默认楼层, 结束记录下标, 只记录默认地图的下标
                isaddfloorindex = False

                mapindex = -1
                isaddindex = True
                for uu in floor.getMaps():
                    if (isaddindex):
                        mapindex = mapindex + 1
                    map = Map(uu)
                    maps.append(map)
                    # 逐条取出保存
                    if map.getMapId() == floor.getDefaultmap():
                        isaddindex = False
                rospy.loginfo("globalPlans map size = %d, defatmapId = %d", len(maps), mapindex)

                globalPlans = []
                isDell = False
                if maps[mapindex].getGlobalPlans():
                    for globalplanarr in maps[mapindex].getGlobalPlans():
                        globalplan = GlobalPlan(globalplanarr)
                        #如果有id,并且与库里相同
                        if request.globalplanId and  request.globalplanId == globalplan.getGlobalplanId():
                            positions = []
                            if globalplan.getAddPoints():
                                for addpoint in globalplan.getAddPoints():
                                    positionbean = Position(addpoint)
                                    if (request.addPoints.pointIdA == positionbean.getX() and request.addPoints.pointIdB == positionbean.getY()):
                                        isDell = True
                                        continue
                                    positions.append(positionbean)
                            if isDell == False:
                                position = Position()
                                position.setX(request.addPoints.pointIdA)
                                position.setY(request.addPoints.pointIdB)
                                positions.append(position)
                            globalplan.setAddPoints(positions)
                        globalPlans.append(globalplan)

                # 回填map的globalplans
                maps[mapindex].setGlobalPlans(globalPlans)
        floors[floorindex].setMaps(maps)
        mapSetting.setFloors(floors)
        rospy.loginfo("mappseting %s", self.bean_to_dict(mapSetting))

        self.write_json(mapSetting, spirit_map_url)
        self.wall_update_pub.publish(0)
        addpointsGlobalplanresponse.result = True
        rospy.loginfo("删除全局路线导航点数值对成功!")
        return addpointsGlobalplanresponse

    def rename_globalplan_cb(self,request):
        rospy.loginfo("service call :rename to globalplan!")
        renameGlobalplanresponse = RenameGlobalplanResponse()
        renameGlobalplanresponse.result = False

        if not self.map_ready:
            rospy.logwarn("map tf can not find")
            return renameGlobalplanresponse

        if self.map_status == 1 or self.map_status == 2:
            rospy.logwarn("map is build, disenable addShape")
            return renameGlobalplanresponse

        self.check_map()  # addShap

        if self.allMap is None:
            return renameGlobalplanresponse

        local_time = self.getId()
        local_time = int(local_time)

        mapSetting = MapSetting(self.allMap)

        # 逐条取出保存，方便后面
        floors = []
        floorindex = -1
        isaddfloorindex = True
        maps = []

        for fll in mapSetting.getFloors():
            if isaddfloorindex:
                floorindex= floorindex+1
            floor = Floor(fll)
            floors.append(floor)
            if (floor.getFloorId() == mapSetting.getDefaultFloor()):
                # 找到默认楼层, 结束记录下标, 只记录默认地图的下标
                isaddfloorindex = False

                mapindex = -1
                isaddindex = True
                for uu in floor.getMaps():
                    if (isaddindex):
                        mapindex = mapindex + 1
                    map = Map(uu)
                    maps.append(map)
                    # 逐条取出保存
                    if map.getMapId() == floor.getDefaultmap():
                        isaddindex = False
                rospy.loginfo("globalPlans map size = %d, defatmapId = %d", len(maps), mapindex)
                globalPlans = []
                if maps[mapindex].getGlobalPlans():
                    for globalplanarr in maps[mapindex].getGlobalPlans():
                        globalplan = GlobalPlan(globalplanarr)
                        #如果有id,并且与库里相同
                        if request.globalplanId and  request.globalplanId == globalplan.getGlobalplanId():
                            globalplan.setDir(request.dir)
                            globalplan.setGlobalplanName(request.globalplanName)
                        globalPlans.append(globalplan)

                # 回填map的globalplans
                maps[mapindex].setGlobalPlans(globalPlans)
        floors[floorindex].setMaps(maps)
        mapSetting.setFloors(floors)
        self.write_json(mapSetting, spirit_map_url)
        renameGlobalplanresponse.result = True
        self.wall_update_pub.publish(0)
        rospy.loginfo("修改一条全局路线成功。")
        return renameGlobalplanresponse

    def point_set(self, request):
        # 此代码操作数据集遵循，
        # 先遍历，后备份，
        # 遍历不操作数据，
        # 对备份数据集操作，之后把备份回填对象

        #  0: 导航点,
        #  1: 充电桩,
        #  2: 初始点，
        #  3: 电梯点，


        # result
        #       0 添加成功
        #       1 地图未就绪
        #       2 未查找到地图信息列表
        #       3 名字不能为空
        #       4 属性或导航点名字错误（普通导航点名字不能为已有属性的关键词）

        rospy.loginfo("Service call : point_set map_status: %d, name: %s, mode: %d ", self.map_status, request.point_name, request.point_mode)
        response = PointSetResponse()
        response.result = 0

        if not self.map_ready:
            rospy.logwarn("map tf can not find")
            response.result = 1
            return response
        if (request.point_name == ""):
            rospy.loginfo("Service call : point_set name: %s is NULL, mode: %d success!", request.point_name,
                          request.point_mode)
            response.result = 3
            return response
        rospy.loginfo("Service call : point_set 3")
        local_time = self.getId()
        local_time = int(local_time)
        poseset = self.get_pos()
        point_name = request.point_name

        if self.map_status == 1 or self.map_status == 2:
            rospy.loginfo("Service call : point_set 4")
            if request.point_mode == 1:
                if self.build_systempoints:
                    loce_points = []
                    loceFind = False 
                    rospy.loginfo("Service call : point_set 4.1") 
                    for point1 in self.build_systempoints:
                        # point1 = SystemPoint(poin)
                        if (point1.getPointName() == "充电桩"):
                            loceFind = True
                            point1.setPointType(1)
                            point1.setPositionX(poseset.x)
                            point1.setPositionY(poseset.y)
                            point1.setPositionYaw(poseset.z)
                            loce_points.append(point1)
                            self.fix_chargepoint_flag = False

                            # 桩位置为空
                            chargepoint = Point32()
                            chargepoint.x = 9999
                            chargepoint.y = 9999
                            chargepoint.z = 9999
                            self.charge_point_pub.publish(chargepoint)
                        elif (point1.getPointName() == "桩位置"):
                            # 去掉桩位置点，重新充电后进行修正
                            continue
                        else:
                            # 保留初始点
                            loce_points.append(point1)
                    rospy.loginfo("Service call : point_set 4.2")
                    if loceFind:
                        rospy.loginfo("修改充电桩成功， 位置：%f, %f, %f", poseset.x, poseset.y, poseset.z)
                        self.build_systempoints = loce_points
                        self.wall_update_pub.publish(0)
                        return response
                rospy.loginfo("Service call : point_set 4.3")
                point = SystemPoint()
                point.setPointId(local_time)
                point.setPointType(1)
                point.setPositionX(poseset.x)
                point.setPositionY(poseset.y)
                point.setPositionYaw(poseset.z)
                point.setPointName("充电桩")
                self.fix_chargepoint_flag = False

                rospy.loginfo("point_set 添加充电桩成功， 位置：%f, %f, %f", poseset.x, poseset.y, poseset.z)
                self.build_systempoints.append(point)
            elif request.point_mode == 2:
                rospy.loginfo("Service call : point_set 5.0")
                if self.build_systempoints:
                    rospy.loginfo("Service call : point_set 5.1")
                    loce_points = []
                    loceFind = False  # type: bool
                    for point1 in self.build_systempoints:                        
                        # point1 = SystemPoint(poin)
                        rospy.loginfo("Service call : point_set 5.2.1, %s ",point1.getPointName())
                        if (point1.getPointName() == "初始点"):
                            loceFind = True
                            point1.setPointType(2)
                            point.setPointId(local_time)
                            point1.setPositionX(poseset.x)
                            point1.setPositionY(poseset.y)
                            point1.setPositionYaw(poseset.z)
                            loce_points.append(point1)                        
                        else:
                            # 保留充电桩和桩位置两个点
                            loce_points.append(point1)
                    rospy.loginfo("Service call : point_set 5.2.2 %d",loceFind)
                    if loceFind:
                        rospy.loginfo("修改初始点成功， 位置：%f, %f, %f", poseset.x, poseset.y, poseset.z)
                        self.build_systempoints = loce_points
                        self.wall_update_pub.publish(0)
                        return response
                rospy.loginfo("Service call : point_set 5.3")
                point = SystemPoint()
                point.setPointId(local_time)
                point.setPointType(2)
                point.setPositionX(poseset.x)
                point.setPositionY(poseset.y)
                point.setPositionYaw(poseset.z)
                point.setPointName("初始点")

                rospy.loginfo("point_set 添加初始点成功， 位置：%f, %f, %f", poseset.x, poseset.y, poseset.z)
                self.build_systempoints.append(point)

            elif request.point_mode == 3: # 电梯点
                rospy.loginfo("Service call : point_set 6.0")
                if self.build_systempoints:
                    loce_points = []
                    loceFind = False  # type: bool
                    for point1 in self.build_systempoints:
                        # point1 = SystemPoint(poin)
                        if (point1.getPointType()==3 and point1.getPointId() == request.point_id):
                            loceFind = True
                            continue
                        else:
                            # 保留充电桩和桩位置两个点
                            loce_points.append(point1)
                    if loceFind:
                        rospy.loginfo("删除电梯点成功， 位置：%f, %f, %f", poseset.x, poseset.y, poseset.z)
                        self.build_systempoints = loce_points
                        self.wall_update_pub.publish(0)
                        return response

                point = SystemPoint()
                point.setPointId(local_time)
                point.setPointType(3)
                point.setPositionX(poseset.x)
                point.setPositionY(poseset.y)
                point.setPositionYaw(poseset.z)
                point.setPointName(request.point_name)

                rospy.loginfo("point_set 添加电梯点成功， 位置：%f, %f, %f", poseset.x, poseset.y, poseset.z)
                self.build_systempoints.append(point)
            else:
                rospy.loginfo("Service call : point_set 7.0")
                point = MapsBeanPoint()
                point.setPointId(local_time)
                #point.setPointType(0)
                point.setPointName(point_name)
                point.setPositionX(poseset.x)
                point.setPositionY(poseset.y)
                point.setPositionYaw(poseset.z)

                rospy.loginfo("point_set 添加导航点（%s）成功，位置：%f, %f, %f", point_name, poseset.x, poseset.y, poseset.z)
                self.build_points.append(point)
            rospy.loginfo("Service call : point_set 7")
            self.wall_update_pub.publish(0)
            return response


        self.check_map()  # point set

        if self.allMap is None:
            response.result = 2
            return response

        mapSetting = MapSetting(self.allMap)

        # 逐条取出保存，方便后面
        floors = []
        floorindex = -1
        isaddfloorindex = True
        maps = []

        for fll in mapSetting.getFloors():
            if isaddfloorindex:
                floorindex= floorindex+1
            floor = Floor(fll)
            floors.append(floor)
            if (floor.getFloorId() == mapSetting.getDefaultFloor()):
                # 找到默认楼层, 结束记录下标, 只记录默认地图的下标
                isaddfloorindex = False

                mapindex = -1
                isaddindex = True
                for uu in floor.getMaps():
                    if (isaddindex):
                        mapindex = mapindex + 1
                    map = Map(uu)
                    maps.append(map)
                    # 逐条取出保存
                    if map.getMapId() == floor.getDefaultmap():
                        isaddindex = False
                rospy.loginfo("point_set map size = %d, defatmapId = %d", len(maps), mapindex)

                points = []
                if maps[mapindex].getPoints():
                    # 在已有导航点上添加点
                    for pointname in maps[mapindex].getPoints():
                        point = MapsBeanPoint(pointname)
                        points.append(point)

                    if request.point_mode == 0:
                        point = MapsBeanPoint()
                        point.setPointId(local_time)
                        point.setPointName(point_name)
                        point.setPositionX(poseset.x)
                        point.setPositionY(poseset.y)
                        point.setPositionYaw(poseset.z)

                        rospy.loginfo("point_set 新增点(%s)位置： %f, %f, %f", str(point_name), poseset.x, poseset.y, poseset.z)
                        points.append(point)
                else:
                    # 没有任何导航点
                    if request.point_mode == 0:
                        point = MapsBeanPoint()
                        point.setPointId(local_time)
                        point.setPointName(point_name)
                        point.setPositionX(poseset.x)
                        point.setPositionY(poseset.y)
                        point.setPositionYaw(poseset.z)
                        rospy.loginfo("point_set 没有任何导航点 新增点(%s)位置： %f, %f, %f", str(point_name), poseset.x, poseset.y, poseset.z)
                        points.append(point)

                systemPoints = []
                if maps[mapindex].getSystemPoints():
                    # 在已有导航点上添加或者修改初始点和充电点
                    is_chargepoint_update = False
                    is_initialpoint_update = False
                    is_elevatorpoint_dell = False
                    for pointname in maps[mapindex].getSystemPoints():
                        systempoint = MapsBeanPoint(pointname)
                        if request.point_mode == 1:
                            # 认为是修改
                            if systempoint.getPointName() == "充电桩":
                                rospy.loginfo("更新充电桩位置： %f, %f, %f", poseset.x, poseset.y, poseset.z)
                                systempoint.setPositionX(poseset.x)
                                systempoint.setPositionY(poseset.y)
                                systempoint.setPositionYaw(poseset.z)
                                maps[mapindex].setChargeId(systempoint.getPointId())
                                is_chargepoint_update = True
                                self.fix_chargepoint_flag = False
                            elif systempoint.getPointName() == "桩位置":
                                continue
                        elif request.point_mode == 2:
                            # 认为是修改
                            if systempoint.getPointName() == "初始点":
                                rospy.loginfo("更新初始点位置： %f, %f, %f", poseset.x, poseset.y, poseset.z)
                                systempoint.setPointId(local_time)
                                systempoint.setPositionX(poseset.x)
                                systempoint.setPositionY(poseset.y)
                                systempoint.setPositionYaw(poseset.z)
                                maps[mapindex].setInitialId(systempoint.getPointId())
                                is_initialpoint_update = True
                        if request.point_mode == 3 and systempoint.getPointId() == request.point_id :
                            # 认为是删除电梯点
                            is_elevatorpoint_dell = True
                            continue
                        systemPoints.append(systempoint)
                    if is_chargepoint_update:
                        rospy.loginfo("已经更新充电桩点point....")
                    else:
                        # 认为是添加点                        
                        if request.point_mode == 1:
                            systempoint = SystemPoint()
                            systempoint.setPointId(local_time)
                            systempoint.setPointType(1)
                            systempoint.setPositionX(poseset.x)
                            systempoint.setPositionY(poseset.y)
                            systempoint.setPositionYaw(poseset.z)
                            systempoint.setPointName("充电桩")
                            maps[mapindex].setChargeId(local_time)
                            systemPoints.append(systempoint)
                            self.fix_chargepoint_flag = False
                            rospy.loginfo("更新点充电桩位置： %f, %f, %f", poseset.x, poseset.y, poseset.z)

                    if is_initialpoint_update:
                        rospy.loginfo("已经更新初始点point....")
                    else:
                        # 认为是添加点
                        if request.point_mode == 0:
                            if request.point_name == "初始点":
                                response.result = 4
                                return response
                        if request.point_mode == 2:
                            systempoint = SystemPoint()
                            systempoint.setPointId(local_time)
                            systempoint.setPointType(2)
                            systempoint.setPositionX(poseset.x)
                            systempoint.setPositionY(poseset.y)
                            systempoint.setPositionYaw(poseset.z)
                            systempoint.setPointName("初始点")
                            maps[mapindex].setInitialId(local_time)
                            systemPoints.append(systempoint)
                            rospy.loginfo("更新点初始点位置： %f, %f, %f", poseset.x, poseset.y, poseset.z)
                    if is_elevatorpoint_dell:
                        rospy.loginfo("已经删除电梯点point....")
                    else:
                        # 认为是电梯点
                        if request.point_mode == 3:
                            systempoint = SystemPoint()
                            systempoint.setPointId(local_time)
                            systempoint.setPointType(3)
                            systempoint.setPositionX(poseset.x)
                            systempoint.setPositionY(poseset.y)
                            systempoint.setPositionYaw(poseset.z)
                            systempoint.setPointName("初始点")
                            systemPoints.append(systempoint)
                            rospy.loginfo("增加电梯点位置： %f, %f, %f", poseset.x, poseset.y, poseset.z)
                else:
                    # 没有任何导航点
                    if request.point_mode == 1:
                        systempoint = SystemPoint()
                        systempoint.setPointId(local_time)
                        systempoint.setPointType(1)
                        systempoint.setPositionX(poseset.x)
                        systempoint.setPositionY(poseset.y)
                        systempoint.setPositionYaw(poseset.z)
                        systempoint.setPointName("充电桩")
                        maps[mapindex].setChargeId(local_time)
                        systemPoints.append(systempoint)
                        self.fix_chargepoint_flag = False
                        rospy.loginfo("添加充电桩位置： %f, %f, %f", poseset.x, poseset.y, poseset.z)
                    elif request.point_mode == 2:
                        systempoint = SystemPoint()
                        systempoint.setPointId(local_time)
                        systempoint.setPointType(2)
                        systempoint.setPositionX(poseset.x)
                        systempoint.setPositionY(poseset.y)
                        systempoint.setPositionYaw(poseset.z)
                        systempoint.setPointName("初始点")
                        maps[mapindex].setInitialId(local_time)
                        systemPoints.append(systempoint)
                        rospy.loginfo("添加初始点位置： %f, %f, %f", poseset.x, poseset.y, poseset.z)
                    elif request.point_mode == 3:
                        systempoint = SystemPoint()
                        systempoint.setPointId(local_time)
                        systempoint.setPointType(3)
                        systempoint.setPositionX(poseset.x)
                        systempoint.setPositionY(poseset.y)
                        systempoint.setPositionYaw(poseset.z)
                        systempoint.setPointName(request.point_name)
                        systemPoints.append(systempoint)
                        rospy.loginfo("添加电梯点位置： %f, %f, %f", poseset.x, poseset.y, poseset.z)

                # 回填map的points
                maps[mapindex].setPoints(points)
                maps[mapindex].setSystemPoints(systemPoints)
        floors[floorindex].setMaps(maps)
        mapSetting.setFloors(floors)
        self.write_json(mapSetting, spirit_map_url)  # point set
        self.wall_update_pub.publish(0)

        self.check_map() #map null point add end
        rospy.loginfo("Service call : point_set name: %s, mode: %d success!", request.point_name, request.point_mode)
        return response

    def pointmanu_set(self, request):
        # 手动增加导航点，包含了导航点坐标

        #  0: 导航点,
        #  1: 充电桩,
        #  2: 初始点，

        # result
        #       0 添加成功
        #       1 地图未就绪
        #       2 未查找到地图信息列表
        #       3 名字不能为空
        #       4 属性或导航点名字错误（普通导航点名字不能为已有属性的关键词）

        rospy.loginfo("Service call : pointnamu_set name: %s, mode: %d, (%f, %f, %f)-(%f,%f,%f,%f)", request.point_name, request.point_mode, request.point.pose.pose.position.x, request.point.pose.pose.position.y, request.point.pose.pose.position.z, request.point.pose.pose.orientation.x, request.point.pose.pose.orientation.y, request.point.pose.pose.orientation.z, request.point.pose.pose.orientation.w)
        response = PointManuSetResponse()
        response.result = 0
        if not self.map_ready:
            rospy.logwarn("map tf can not find")
            response.result = 1
            return response
        if (request.point_name == ""):
            rospy.loginfo("Service call : pointnamu_set name: %s is NULL, mode: %d success!", request.point_name,
                          request.point_mode)
            response.result = 3
            return response
        local_time = self.getId()
        local_time = int(local_time)

        (euler_r,euler_p,euler_y) = transformations.euler_from_quaternion([request.point.pose.pose.orientation.x,request.point.pose.pose.orientation.y,request.point.pose.pose.orientation.z,request.point.pose.pose.orientation.w])
        poseset = Point32()
        poseset.x = request.point.pose.pose.position.x
        poseset.y = request.point.pose.pose.position.y
        poseset.z = euler_y / pi * 180
        covariance = request.point.pose.covariance
        # covariance 是 float64[] 这里借用数组 第一个作为id
        point_name = request.point_name

        if self.map_status == 1 or self.map_status == 2:
            if request.point_mode == 1:
                if self.build_systempoints:
                    loce_points = []
                    loceFind = False  # type: bool
                    for point1 in self.build_systempoints:
                        # point1 = SystemPoint(poin)
                        if (point1.getPointName() == "充电桩"):
                            loceFind = True
                            point1.setPointType(1)
                            point1.setPositionX(poseset.x)
                            point1.setPositionY(poseset.y)
                            point1.setPositionYaw(poseset.z)
                            loce_points.append(point1)
                            self.fix_chargepoint_flag = False
                        elif point1.getPointName() == "桩位置":
                            continue
                        else:
                            # 保留初始点
                            loce_points.append(point1)
                    if loceFind:
                        rospy.loginfo("修改充电桩成功， 位置：%f, %f, %f", poseset.x, poseset.y, poseset.z)
                        self.build_systempoints = loce_points
                        self.wall_update_pub.publish(0)
                        return response

                point = SystemPoint()
                point.setPointType(1)
                point.setPointId(local_time)
                point.setPositionX(poseset.x)
                point.setPositionY(poseset.y)
                point.setPositionYaw(poseset.z)
                point.setPointName("充电桩")
                self.fix_chargepoint_flag = False

                rospy.loginfo("添加充电桩成功， 位置：%f, %f, %f", poseset.x, poseset.y, poseset.z)
                self.build_systempoints.append(point)
            elif request.point_mode == 2:
                if self.build_systempoints:
                    loce_points = []
                    loceFind = False  # type: bool
                    for point1 in self.build_systempoints:
                        # point1 = SystemPoint(poin)
                        if (point1.getPointName() == "初始点"):
                            loceFind = True
                            point1.setPointType(2)
                            point1.setPointId(local_time)
                            point1.setPositionX(poseset.x)
                            point1.setPositionY(poseset.y)
                            point1.setPositionYaw(poseset.z)
                            loce_points.append(point1)
                        else:
                            # 保留充电桩
                            loce_points.append(point1)
                    if loceFind:
                        rospy.loginfo("修改初始点成功， 位置：%f, %f, %f", poseset.x, poseset.y, poseset.z)
                        self.build_systempoints = loce_points
                        self.wall_update_pub.publish(0)
                        return response

                point = SystemPoint()
                point.setPointType(2)
                point.setPointId(local_time)
                point.setPositionX(poseset.x)
                point.setPositionY(poseset.y)
                point.setPositionYaw(poseset.z)
                point.setPointName("初始点")

                rospy.loginfo("pointmanu_set 添加初始点成功， 位置：%f, %f, %f", poseset.x, poseset.y, poseset.z)
                self.build_systempoints.append(point)
            else:
                if self.build_points:
                    loce_points = []
                    loceFind = False  # type: bool
                    for point1 in self.build_points:
                        # point1 = MapsBeanPoint(poin)
                        if covariance and len(covariance)>0 and (point1.getPointId() == covariance[0]):
                            loceFind = True
                            point1.setPointName(point_name)
                            point1.setPositionX(poseset.x)
                            point1.setPositionY(poseset.y)
                            point1.setPositionYaw(poseset.z)
                            loce_points.append(point1)
                        else:
                            # 保留其他导航点
                            loce_points.append(point1)
                    if loceFind:
                        rospy.loginfo("修改导航点成功， 位置：%f, %f, %f", poseset.x, poseset.y, poseset.z)
                        self.build_points = loce_points
                        self.wall_update_pub.publish(0)
                        return response
                point = MapsBeanPoint()
                point.setPointId(local_time)
                point.setPointName(point_name)
                point.setPositionX(poseset.x)
                point.setPositionY(poseset.y)
                point.setPositionYaw(poseset.z)

                rospy.loginfo("添加导航点（%s）成功，位置：%f, %f, %f", point_name, poseset.x, poseset.y, poseset.z)
                self.build_points.append(point)
            self.wall_update_pub.publish(0)
            return response


        self.check_map()  # point set

        if self.allMap is None:
            response.result = 2
            return response

        mapSetting = MapSetting(self.allMap)

        # 逐条取出保存，方便后面
        floors = []
        floorindex = -1
        isaddfloorindex = True
        maps = []

        for fll in mapSetting.getFloors():
            if isaddfloorindex:
                floorindex= floorindex+1
            floor = Floor(fll)
            floors.append(floor)
            if (floor.getFloorId() == mapSetting.getDefaultFloor()):
                # 找到默认楼层, 结束记录下标, 只记录默认地图的下标
                isaddfloorindex = False

                mapindex = -1
                isaddindex = True
                for uu in floor.getMaps():
                    if (isaddindex):
                        mapindex = mapindex + 1
                    map = Map(uu)
                    maps.append(map)
                    # 逐条取出保存
                    if map.getMapId() == floor.getDefaultmap():
                        isaddindex = False
                rospy.loginfo("pointmanu map size = %d, defatmapId = %d", len(maps), mapindex)

                points = []
                if maps[mapindex].getPoints():
                    # 在已有导航点上添加点
                    is_navipoint_update = False
                    for pointname in maps[mapindex].getPoints():
                        point = MapsBeanPoint(pointname)
                        if covariance and len(covariance)>0 and (point.getPointId() == covariance[0]):
                            rospy.loginfo("更新导航点位置： %f, %f, %f", poseset.x, poseset.y, poseset.z)
                            point.setPointName(point_name)
                            point.setPositionX(poseset.x)
                            point.setPositionY(poseset.y)
                            point.setPositionYaw(poseset.z)
                            is_navipoint_update = True
                        points.append(point)

                    if request.point_mode == 0:
                        if request.point_name == "充电桩":
                            response.result = 4
                            return response
                    if request.point_mode == 0 and (not is_navipoint_update):
                        point = MapsBeanPoint()
                        point.setPointId(local_time)
                        point.setPointName(point_name)
                        point.setPositionX(poseset.x)
                        point.setPositionY(poseset.y)
                        point.setPositionYaw(poseset.z)
                        response.result = local_time
                        rospy.loginfo("新增点(%s)位置： %f, %f, %f", str(point_name), poseset.x, poseset.y, poseset.z)
                        points.append(point)
                else:
                    # 没有任何导航点
                    if request.point_mode == 0:
                        point = MapsBeanPoint()
                        point.setPointId(local_time)
                        point.setPointName(point_name)
                        point.setPositionX(poseset.x)
                        point.setPositionY(poseset.y)
                        point.setPositionYaw(poseset.z)
                        response.result = local_time
                        rospy.loginfo("更新点(%s)位置： %f, %f, %f", str(point_name), poseset.x, poseset.y, poseset.z)
                        points.append(point)

                systemPoints = []
                if maps[mapindex].getSystemPoints():
                    # 在已有导航点上添加或者修改初始点和充电点
                    is_chargepoint_update = False
                    is_initialpoint_update = False
                    for pointname in maps[mapindex].getSystemPoints():
                        systempoint = MapsBeanPoint(pointname)
                        if request.point_mode == 1:
                            # 认为是修改
                            if systempoint.getPointName() == "充电桩":
                                rospy.loginfo("更新充电桩位置： %f, %f, %f", poseset.x, poseset.y, poseset.z)
                                systempoint.setPositionX(poseset.x)
                                systempoint.setPositionY(poseset.y)
                                systempoint.setPositionYaw(poseset.z)
                                is_chargepoint_update = True
                                self.fix_chargepoint_flag = False
                            elif systempoint.getPointName() == "桩位置":
                                continue
                        elif request.point_mode == 2:
                            # 认为是修改
                            if systempoint.getPointName() == "初始点":
                                rospy.loginfo("更新初始点位置： %f, %f, %f", poseset.x, poseset.y, poseset.z)
                                systempoint.setPointId(local_time)
                                maps[mapindex].setInitialId(local_time)
                                systempoint.setPositionX(poseset.x)
                                systempoint.setPositionY(poseset.y)
                                systempoint.setPositionYaw(poseset.z)
                                is_initialpoint_update = True
                        systemPoints.append(systempoint)
                    if is_chargepoint_update:
                        rospy.loginfo("已经更新充电桩点point....")
                    else:
                        # 认为是添加点
                        if request.point_mode == 0:
                            if request.point_name == "充电桩":
                                response.result = 4
                                return response
                        if request.point_mode == 1:
                            systempoint = MapsBeanPoint()
                            systempoint.setPointId(local_time)
                            systempoint.setPositionX(poseset.x)
                            systempoint.setPositionY(poseset.y)
                            systempoint.setPositionYaw(poseset.z)
                            systempoint.setPointName("充电桩")
                            self.fix_chargepoint_flag = False
                            maps[mapindex].setChargeId(local_time)
                            systemPoints.append(systempoint)
                            rospy.loginfo("更新点充电桩位置： %f, %f, %f", poseset.x, poseset.y, poseset.z)
                    if is_initialpoint_update:
                        rospy.loginfo("已经更新初始点point....")
                    else:
                        # 认为是添加点
                        if request.point_mode == 0:
                            if request.point_name == "初始点":
                                response.result = 4
                                return response
                        if request.point_mode == 2:
                            systempoint = MapsBeanPoint()
                            systempoint.setPointId(local_time)
                            systempoint.setPositionX(poseset.x)
                            systempoint.setPositionY(poseset.y)
                            systempoint.setPositionYaw(poseset.z)
                            systempoint.setPointName("初始点")
                            maps[mapindex].setInitialId(local_time)
                            systemPoints.append(systempoint)
                            rospy.loginfo("更新点初始点位置： %f, %f, %f", poseset.x, poseset.y, poseset.z)
                else:
                    # 没有任何导航点
                    if request.point_mode == 1:
                        systempoint = MapsBeanPoint()
                        systempoint.setPointId(local_time)
                        systempoint.setPositionX(poseset.x)
                        systempoint.setPositionY(poseset.y)
                        systempoint.setPositionYaw(poseset.z)
                        systempoint.setPointName("充电桩")
                        maps[mapindex].setChargeId(local_time)
                        systemPoints.append(systempoint)
                        self.fix_chargepoint_flag = False
                        rospy.loginfo("添加充电桩位置： %f, %f, %f", poseset.x, poseset.y, poseset.z)
                    elif request.point_mode == 1:
                        systempoint = MapsBeanPoint()
                        systempoint.setPointId(local_time)
                        systempoint.setPositionX(poseset.x)
                        systempoint.setPositionY(poseset.y)
                        systempoint.setPositionYaw(poseset.z)
                        systempoint.setPointName("初始点")
                        maps[mapindex].setInitialId(local_time)
                        systemPoints.append(systempoint)
                        rospy.loginfo("添加初始点位置： %f, %f, %f", poseset.x, poseset.y, poseset.z)

                # 回填map的points
                maps[mapindex].setPoints(points)
                maps[mapindex].setSystemPoints(systemPoints)
        floors[floorindex].setMaps(maps)
        mapSetting.setFloors(floors)
        self.write_json(mapSetting, spirit_map_url)  # point set
        self.wall_update_pub.publish(0)

        self.check_map() #map null point add end
        rospy.loginfo("Service call : pointmanu name: %s, mode: %d success!", request.point_name, request.point_mode)
        return response
    
    def get_lorastatus_cb(self, request):
        rospy.loginfo("Service call : get_lorastatus_cb")
        response = SetInt64Response()
        response.success = self.lora_status
        rospy.loginfo("get_lorastatus_cb success!!!")
        return response
    
    def get_espstatus_cb(self, request):
        rospy.loginfo("Service call : get_espstatus_cb")
        response = SetInt64Response()
        response.success = self.esp_status
        rospy.loginfo("get_espstatus_cb success!!!")
        return response

    def robotsonar_set(self, request):
        rospy.loginfo("Service call : robotsonar_set")
        response = RobotSonarSetResponse()
        response.result = 0
        rospy.loginfo("sonar set is ignoreed!!!")
        return response

        sonars_entry = request.sonars
        tmp_sonarsetting = dict()
        tmp_sonarsetting.clear()
        tmp_sonarsetting = self.read_param(robot_driver_param_url)
        sonar_setting = dict()
        sonar_setting.clear()
        if not bool(tmp_sonarsetting):
            rospy.loginfo("sonar setting is empty")
            return response
        else:
            sonar_setting = tmp_sonarsetting.copy()

        sonar_setting["useSonar"] = request.user_sonar
        sonar_setting["sonar0_switch"] = sonars_entry[0].sonar_switch
        sonar_setting["sonar0_minrange"] = sonars_entry[0].min_range
        sonar_setting["sonar0_maxrange"] = sonars_entry[0].max_range

        sonar_setting["sonar1_switch"] = sonars_entry[1].sonar_switch
        sonar_setting["sonar1_minrange"] = sonars_entry[1].min_range
        sonar_setting["sonar1_maxrange"] = sonars_entry[1].max_range

        sonar_setting["sonar2_switch"] = sonars_entry[2].sonar_switch
        sonar_setting["sonar2_minrange"] = sonars_entry[2].min_range
        sonar_setting["sonar2_maxrange"] = sonars_entry[2].max_range

        sonar_setting["sonar3_switch"] = sonars_entry[3].sonar_switch
        sonar_setting["sonar3_minrange"] = sonars_entry[3].min_range
        sonar_setting["sonar3_maxrange"] = sonars_entry[3].max_range

        sonar_setting["sonar4_switch"] = sonars_entry[4].sonar_switch
        sonar_setting["sonar4_minrange"] = sonars_entry[4].min_range
        sonar_setting["sonar4_maxrange"] = sonars_entry[4].max_range

        sonar_setting["sonar5_switch"] = sonars_entry[5].sonar_switch
        sonar_setting["sonar5_minrange"] = sonars_entry[5].min_range
        sonar_setting["sonar5_maxrange"] = sonars_entry[5].max_range

        sonar_setting["sonar6_switch"] = sonars_entry[6].sonar_switch
        sonar_setting["sonar6_minrange"] = sonars_entry[6].min_range
        sonar_setting["sonar6_maxrange"] = sonars_entry[6].max_range

        self.writ_patam(robot_driver_param_url, sonar_setting)

        sonars_set = RobotSonarEntrys()
        sonars_set.user_sonar = request.user_sonar

        sonar0_set = RobotSonarEntry()
        sonar0_set.name = "sonar0"
        sonar0_set.sonar_switch = sonars_entry[0].sonar_switch
        sonar0_set.min_range = sonars_entry[0].min_range
        sonar0_set.max_range = sonars_entry[0].max_range
        sonars_set.sonars.append(sonar0_set)

        sonar1_set = RobotSonarEntry()
        sonar1_set.name = "sonar1"
        sonar1_set.sonar_switch = sonars_entry[1].sonar_switch
        sonar1_set.min_range = sonars_entry[1].min_range
        sonar1_set.max_range = sonars_entry[1].max_range
        sonars_set.sonars.append(sonar1_set)

        sonar2_set = RobotSonarEntry()
        sonar2_set.name = "sonar2"
        sonar2_set.sonar_switch = sonars_entry[2].sonar_switch
        sonar2_set.min_range = sonars_entry[2].min_range
        sonar2_set.max_range = sonars_entry[2].max_range
        sonars_set.sonars.append(sonar2_set)

        sonar3_set = RobotSonarEntry()
        sonar3_set.name = "sonar3"
        sonar3_set.sonar_switch = sonars_entry[3].sonar_switch
        sonar3_set.min_range = sonars_entry[3].min_range
        sonar3_set.max_range = sonars_entry[3].max_range
        sonars_set.sonars.append(sonar3_set)

        sonar4_set = RobotSonarEntry()
        sonar4_set.name = "sonar4"
        sonar4_set.sonar_switch = sonars_entry[4].sonar_switch
        sonar4_set.min_range = sonars_entry[4].min_range
        sonar4_set.max_range = sonars_entry[4].max_range
        sonars_set.sonars.append(sonar4_set)

        sonar5_set = RobotSonarEntry()
        sonar5_set.name = "sonar5"
        sonar5_set.sonar_switch = sonars_entry[5].sonar_switch
        sonar5_set.min_range = sonars_entry[5].min_range
        sonar5_set.max_range = sonars_entry[5].max_range
        sonars_set.sonars.append(sonar5_set)

        sonar6_set = RobotSonarEntry()
        sonar6_set.name = "sonar6"
        sonar6_set.sonar_switch = sonars_entry[6].sonar_switch
        sonar6_set.min_range = sonars_entry[6].min_range
        sonar6_set.max_range = sonars_entry[6].max_range
        sonars_set.sonars.append(sonar6_set)

        self.sonars_pub.publish(sonars_set)

        rospy.loginfo("重新配置超声波设置成功。")
        rospy.loginfo("Service call : robotsonar_set success!")
        return response

    def robotsonar_list(self, request):
        rospy.loginfo("Service call : robotsonar_list")
        response = RobotSonarListResponse()
        rospy.loginfo("sonar get is ignoreed!!!")
        return response

        sonar_setting = dict()
        sonar_setting.clear()
        sonar_setting = self.read_param(robot_driver_param_url)

        response.user_sonar = sonar_setting["useSonar"]

        front_entry = RobotSonarEntry()
        front_entry.name = str("front")
        front_entry.sonar_switch = sonar_setting["sonar0_switch"]
        front_entry.min_range = sonar_setting["sonar0_minrange"]
        front_entry.max_range = sonar_setting["sonar0_maxrange"]
        front_entry.range = self.sonardata[0]
        response.sonars.append(front_entry)

        up_entry = RobotSonarEntry()
        up_entry.name = str("up")
        up_entry.sonar_switch = sonar_setting["sonar1_switch"]
        up_entry.min_range = sonar_setting["sonar1_minrange"]
        up_entry.max_range = sonar_setting["sonar1_maxrange"]
        up_entry.range = self.sonardata[1]
        response.sonars.append(up_entry)

        left_entry = RobotSonarEntry()
        left_entry.name = str("left")
        left_entry.sonar_switch = sonar_setting["sonar2_switch"]
        left_entry.min_range = sonar_setting["sonar2_minrange"]
        left_entry.max_range = sonar_setting["sonar2_maxrange"]
        left_entry.range = self.sonardata[2]
        response.sonars.append(left_entry)

        right_entry = RobotSonarEntry()
        right_entry.name = str("right")
        right_entry.sonar_switch = sonar_setting["sonar3_switch"]
        right_entry.min_range = sonar_setting["sonar3_minrange"]
        right_entry.max_range = sonar_setting["sonar3_maxrange"]
        right_entry.range = self.sonardata[3]
        response.sonars.append(right_entry)

        leftm_entry = RobotSonarEntry()
        leftm_entry.name = str("leftmiddle")
        leftm_entry.sonar_switch = sonar_setting["sonar4_switch"]
        leftm_entry.min_range = sonar_setting["sonar4_minrange"]
        leftm_entry.max_range = sonar_setting["sonar4_maxrange"]
        leftm_entry.range = self.sonardata[4]
        response.sonars.append(leftm_entry)

        rightm_entry = RobotSonarEntry()
        rightm_entry.name = str("rightmiddle")
        rightm_entry.sonar_switch = sonar_setting["sonar5_switch"]
        rightm_entry.min_range = sonar_setting["sonar5_minrange"]
        rightm_entry.max_range = sonar_setting["sonar5_maxrange"]
        rightm_entry.range = self.sonardata[5]
        response.sonars.append(rightm_entry)

        back_entry = RobotSonarEntry()
        back_entry.name = str("back")
        back_entry.sonar_switch = sonar_setting["sonar6_switch"]
        back_entry.min_range = sonar_setting["sonar6_minrange"]
        back_entry.max_range = sonar_setting["sonar6_maxrange"]
        back_entry.range = self.sonardata[6]
        response.sonars.append(back_entry)

        rospy.loginfo("Service call : robotsonar_list success!")
        return response

    def robotspeed_set(self, request):
        rospy.loginfo("Service call : robotspeed_set: %d", request.speed)
        response = RobotSpeedSetResponse()
        tmp_speedsetting = dict()
        tmp_speedsetting.clear()
        tmp_speedsetting = self.read_param(robot_driver_param_url)
        speed_setting = dict()
        speed_setting.clear()
        if not bool(tmp_speedsetting):
            rospy.loginfo("speed setting is empty")
            return response
        else:
            speed_setting = tmp_speedsetting.copy()

        speed_setting["speed_dwa"] = request.speed

        self.dwaspeed = request.speed
        self.writ_patam(robot_driver_param_url, speed_setting)

        self.sendSpeed_pub.publish(request.speed)

        response.result = 0
        rospy.loginfo("重新配置速度模式设置成功：%d。", request.speed)

        rospy.loginfo("Service call : robotspeed_set %d success!", request.speed)
        return response

    def robotspeed_list(self, request):
        rospy.loginfo("Service call : robotspeed_list!")
        response = RobotSpeedListResponse()
        # os.chmod(robot_driver_param_url, stat.S_IRWXU+stat.S_IRWXG+stat.S_IRWXO)
        # 打开文件
        fo = open(robot_driver_param_url)
        # d = yaml.load(fo, Loader=yaml.FullLoader)
        d = yaml.load(fo)
        if (d['speed_dwa'] == 1):
            response.speed = 1
            self.dwaspeed = 1
            rospy.loginfo("目前速度设置为低速模式（0.4米每秒）！")
        elif (d['speed_dwa'] == 2):
            response.speed = 2
            self.dwaspeed = 2
            rospy.loginfo("目前速度设置为中速模式（0.6米每秒）！")
        elif (d['speed_dwa'] == 3):
            response.speed = 3
            self.dwaspeed = 3
            rospy.loginfo("目前速度设置为高速模式（1.2米每秒）！")
        else:
            rospy.logerr("Unable to query")
            self.Errormsg_pub.publish("Unable to query")
        # 关闭文件
        fo.close()

        rospy.loginfo("Service call : robotspeed_list success!")
        return response

    def navigationsafeset_set(self, request):
        rospy.loginfo("Service call : navigationsafeset_set: %d", request.op)
        response = NavigationSafeSetResponse()
        tmp_navigationsafesetting = dict()
        tmp_navigationsafesetting.clear()
        tmp_navigationsafesetting = self.read_param(robot_driver_param_url)
        navigation_safesetting = dict()
        navigation_safesetting.clear()
        if not bool(tmp_navigationsafesetting):
            rospy.loginfo("navigation safe setting is empty")
            return response
        else:
            navigation_safesetting = tmp_navigationsafesetting.copy()

        navigation_safesetting["navigtaion_safeset"] = request.op

        self.navigation_safeset = request.op
        self.writ_patam(robot_driver_param_url, navigation_safesetting)

        if (request.op == 0 and self.need_navigation == False):
            self.neednavigation_pub.publish(0)
        else:
            self.neednavigation_pub.publish(1)

        response.result = 0
        rospy.loginfo("重新配置导航安全模式设置成功：%d。", request.op)

        rospy.loginfo("Service call : navigationsafeset_set %d success!", request.op)
        return response

    def navigationsafeset_list(self, request):
        rospy.loginfo("Service call : navigationsafeset_list!")
        response = NavigationSafeSetListResponse()
        # os.chmod(robot_driver_param_url, stat.S_IRWXU+stat.S_IRWXG+stat.S_IRWXO)
        # 打开文件
        fo = open(robot_driver_param_url)
        # d = yaml.load(fo, Loader=yaml.FullLoader)
        d = yaml.load(fo)
        if (d['navigtaion_safeset'] == 0):
            response.op = 0
            self.navigation_safeset = 0
            rospy.loginfo("安全配置设置为低！")
        elif (d['navigtaion_safeset'] == 1):
            response.op = 1
            self.navigation_safeset = 1
            rospy.loginfo("安全配置设置为中！")
        elif (d['navigtaion_safeset'] == 2):
            response.op = 2
            self.navigation_safeset = 2
            rospy.loginfo("安全配置设置为高！")
        else:
            rospy.logerr("Unable to query")
            self.Errormsg_pub.publish("Unable to query")
        # 关闭文件
        fo.close()

        rospy.loginfo("Service call : navigationsafeset_list success!")
        return response

    def bag_list_cb(self, request):
        response = BagListResponse()
        # response.bag_names.append('test.bag')
        # os.chdir("/home/spirit/.ros")
        # names=os.system("find ./ -name '*.bag' ")
        # response.bag_names.append(str(names))
        f_list = os.listdir(bag_param)
        for i in f_list:
            if os.path.splitext(i)[1] == '.bag':
                response.bag_names.append(i)
        response.status = 0
        rospy.loginfo("上层获取bag包列表成功。")
        return response

    def get_log_cb(self, request):
        response = GetLogResponse()
        LOG_PATH = log_param
        if not os.path.exists(LOG_PATH):
            os.mkdir(LOG_PATH)

        os.chdir("/root/.ros/log/latest")

        pathname = rospy.Time.now()
        os.system("zip -r " + str(pathname) + ".zip *")

        os.system("mv " + str(pathname) + ".zip " + LOG_PATH)

        response.log_name = str(pathname) + ".zip"
        response.status = 0
        rospy.loginfo("上层获取导航日志包列表成功。")
        return response

    def delete_alllogs_cb(self, request):
        response = DeleteAlllogsResponse()
        LOG_PATH = log_param
        if not os.path.exists(LOG_PATH):
            response.status = 0
            return response

        os.system("rm -rf "+ log_param + "/*.*")
        response.status = 0
        rospy.loginfo("上层删除所有日志成功。")
        return response

    def delete_allbags_cb(self, request):
        response = DeleteAllbagsResponse()

        os.system("rm -rf " + bag_param + "/*.*")
        response.status = 0
        rospy.loginfo("上层删除所有bag包成功。")
        return response

    def upload_server_cb(self, request):
        rospy.loginfo("Mapserver upload_server type: %d ", request.type)
        commit_file = CommitFile()
        commit_file.type = request.type
        commit_file.bag_name = request.bag_name
        response = UploadServerResponse()
        self.commitFile_pub.publish(commit_file)
        response.status = 0
        return response

    def testodom_thread_func(self):
        rospy.loginfo("开启odom测试子线程... ")
        if self.testodom_type == 1:
            # 测试自由旋转
            self.startAngular(0, self.testodom_value)
        elif self.testodom_type == 2:
            # 测试直行
            self.startLinear(self.testodom_value)
        elif self.testodom_type == 3:
            # 自动重定位
            self.startAutoRelocation()
        self.testodom_switch = False
        self.testodom_pub.publish(9999)
        rospy.loginfo("结束odom测试线程... ")


    def move_calibration_cb(self, request):
        rospy.loginfo("Service call : move_calibration, type: %d, value: %f" ,request.type, request.value)
        response = ScaleTestResponse()
        self.testodom_type = request.type
        self.testodom_value = request.value
        if (self.testodom_switch):
            rospy.loginfo("正在测试中，不进行重复测试，返回")
            return response
        try:
            if (self.testodom_type == 1 or self.testodom_type == 2 or self.testodom_type == 3):
                # 1： 测试旋转 2： 测试直行  3： 自动重定位
                self.testodom_switch = True
                self.testodom = threading.Thread(target=self.testodom_thread_func)
                # start 之后，子线程开始执行，主线程也会继续执行，不会等待
                self.testodom.start()  # 子线程执行
            elif (self.testodom_type == 4):
                self.TestDriver(101)
            elif (self.testodom_type == 5):
                self.TestDriver(102)
            elif (self.testodom_type == 6):
                self.TestDriver(103)
            elif (self.testodom_type == 7):
                self.TestDriver(104)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logerr("TF Exception")
            response.status = 1
            return response
        response.status = 0
        rospy.loginfo("机器人移动测试service接收成功！type: %d, value: %d" ,request.type, request.value)
        return response

    def set_coconum_cb(self, request):
        rospy.loginfo("Service call : set_coconum, coco_num: %s" % request.coco_num)
        response = SetCocoResponse()
        with open(version_url, 'r') as f:
            update = json.load(f)
            update["coco_num"] = str(request.coco_num)

        with open(version_url, 'w') as f:
            json.dump(update, f)
        f.close()
        response.status = 0
        rospy.loginfo("重新配置机器号：%s 成功。", str(request.coco_num))
        return response

    def auto_upgrade_cb(self, request):
        rospy.logdebug("Service call : auto_upgrade", request.op)
        response = AutoUpgradeResponse()
        if (request.op == 1):
            os.system("roslaunch auto_upgrade autoupgrade.launch")
            response.status = 0
            rospy.loginfo("auto_upgrade success")
        rospy.loginfo("重新启动升级程序成功。")
        return response

    def navi_info_cb(self, request):
        naviresponse = NaviInfoResponse()
        try:
            with io.open(version_url, 'r', encoding='utf-8') as f:
                content = f.read()
                naviresponse.status = 0
                naviresponse.nav_info = content
        except Exception as e:
            naviresponse.status = 1
        return naviresponse

    def robot_ssh_cb(self, request):
        robotssh_response = RobotSshResponse()
        rospy.loginfo("robot_ssh_cb %s", request.instruction)
        if 'passwd' in request.instruction:
            robotssh_response.status = 1
            robotssh_response.data = '命令非法'
            rospy.loginfo("robot_ssh_cb 命令非法")
            return robotssh_response
        pipe = subprocess.Popen(request.instruction, shell=True, stdout=subprocess.PIPE).stdout
        robotssh_response.data = pipe.read()
        robotssh_response.status = 0
        rospy.loginfo("配置ssh命令成功。")
        return robotssh_response

    def set_diypath_cb(self, request):
        rospy.loginfo("sey_diypath_cb type = %d ", request.type)
        setdiypathresponse = SetDiyPathResponse()
        setdiypathresponse.status = 0
        rospy.loginfo("上层配置DIYPATH成功。")
        return setdiypathresponse

    def get_config_cb(self, request):
        getconfigresponse = GetConfigsResponse()
        rospy.loginfo("get_config_cb type = %d ", request.type)
        getconfigresponse.status = 0

        config = Config()
        
        for param in rospy.get_param_names():
            if mapserver_param in param and (request.type == 9 or request.type == 0):
                config = Config()
                config.type = 9
                config.config_name = param.replace(mapserver_param, '', 1)
                # config.config_name = param.strip(mapserver_param)
                config.config_value = repr(rospy.get_param(param, ""))
                getconfigresponse.configs.append(config)
            elif sensor_imu_param in param and (request.type == 6 or request.type == 0):
                config = Config()
                config.type = 8
                config.config_name = param.replace(sensor_imu_param, '', 1)
                config.config_value = repr(rospy.get_param(param, ""))
                getconfigresponse.configs.append(config)
            elif ydlidar_node_param in param and (request.type == 6 or request.type == 0):
                config = Config()
                config.type = 7
                config.config_name = param.replace(ydlidar_node_param, '', 1)
                config.config_value = repr(rospy.get_param(param, ""))
                getconfigresponse.configs.append(config)
            elif robot_driver_param in param and (request.type == 6 or request.type == 0):
                config = Config()
                config.type = 6
                config.config_name = param.replace(robot_driver_param, '', 1)
                config.config_value = repr(rospy.get_param(param, ""))
                getconfigresponse.configs.append(config)
            elif dwa_planner_param in param and (request.type == 5 or request.type == 0):
                config = Config()
                config.type = 5
                config.config_name = param.replace(dwa_planner_param, '', 1)
                config.config_value = repr(rospy.get_param(param, ""))
                getconfigresponse.configs.append(config)
            elif local_cost_param in param and (request.type == 4 or request.type == 0):
                config = Config()
                config.type = 4
                config.config_name = param.replace(local_cost_param, '', 1)
                config.config_value = repr(rospy.get_param(param, ""))
                getconfigresponse.configs.append(config)
            elif navfn_ros_param in param and (request.type == 3 or request.type == 0):
                config = Config()
                config.type = 3
                config.config_name = param.replace(navfn_ros_param, '', 1)
                config.config_value = repr(rospy.get_param(param, ""))
                getconfigresponse.configs.append(config)
            elif (global_cost_param in param) and (request.type == 2 or request.type == 0):
                config = Config()
                config.type = 2
                config.config_name = param.replace(global_cost_param, '', 1)
                config.config_value = repr(rospy.get_param(param, ""))
                getconfigresponse.configs.append(config)
            elif move_base_param in param and (request.type == 1 or request.type == 0):
                config = Config()
                config.type = 1
                config.config_name = param.replace(move_base_param, '', 1)
                config.config_value = repr(rospy.get_param(param, ""))
                getconfigresponse.configs.append(config)
            elif request.type == 0:
                config = Config()
                config.type = -1
                config.config_name = param
                config.config_value = repr(rospy.get_param(param, ""))
                getconfigresponse.configs.append(config)
        rospy.loginfo("获取机器人配置 %d 成功。 ", request.type)
        return getconfigresponse

    def set_config_cb(self, request):
        setconfigsresponse = SetOrDelConfigsResponse()
        rospy.loginfo("set_config_cb type = %d ", request.type)
        setconfigsresponse.status = 0
        for config in request.configs:
            rospy.loginfo("setconfig type: %d name: %s value: %s", config.type, config.config_name,
                          config.config_value)
            if config.type == 1:
                try:
                    value = type(rospy.get_param(move_base_param + config.config_name, "1"))(config.config_value)
                    rospy.set_param(move_base_param + config.config_name, value)
                    testmaper = dict()
                    testmaper = self.read_param(move_base_param_url)
                    testmaper[config.config_name] = value
                    self.writ_patam(move_base_param_url, testmaper)
                except Exception as e:
                    setconfigsresponse.status = 5
                    return setconfigsresponse
            elif config.type == 2:
                try:
                    value = type(rospy.get_param(global_cost_param + config.config_name, "1"))(config.config_value)
                    rospy.set_param(global_cost_param + config.config_name, value)

                    name_list = config.config_name.split("/")
                    name_dict = dict()
                    name_dict = self.read_param(global_cost_param_url)
                    if len(name_list) == 1:
                        name_dict['global_costmap'][name_list[0]] = value
                    elif len(name_list) == 2:
                        name_dict['global_costmap'][name_list[0]][name_list[1]] = value
                    elif len(name_list) == 3:
                        name_dict['global_costmap'][name_list[0]][name_list[1]][name_list[2]] = value
                    self.writ_patam(global_cost_param_url, name_dict)
                except Exception as e:
                    setconfigsresponse.status = 5
                    return setconfigsresponse
            elif config.type == 3:
                try:
                    value = type(rospy.get_param(navfn_ros_param + config.config_name, "1"))(config.config_value)
                    rospy.set_param(navfn_ros_param + config.config_name, value)
                    testmaper = dict()
                    testmaper = self.read_param(navfn_ros_param_url)
                    testmaper["NavfnROS"][config.config_name] = value
                    self.writ_patam(navfn_ros_param_url, testmaper)

                except Exception as e2:
                    setconfigsresponse.status = 5
                    return setconfigsresponse
            elif config.type == 4:
                try:
                    value = type(rospy.get_param(local_cost_param + config.config_name, "1"))(config.config_value)
                    rospy.set_param(local_cost_param + config.config_name, value)

                    name_list = config.config_name.split("/")
                    name_dict = dict()
                    name_dict = self.read_param(local_cost_param_url)
                    if len(name_list) == 1:
                        name_dict['local_costmap'][name_list[0]] = value
                    elif len(name_list) == 2:
                        name_dict['local_costmap'][name_list[0]][name_list[1]] = value
                    elif len(name_list) == 3:
                        name_dict['local_costmap'][name_list[0]][name_list[1]][name_list[2]] = value
                    self.writ_patam(local_cost_param_url, name_dict)

                except Exception as e2:
                    setconfigsresponse.status = 5
                    return setconfigsresponse
            elif config.type == 5:
                try:
                    value = type(rospy.get_param(dwa_planner_param + config.config_name, "1"))(config.config_value)
                    rospy.set_param(dwa_planner_param + config.config_name, value)
                    testmaper = dict()
                    testmaper = self.read_param(dwa_planner_param_url)
                    testmaper["DWAPlannerROS"][config.config_name] = value
                    self.writ_patam(dwa_planner_param_url, testmaper)
                except Exception as e2:
                    setconfigsresponse.status = 5
                    return setconfigsresponse
            elif config.type == 6:
                try:
                    value = type(rospy.get_param(robot_driver_param + config.config_name, "1"))(config.config_value)
                    rospy.set_param(robot_driver_param + config.config_name, value)
                    testmaper = dict()
                    testmaper = self.read_param(robot_driver_param_url)
                    testmaper[config.config_name] = value
                    self.writ_patam(robot_driver_param_url, testmaper)
                    self.driver_config_pub.publish(1)
                except Exception as e2:
                    setconfigsresponse.status = 5
                    return setconfigsresponse
            elif config.type == 7:
                try:
                    value = type(rospy.get_param(ydlidar_node_param + config.config_name, "1"))(config.config_value)
                    rospy.set_param(ydlidar_node_param + config.config_name, value)
                except Exception as e2:
                    setconfigsresponse.status = 5
                    return setconfigsresponse
            elif config.type == 8:
                try:
                    value = type(rospy.get_param(sensor_imu_param + config.config_name, "1"))(config.config_value)
                    rospy.set_param(sensor_imu_param + config.config_name, value)
                    # TODO:...
                except Exception as e2:
                    setconfigsresponse.status = 5
                    return setconfigsresponse
            elif config.type == 9:
                try:
                    value = type(rospy.get_param(mapserver_param + config.config_name, "1"))(config.config_value)
                    if config.config_value == 'false' or config.config_value == 'False':
                        value = False
                    rospy.set_param(mapserver_param + config.config_name, value)
                    rospy.loginfo(" type: %s , value: %s",config.config_name,value)
                    if str(config.config_name) == "powerThreashold":
                        self.powerThreashold = value
                    if str(config.config_name) == "powerThreasholdMax":
                        self.powerThreasholdMax = value
                    if str(config.config_name) == "task_executing_wait_time":
                        self.task_executing_wait_time = value
                    if str(config.config_name) == "task_circle":
                        self.task_circle = value
                    testmaper = dict()
                    testmaper = self.read_param(mapserver_param_url)
                    testmaper[config.config_name] = value
                    self.writ_patam(mapserver_param_url, testmaper)
                except Exception as e2:
                    setconfigsresponse.status = 5
                    return setconfigsresponse
        rospy.loginfo("设置机器人配置 %d 成功。", request.type)
        return setconfigsresponse

    def del_config_cb(self, request):
        setconfigsresponse = SetOrDelConfigsResponse()
        rospy.loginfo("del_config_cb type = %d ", request.type)
        setconfigsresponse.status = 0
        for config in request.configs:
            if config.type == 1:
                if rospy.has_param(move_base_param + config.config_name):
                    rospy.delete_param(move_base_param + config.config_name)
                else:
                    setconfigsresponse.status = 4
                    return setconfigsresponse
            elif config.type == 2:
                if rospy.has_param(global_cost_param + config.config_name):
                    rospy.delete_param(global_cost_param + config.config_name)
                else:
                    setconfigsresponse.status = 4
                    return setconfigsresponse
            elif config.type == 3:
                if rospy.has_param(navfn_ros_param + config.config_name):
                    rospy.delete_param(navfn_ros_param + config.config_name)
                else:
                    setconfigsresponse.status = 4
                    return setconfigsresponse
            elif config.type == 4:
                rospy.delete_param(local_cost_param + config.config_name)
                if rospy.has_param(global_cost_param + config.config_name):
                    rospy.delete_param(global_cost_param + config.config_name)
                else:
                    setconfigsresponse.status = 4
                    return setconfigsresponse
            elif config.type == 5:
                if rospy.has_param(dwa_planner_param + config.config_name):
                    rospy.delete_param(dwa_planner_param + config.config_name)
                else:
                    setconfigsresponse.status = 4
                    return setconfigsresponse
            elif config.type == 6:
                if rospy.has_param(robot_driver_param + config.config_name):
                    rospy.delete_param(robot_driver_param + config.config_name)
                else:
                    setconfigsresponse.status = 4
                    return setconfigsresponse
            elif config.type == 7:
                if rospy.has_param(ydlidar_node_param + config.config_name):
                    rospy.delete_param(ydlidar_node_param + config.config_name)
                else:
                    setconfigsresponse.status = 4
                    return setconfigsresponse
            elif config.type == 8:
                if rospy.has_param(sensor_imu_param + config.config_name):
                    rospy.delete_param(sensor_imu_param + config.config_name)
                else:
                    setconfigsresponse.status = 4
                    return setconfigsresponse
            elif config.type == 9:
                if rospy.has_param(mapserver_param + config.config_name):
                    rospy.delete_param(mapserver_param + config.config_name)
                else:
                    setconfigsresponse.status = 4
                    return setconfigsresponse
        rospy.loginfo("删除机器人配置 %d 成功。", request.type)
        return setconfigsresponse

    def list_test_points(self, request):
        self.check_map()  # pointlist
        rospy.loginfo("Service call : list_test_points")
        response = ListTestPointsResponse()
        response.list_test_point = self.local_point_list
        if self.map_status == 1 or self.map_status == 2:
            if self.build_points:
                this_app_point = []
                rospy.loginfo("listpoint get %s", len(self.build_points))
                for point in self.build_points:
                    # point = MapsBeanPoint(poin)
                    position = PointListEntry()
                    position.point_id = str(point.getPointId())
                    position.point_name = str(point.getPointName())
                    position.x = point.getPositionX()
                    position.y = point.getPositionY()
                    position.z = point.getPositionYaw()
                    this_app_point.append(position)
                response.list_test_point = this_app_point
            else:
                response.list_test_point = []
        rospy.loginfo("Service call : list_test_points success!")
        return response

    def list_navi_points(self, request):
        self.check_map()  # pointlist
        rospy.loginfo("Service call : list_navi_points")
        response = ListNaviPointsResponse()
        response.list_navi_points = self.local_point_list
        response.list_system_points = self.local_systempoint_list

        if self.map_status == 1 or self.map_status == 2:
            if self.build_points:
                this_app_point = []
                rospy.loginfo("list navi points, len: %s", len(self.build_points))
                for point in self.build_points:
                    # point = MapsBeanPoint(poin)
                    position = PointListEntry()
                    position.point_id = str(point.getPointId())
                    position.point_name = str(point.getPointName())
                    position.x = point.getPositionX()
                    position.y = point.getPositionY()
                    position.z = point.getPositionYaw()
                    this_app_point.append(position)
                response.list_navi_points = this_app_point
            else:
                response.list_navi_points = []

            if self.build_systempoints:
                this_system_point = []
                rospy.loginfo("list system points len1:  %s", len(self.build_systempoints))
                # rospy.loginfo("list system points len2:  %s",type(self.build_systempoints[0]))
                # rospy.loginfo("list system points len3:  %s", self.build_systempoints[0].getPointId())
                for point in self.build_systempoints:
                    # rospy.loginfo("list system points aaaaaaaaa")
                    # point = SystemPoint(poin)
                    position = PointListEntry()
                    position.point_id = str(point.getPointId())
                    position.point_name = str(point.getPointName())
                    position.x = point.getPositionX()
                    position.y = point.getPositionY()
                    position.z = point.getPositionYaw()
                    this_system_point.append(position)
                response.list_system_points = this_system_point
                # rospy.loginfo("list system points bbbbbb")
            else:
                response.list_system_points = []
        rospy.loginfo("Service call : list_navi_points success!")
        return response

    # 切换地图功能
    def publish_map(self, request):
        rospy.loginfo("Service call : publish_map, type: %d , floor_id: %s , map_id: %s" % \
            (request.type, request.floor_id, request.map_id))
        response = PublishMapResponse()
        if request.floor_id == 0 or request.map_id == 0:
            rospy.logerr("publish_map Service call : 无效参数 ")
            return response
        if self.allMap is not None:
            mapSetting = MapSetting(self.allMap)
            # 逐条取出保存，方便后面
            floors = []            
            for fll in mapSetting.getFloors():
                floor = Floor(fll)
                
                if (floor.getFloorId() == request.floor_id):
                    # 找到要更新的楼层, 
                    for uu in floor.getMaps():
                        map = Map(uu)
                        if map.getMapId() == request.map_id:
                            floor.setDefaultmap(request.map_id)
                            break
                floors.append(floor)

            mapSetting.setFloors(floors)
            if request.type == 0:
                mapSetting.setDefaultFloor(request.floor_id)
                
            #rospy.loginfo("mappseting %s", self.bean_to_dict(mapSetting))
            self.write_json(mapSetting, spirit_map_url)  # publish map

            # clear save position
            if request.type == 0:
                if self.save_position_flag !=2:
                    self.save_position_flag = 3
                self.publishlocation_miss = 1
                #停止所有任务
                self.task_stop(True)

                self.check_map(True)  # change map
            else:
                self.wall_update_pub.publish(0)
            rospy.loginfo("机器人切换地图成功，切换地图id：%d", request.map_id)
            return response
        else:
            #self.AndroidmsgReportPub.publish(61)
            self.androidMapStatus_pub.publish(61)
            self.AndroidAlarm.androidmsg_mapstatus = 61
            self.map_ready = False
            self.localmap_ready = False
            rospy.logwarn("机器人地图加载失败！")
        return response

    def set_charpriority(self, request):
        # 设置全局充电桩
        rospy.loginfo("Service call : set_charpriority, request.data: %d", request.data[0])
        response = SetInt64ArrayResponse()
        response.success = True
        self.check_map() 
        if self.allMap is None:
            response.success = False
            response.message = "map is null"
            return response

        mapSetting = MapSetting(self.allMap)
        mapSetting.setChargingPrioritys(request.data)
        self.write_json(mapSetting, spirit_map_url) 

        response.success = True
        response.message = "success"
        self.wall_update_pub.publish(0)
        rospy.loginfo("Service call : set_charpriority success")

        return response
    
    def set_defaultfloor_pointid(self, request):
        # 设置待命点
        rospy.loginfo("Service call : set_defaultfloor_pointid, request.data: %d", request.data)
        response = SetInt64Response()
        response.success = True
        self.check_map()  
        if self.allMap is None:
            response.success = False
            response.message = "map is null"
            return response

        mapSetting = MapSetting(self.allMap)
        mapSetting.setDefaultFloorPointId(request.data)
        self.write_json(mapSetting, spirit_map_url) 

        response.success = True
        response.message = "success"
        self.wall_update_pub.publish(0)
        rospy.loginfo("Service call : set_defaultfloor_pointid success")

        return response
    
    def reset_wait_time(self, request):
        # 清除时间
        rospy.loginfo("Service call : reset_wait_time, request.data: %d", request.data)
        response = SetInt64Response()
 
        self.task_current_wait_time = -1
        response.success = True
        response.message = "success"
        rospy.loginfo("Service call : reset_wait_time success")

        return response
    
    def findDefaultPointIdEntry(self):
        # 根据defaultfloorPointId进行怕暖，如果大于0，则导航到待命点，如果小于0，则去充电，如果等于0，则不设置
        if self.allMap:
            mapSetting = MapSetting(self.allMap)
            defaultFloorPointId = 0
            if mapSetting.getDefaultFloorPointId():
                defaultFloorPointId = mapSetting.getDefaultFloorPointId()
                rospy.loginfo("findDefaultPointIdEntry  defaultFloorPointId = %d",defaultFloorPointId)
            
            # 如果没有设置待命点，直接返回
            if defaultFloorPointId == 0:
                return (False, None)
            
            elif defaultFloorPointId > 0:
                # 去待命点
                for fll in mapSetting.getFloors():
                    floor = Floor(fll)
                    #if (floor.getFloorId() == mapSetting.getDefaultFloor()):
                    for uu in floor.getMaps():
                        map = Map(uu)
                        if map.getMapId() == floor.getDefaultmap():
                            for pin in map.getPoints():
                                point = MapsBeanPoint(pin)
                                if point.getPointId() == defaultFloorPointId:
                                    # 先找待命点
                                    rospy.loginfo("findDefaultPointIdEntry 找到全局待命点")
                                    adddefaultPointGoal = TargetGoalEntry()
                                    adddefaultPointGoal.floor_id = int(floor.getFloorId())
                                    adddefaultPointGoal.floor_name = str(floor.getFloorName())
                                    adddefaultPointGoal.map_id = int(map.getMapId())
                                    adddefaultPointGoal.map_name = str(map.getMapName())
                                    adddefaultPointGoal.point_id = int(point.getPointId())
                                    adddefaultPointGoal.point_name = str(point.getPointName())
                                    adddefaultPointGoal.type = 28    #待命点
                                    rospy.loginfo("机器人开始导航到待命点: %f, %f, %f", point.getPositionX(), point.getPositionY(), point.getPositionYaw())
                                    return (True, adddefaultPointGoal)
            else:
                # 去充电
                chargeFloorId = 0
                if mapSetting.getChargingPrioritys():
                    chargeFloorId = mapSetting.getChargingPrioritys()[0]
                    rospy.loginfo("chargeFloorId = %d",chargeFloorId)
                
                # 如果没有找到跨楼层充电，则在本楼层充电
                if chargeFloorId == 0:
                    chargeFloorId = mapSetting.getDefaultFloor()

                for chargefl in mapSetting.getFloors():
                    chargefloor = Floor(chargefl)
                    if (chargefloor.getFloorId() == chargeFloorId):
                        for uu in chargefloor.getMaps():
                            map = Map(uu)
                            if map.getMapId() == chargefloor.getDefaultmap():
                                for pin in map.getPoints():
                                    point = MapsBeanPoint(pin)
                                    if point.getPointName() == str("充电桩"):
                                        # 先找全局充电桩
                                        rospy.loginfo("AutoCharge_CB 找到普通充电桩")
                                        addchargeGoal = TargetGoalEntry()
                                        addchargeGoal.floor_id = int(chargefloor.getFloorId())
                                        addchargeGoal.floor_name = str(chargefloor.getFloorName())
                                        addchargeGoal.map_id = int(map.getMapId())
                                        addchargeGoal.map_name = str(map.getMapName())
                                        addchargeGoal.point_id = int(point.getPointId())
                                        addchargeGoal.point_name = str(point.getPointName())
                                        addchargeGoal.type = 24
                                        rospy.loginfo("机器人开始导航到全局普通充电桩: %f, %f, %f", point.getPositionX(), point.getPositionY(), point.getPositionYaw())
                                        return (True, addchargeGoal)

                                # 没有找到，寻找当楼层系统充电桩
                                for cpin in map.getSystemPoints():
                                    chargepoint = MapsBeanPoint(cpin)
                                    if chargepoint.getPointName() == str("充电桩"):
                                        addchargeGoal = TargetGoalEntry()
                                        addchargeGoal.floor_id = int(chargeFloorId)
                                        addchargeGoal.floor_name = str(chargefloor.getFloorName())
                                        addchargeGoal.map_id = int(chargefloor.getDefaultmap())
                                        addchargeGoal.map_name = str(map.getMapName())
                                        addchargeGoal.point_id = int(chargepoint.getPointId())
                                        addchargeGoal.point_name = str(chargepoint.getPointName())
                                        addchargeGoal.type = 24
                                        rospy.loginfo("机器人开始导航到全局系统充电桩: %f, %f, %f", chargepoint.getPositionX(), chargepoint.getPositionY(), chargepoint.getPositionYaw())
                                        return (True, addchargeGoal)
                                break
                        break                   
               
        return (False, None)
    
    def add_default_point(self):
        # 增加待命点
        rospy.loginfo("add_default_point")
        (isFind, defaultPointTarget) = self.findDefaultPointIdEntry() # set_mode
        if isFind:
            self.target_goal_lock.acquire()
            self.low_targetgoals.append(defaultPointTarget)
            self.low_targetgoals_backup = self.low_targetgoals
            self.target_goal_lock.release()
            if self.task_executing == 0:
                self.task_executing = 1
                self.get_nexttask = True
                self.task_runs_status = False
        rospy.loginfo("add_default_point success")
        return

    def clear_map(self, request):
        rospy.loginfo("Service call : clear_map")
        response = ClearMapResponse()
        response.result = 0

        if self.map_status == 1:
            #当前处于新建地图模式，不能再新建地图
            rospy.loginfo("当前已经处于新建地图模式，直接返回！！")
            return response

        self.local_point_list = []  # clear_map
        self.local_systempoint_list = []  # clear points
        self.local_globalplans_list = []
        points_posearray = PointPoseArray()
        self.PointsPose_Pub.publish(points_posearray)
        self.map_status = 1
        self.mapStatus_pub.publish(1)
        #self.AndroidmsgReportPub.publish(68)
        self.androidMapBuildStatus_pub.publish(68)
        self.AndroidAlarm.androidmsg_mapbuildstatus = 68
        rospy.loginfo("机器人处于新建地图模式，请扫描新建地图。")
        self.map_loading = True  # map clear
        # self.loading_cartographer_buildMap()  # clear_map

        rospy.loginfo("Service call : clear_map success")
        return response

    def rebuild_map(self, request):
        rospy.loginfo("Service call : rebuild_map")
        response = ClearMapResponse()
        response.result = 0

        if self.map_status == 2:
            #当前处于扩展地图模式，不能再扩展地图
            rospy.loginfo("当前已经处于扩展地图模式，直接返回！！")
            return response

        self.save_position_flag = 1

        #self.local_point_list = []  # rebuild_map
        #self.local_systempoint_list = []  # clear points
        #points_posearray = PointPoseArray()
        #self.PointsPose_Pub.publish(points_posearray)

        # fill build_points
        if self.allMap is None:
            response.result = 2
            return response

        mapSetting = MapSetting(self.allMap)

        # 逐条取出
        floors = []
        floorindex = -1
        isaddfloorindex = True
        maps = []
        for fll in mapSetting.getFloors():
            if isaddfloorindex:
                floorindex= floorindex+1
            floor = Floor(fll)
            floors.append(floor)
            if (floor.getFloorId() == mapSetting.getDefaultFloor()):
                # 找到默认楼层, 结束记录下标, 只记录默认地图的下标
                isaddfloorindex = False

                mapindex = -1
                isaddindex = True
                for uu in floor.getMaps():
                    if (isaddindex):
                        mapindex = mapindex + 1
                    map = Map(uu)
                    maps.append(map)
                    # 逐条取出保存
                    if map.getMapId() == floor.getDefaultmap():
                        isaddindex = False
                rospy.loginfo("rebuild map size = %d, defatmapId = %d", len(maps), mapindex)

                points = []
                if maps[mapindex].getPoints():
                    # 在已有导航点上添加点
                    for pointname in maps[mapindex].getPoints():
                        point = MapsBeanPoint(pointname)
                        points.append(point)

                systemPoints = []
                if maps[mapindex].getSystemPoints():
                    # 在已有导航点上添加或者修改初始点和充电点
                    for pointname in maps[mapindex].getSystemPoints():
                        systempoint = MapsBeanPoint(pointname)
                        systemPoints.append(systempoint)

                self.build_points = points
                self.build_systempoints = systemPoints

        self.map_status = 2
        self.mapStatus_pub.publish(2)
        #self.AndroidmsgReportPub.publish(68)
        self.androidMapBuildStatus_pub.publish(68)
        self.AndroidAlarm.androidmsg_mapbuildstatus = 68
        rospy.loginfo("机器人处于扩展地图模式，请扫描新建地图。")
        self.map_loading = True  # map clear

        rospy.loginfo("Service call : rebuild_map success")
        return response

    def delete_map(self, request):
        rospy.loginfo("Service call : delete map, map_id: %s" % request.map_id)
        response = DeleteMapResponse()
        if not request.map_id:
            return response

        self.check_map() #delete map

        if self.allMap:
            mapSetting = MapSetting(self.allMap)

            # 逐条取出保存，方便后面
            floors = []          

            isFind = False  # type: bool

            for fll in mapSetting.getFloors():
                
                floor = Floor(fll)
                
                if (floor.getFloorId() == request.floor_id):
                    maps = []
                    for uu in floor.getMaps():
                        map = Map(uu)
                        if int(map.getMapId()) == int(request.map_id):
                            # 如果找到，就这个地图不添加备份
                            rospy.loginfo("删除地图 楼层: %s , 地图: %s", floor.getFloorName(), map.getMapName())
                            os.system("rm " + map.getMapFile())
                            os.system("rm " + str(map.getMapFile()).strip('.pbstream')+".*")
                            isFind = True
                            continue
                        maps.append(map)
                    if len(maps) == 0: # 如果楼层中没有地图了，则把楼层清除掉
                        rospy.loginfo("删除楼层 %s",floor.getFloorName())
                        os.system("rm -rf " + spirit_dir+"maps/"+str(floor.getFloorId()))
                        continue
                    rospy.loginfo(" 删除.. 留下的地图楼层 %s", floor.getFloorName())
                    floor.setMaps(maps)
                floors.append(floor)
            if isFind:
                mapSetting.setFloors(floors)
                self.write_json(mapSetting, spirit_map_url)  # map dela
                self.check_map()  # map dela
            self.wall_update_pub.publish(0)
        rospy.loginfo("删除地图成功，map_id: %s" % request.map_id)
        return response
    
    def delete_floor(self, request):
        rospy.loginfo("Service call : delete_floor, floor_id: %s" % request.floor_id)
        response = DeleteMapResponse()
        if not request.floor_id:
            return response

        self.check_map() #delete map

        if self.allMap:
            mapSetting = MapSetting(self.allMap)

            # 逐条取出保存，方便后面
            floors = []
            isFind = False  # type: bool

            for fll in mapSetting.getFloors():                
                floor = Floor(fll)                
                if (floor.getFloorId() == request.floor_id):
                    isFind = True
                    rospy.loginfo("删除楼层 %s",floor.getFloorName())
                    os.system("rm -rf " + spirit_dir+"maps/"+str(floor.getFloorId()))
                    continue
                floors.append(floor)
            if isFind:
                mapSetting.setFloors(floors)
                self.write_json(mapSetting, spirit_map_url)  # map dela
                self.check_map()  # map dela
            self.wall_update_pub.publish(0)
        rospy.loginfo("删除楼层成功，楼层id %s" % request.floor_id)
        return response

    def delete_test_point(self, request):
        rospy.loginfo("Service call : delete point, point_id: %s" % request.point_id)
        response = DeleteTestPointResponse()
        response.result = 0

        if not request.point_id:
            response.result = 1
            return response

        if self.map_status == 1 or self.map_status == 2:
            if self.build_points:
                this_points = []
                for point in self.build_points:
                    # point = MapsBeanPoint(poin)
                    if int(point.getPointId()) == int(request.point_id):
                        continue
                    this_points.append(point)
                self.build_points = this_points
            else:
                response.result = 4
            rospy.loginfo("在建图模式下删除导航点成功, point_id: %s" % request.point_id)
            self.wall_update_pub.publish(0)
            return response

        self.check_map() #del point start

        if self.allMap:
            mapSetting = MapSetting(self.allMap)

            # 逐条取出保存，方便后面
            floors = []
            floorindex = -1
            isaddfloorindex = True
            
            maps = []
            
            points = []
            isFind = False  # type: bool

            for fll in mapSetting.getFloors():
                if isaddfloorindex:
                    floorindex= floorindex+1
                floor = Floor(fll)
                floors.append(floor)
                if (floor.getFloorId() == request.floor_id):
                    # 找到默认楼层, 结束记录下标, 只记录默认地图的下标
                    isaddfloorindex = False

                    mapindex = -1
                    isaddindex = True  # type: bool
                    for uu in floor.getMaps():
                        if isaddindex:
                            mapindex = mapindex + 1
                        map = Map(uu)
                        maps.append(map)
                        # 逐条取出保存
                        if int(map.getMapId()) == int(request.map_id):
                            isaddindex = False
                    rospy.loginfo("point_del map size = %d, defatmapId = %d", len(maps), mapindex)
                    
                    if maps[mapindex].getPoints():
                        for pointname in maps[mapindex].getPoints():
                            point = MapsBeanPoint(pointname)
                            if int(point.getPointId()) == int(request.point_id):
                                nglobalPlan =maps[mapindex].getNGlobalPlans()
                                if nglobalPlan:
                                    if nglobalPlan.getNLines():
                                        for nLineLocal in nglobalPlan.getNLines():
                                            nLineBean = NLine(nLineLocal)
                                            if nLineBean.getStartNid() == int(request.point_id) or nLineBean.getEndNid() == int(request.point_id):
                                                # 被路线引用, 不允许删除
                                                response.result = 4
                                                return response
                                isFind = True
                                continue  # 如果找到，就这个不添加备份
                            points.append(point)
            # 回填map的points
            if isFind:
                maps[mapindex].setPoints(points)
                floors[floorindex].setMaps(maps)
                mapSetting.setFloors(floors)
                self.write_json(mapSetting, spirit_map_url)  # del point
                self.check_map() #del point end
            else:
                response.result = 3

        self.wall_update_pub.publish(0)
        rospy.loginfo("在导航定位模式下删除导航点成功, point_id: %s" % request.point_id)
        return response

    def delete_allmap(self, request):
        rospy.loginfo("Service call : delete all map")
        response = DeleteMapResponse()

        os.system("rm -rf "+spirit_dir+"maps/*")
        self.local_point_list = []  # del allmap
        self.local_systempoint_list = []
        self.local_globalplans_list = []
        self.map_isEmpty = 1

        BAG_PATH = bag_param
        if not os.path.exists(BAG_PATH):
            os.mkdir(BAG_PATH)

        self.check_map()  # delete All
        rospy.loginfo("删除所有地图成功。")
        self.wall_update_pub.publish(0)
        return response

    def rename_map(self, request):
        rospy.loginfo("rename_map Service call : rename floorid: %d , map %d as %s" % \
            (request.floor_id, request.map_id, request.new_name))
        response = RenameMapResponse()

        if request.floor_id == 0 or request.map_id == 0:
            return response
        if not request.new_name:
            return response

        self.check_map()  # rename map read

        if self.allMap:
            mapSetting = MapSetting(self.allMap)

            # 逐条取出保存，方便后面
            floors = []
            floorindex = -1
            isaddfloorindex = True
            
            maps = []
            
            isFind = False  # type: bool

            for fll in mapSetting.getFloors():
                if isaddfloorindex:
                    floorindex= floorindex+1
                floor = Floor(fll)
                floors.append(floor)
                if (floor.getFloorId() == request.floor_id):
                    # 找到要修改的楼层, 结束记录下标, 只记录要修改地图的下标
                    isaddfloorindex = False

                    
                    for uu in floor.getMaps():
                        map = Map(uu)
                        if map.getMapId() == request.map_id:
                            map.setMapName(request.new_name)
                            isFind = True
                        maps.append(map)

            if isFind:
                floors[floorindex].setMaps(maps)
                mapSetting.setFloors(floors)
                self.write_json(mapSetting, spirit_map_url)  # map rename
                self.check_map() #rename map end

        self.wall_update_pub.publish(0)
        rospy.loginfo("重新命名地图 floorid: %d, mapid: %d 为 %s。" % (request.floor_id, request.map_id, request.new_name))
        return response
    
    def rename_floor(self, request):
        rospy.loginfo("rename_floor Service call : rename floorid: %d  as %s" % \
            (request.floor_id, request.new_name))
        response = RenameMapResponse()

        if request.floor_id == 0:
            return response
        if not request.new_name:
            return response

        self.check_map()  # rename map read

        if self.allMap:
            mapSetting = MapSetting(self.allMap)

            # 逐条取出保存，方便后面
            floors = []           
            isFind = False  # type: bool

            for fll in mapSetting.getFloors():
                floor = Floor(fll)
                
                if (floor.getFloorId() == request.floor_id):
                    # 找到要修改的楼层, 结束记录下标, 只记录要修改地图的下标
                    floor.setFloorName(request.new_name)
                    isFind = True
                floors.append(floor)

            if isFind:
                mapSetting.setFloors(floors)
                self.write_json(mapSetting, spirit_map_url)  # map rename
                self.check_map() #rename map end

        self.wall_update_pub.publish(0)
        rospy.loginfo("重新命名楼层名 floorid: %d 为 %s。" % (request.floor_id, request.new_name))
        return response

    def point_rename(self, request):
        rospy.loginfo("Service call : rename type %d  point %s as %s" % (request.type, request.point_id, request.new_name))
        response = PointRenameResponse()
        response.result = 0

        if not request.point_id:
            response.result = 1
            return response
        if not request.type == 5 and not request.new_name:
            response.result = 2
            return response
        if self.map_status == 1 or self.map_status == 2:
            if self.build_points:
                this_points = []
                for point in self.build_points:
                    # point = MapsBeanPoint(poin)
                    if int(point.getPointId()) == int(request.point_id):
                        point.setPointName(request.new_name)
                    this_points.append(point)
                self.build_points = this_points
            else:
                response.result = 4
            rospy.loginfo("在建图模式下，重新命名 type:%d ,导航点名: %d, 新 %s" % (request.type, request.point_id, request.new_name))
            self.wall_update_pub.publish(0)
            return response
        self.check_map() # rename point ,map isbuild

        if self.allMap:
            mapSetting = MapSetting(self.allMap)

            # 逐条取出保存，方便后面
            floors = []
            floorindex = -1
            isaddfloorindex = True
            maps = []

            points = []
            isFind = False

            for fll in mapSetting.getFloors():
                if isaddfloorindex:
                    floorindex= floorindex+1
                floor = Floor(fll)
                floors.append(floor)

                if (floor.getFloorId() == mapSetting.getDefaultFloor()):
                    # 找到默认楼层, 结束记录下标, 只记录默认地图的下标
                    isaddfloorindex = False
                    rospy.loginfo("找到默认楼层")

                    mapindex = -1
                    isaddindex = True
                    for uu in floor.getMaps():
                        if (isaddindex):
                            mapindex = mapindex + 1
                        map = Map(uu)
                        maps.append(map)
                        # 逐条取出保存
                        if map.getMapId() == floor.getDefaultmap():
                            isaddindex = False
                    rospy.loginfo("point_rename map size = %d, defatmapId = %d", len(maps), mapindex)
                    

                    if maps[mapindex].getPoints():
                        # rospy.loginfo("points find is not Null ")
                        for pointname in maps[mapindex].getPoints():
                            point = MapsBeanPoint(pointname)
                            rospy.loginfo("point id = %d", point.getPointId())
                            if int(point.getPointId()) == int(request.point_id):
                                rospy.loginfo("has isFind the pointid")
                                isFind = True
                                if request.type == 0:
                                    point.setPointName(request.new_name)
                                    rospy.loginfo("发现导航点重命名")
                                if request.type == 1: # 认为是添加等待点 
                                    waitPoints = []
                                    if (point.getWaitPoints):
                                        for waitpoin in point.getWaitPoints():
                                            waitpoint = Position(waitpoin)
                                            # if waitpoint.getX
                                            waitPoints.append(waitpoin)
                                        # 先保存旧的 再增加新的
                                        newwpotin = Position()
                                        # 获取当前位置
                                        rospy.loginfo("发现等待点新增")
                                        poseset = self.get_pos()
                                        newwpotin.setX(poseset.x)
                                        newwpotin.setY(poseset.y)
                                        waitPoints.append(newwpotin)
                                    else:
                                        # 没找到数据库 直接加
                                        newwpotin = Position()
                                        rospy.loginfo("添加第一个等待点")
                                        # 获取当前位置
                                        poseset = self.get_pos()
                                        newwpotin.setX(poseset.x)
                                        newwpotin.setY(poseset.y)
                                        waitPoints.append(newwpotin)
                                    point.setWaitPoints(waitPoints)

                                if request.type == 5:
                                    rospy.loginfo("修改呼叫器id")
                                    point.setDeviceID(request.new_name)

                            #rospy.loginfo("point id = %s", point.getPointName())
                            points.append(point)
            # 回填map的points
            if isFind:
                maps[mapindex].setPoints(points)
                floors[floorindex].setMaps(maps)
                mapSetting.setFloors(floors)
                self.write_json(mapSetting, spirit_map_url)  # point rename
                self.check_map() #rename point end
            else:
                rospy.logwarn("point rename Modification not detected")
                response.result = 3
        else:
            response.result = 4
        self.wall_update_pub.publish(0)
        rospy.loginfo("在导航定位模式下，重新命名导航点名 %s 为 %s" % (request.point_id, request.new_name))
        return response
    
    def point_update(self,request):
        rospy.loginfo("point_update %s", request.data)
        response = PointUpdateResponse()
        if not request.point_id:
            response.result = 1
            return response
        self.check_map() # point_update 

        if self.allMap:
            mapSetting = MapSetting(self.allMap)
            # 逐条取出保存，方便后面
            floors = []
            for fll in mapSetting.getFloors():
                floor = Floor(fll)
                if (floor.getFloorId() == request.floor_id):
                    # 找到要修改的楼层
                    rospy.loginfo("point_update 找到默认楼层")
                    maps = []
                    for uu in floor.getMaps():
                        map = Map(uu)
                        points = []
                        if map.getMapId() == request.map_id:
                            # 找到要修改的地图
                            for pointname in map.getPoints():
                                point = MapsBeanPoint(pointname)
                                # rospy.loginfo("point_update point id = %d", point.getPointId())
                                if int(point.getPointId()) == int(request.point_id):
                                    # 找到要修改的导航点
                                    rospy.loginfo("point_update has isFind the point %s",point.getPointName())
                                    # 直接覆盖写入 (需要json转字典才能用)
                                    newpoint = MapsBeanPoint(json.loads(request.data))
                                    point = newpoint
                                points.append(point)
                            map.setPoints(points)
                        maps.append(map)
                    floor.setMaps(maps)
                floors.append(floor) # 每层楼的都要加
            mapSetting.setFloors(floors)
            response.result = 0
            self.write_json(mapSetting, spirit_map_url)  # point rename
            self.check_map() #rename point end
            self.wall_update_pub.publish(0)

        return response

    def get_maps_cb(self,request):
        #rospy.loginfo("Service call : getMaps")  app每一秒读取一次，打印取消
        mapresponse = TriggerResponse()
        mapresponse.success = False

        try:
            with io.open(spirit_map_url, 'r', encoding='utf-8') as f:
                content = f.read()
                mapresponse.success = True
                mapresponse.message = str(content)
        except Exception as e:
            mapresponse.success = False
        rospy.loginfo("上层获取地图信息所有信息成功。")
        return mapresponse

    def list_androidalarm_report_cb(self,request):
        rospy.loginfo("Service call : list_androidalarm_report")
        anddroidalarmresportresponse = AndroidAlarmReportResponse()
        anddroidalarmresportresponse.androidmsg_chargestatus = self.AndroidAlarm.androidmsg_chargestatus
        anddroidalarmresportresponse.androidmsg_navigationstatus = self.AndroidAlarm.androidmsg_navigationstatus
        anddroidalarmresportresponse.androidmsg_outofchargepoint = self.AndroidAlarm.androidmsg_outofchargepoint
        anddroidalarmresportresponse.androidmsg_locationstatus = self.AndroidAlarm.androidmsg_locationstatus
        anddroidalarmresportresponse.androidmsg_motor1status = self.AndroidAlarm.androidmsg_motor1status
        anddroidalarmresportresponse.androidmsg_sonarstatus = self.AndroidAlarm.androidmsg_sonarstatus
        anddroidalarmresportresponse.androidmsg_leftmotorstatus = self.AndroidAlarm.androidmsg_leftmotorstatus
        anddroidalarmresportresponse.androidmsg_rightmotorstatus = self.AndroidAlarm.androidmsg_rightmotorstatus
        anddroidalarmresportresponse.androidmsg_camerastatus = self.AndroidAlarm.androidmsg_camerastatus
        anddroidalarmresportresponse.androidmsg_motor2status = self.AndroidAlarm.androidmsg_motor2status
        anddroidalarmresportresponse.androidmsg_batterystatus = self.AndroidAlarm.androidmsg_batterystatus
        anddroidalarmresportresponse.androidmsg_emergencystatus = self.AndroidAlarm.androidmsg_emergencystatus
        anddroidalarmresportresponse.androidmsg_motorenabledstatus = self.AndroidAlarm.androidmsg_motorenabledstatus
        anddroidalarmresportresponse.androidmsg_mapstatus = self.AndroidAlarm.androidmsg_mapstatus
        anddroidalarmresportresponse.androidmsg_virtualmapstatus = self.AndroidAlarm.androidmsg_virtualmapstatus
        anddroidalarmresportresponse.androidmsg_mapbuildstatus = self.AndroidAlarm.androidmsg_mapbuildstatus
        anddroidalarmresportresponse.androidmsg_initstatus = self.AndroidAlarm.androidmsg_initstatus
        anddroidalarmresportresponse.androidmsg_lidarstatus = self.AndroidAlarm.androidmsg_lidarstatus
        anddroidalarmresportresponse.androidmsg_imustatus = self.AndroidAlarm.androidmsg_imustatus
        anddroidalarmresportresponse.androidmsg_stm32status = self.AndroidAlarm.androidmsg_stm32status
        anddroidalarmresportresponse.androidmsg_odomstatus = self.AndroidAlarm.androidmsg_odomstatus
        anddroidalarmresportresponse.androidmsg_navinetworkstatus = self.AndroidAlarm.androidmsg_navinetworkstatus

        rospy.loginfo("获取导航版向安卓上报所有信息成功。")
        return anddroidalarmresportresponse

    def save_map(self, request):
        rospy.logerr("save_map Service call : save current map as type:%d , floor_id:%d, floor_name:%s, map_name:%s"\
             % (request.type, request.floor_id, request.floor_name, request.map_name))
        # type: 0 保存当前楼层下 1 保存其他指定楼层下 2 保存新楼层
        response = SaveMapResponse()

        recordbag = RecordBag()
        recordbag.type = 4
        #self.sendToRecordbag_pub.publish(recordbag)



        local_time = self.getId()
        local_time = int(local_time)





        if request.floor_id == 0:
            if request.type == 0 or request.type == 1:
                response.status = 200
                response.message = "无效参数, 缺失 floor_id"
                return response

        
        #if request.type == 0:  or
        #if request.type == 1:
        floors_path =spirit_dir+"maps/" + str(request.floor_id) # 楼层文件夹, type = 0 1 都要传 request.floor_id
        
        pbsi = spirit_dir+"maps/" + str(request.floor_id) + "/" + str(local_time)
        pbs = spirit_dir+"maps/"  + str(request.floor_id) + "/" + str(local_time) + ".pbstream"
        mappgm = spirit_dir+"maps/" + str(request.floor_id) + "/" + str(local_time) + ".pgm"
        mapyaml = spirit_dir+"maps/" + str(request.floor_id) + "/" + str(local_time) + ".yaml"
        if request.type == 2:
            
            floors_path =spirit_dir+"maps/" + str(local_time)

            pbsi = spirit_dir+"maps/" + str(local_time) + "/" + str(local_time)
            pbs = spirit_dir+"maps/"  + str(local_time) + "/" + str(local_time) + ".pbstream"
            mappgm = spirit_dir+"maps/"  + str(local_time) + "/" + str(local_time) + ".pgm"
            mapyaml = spirit_dir+"maps/" + str(local_time) + "/" + str(local_time) + ".yaml"
            
        # 新的楼层没有那个楼层文件夹，需要重新创建
        if not os.path.exists(floors_path):
            os.mkdir(floors_path)


        #停止接收子图，避免地图过小保存不了，或在地图过大最后一张子图未绘制完整
        cartographer_response = self.finish_trajectory(0)

        cartographer_response_status = StatusResponse()
        cartographer_response_status = cartographer_response.status
        response.status = cartographer_response_status.code
        response.message = cartographer_response_status.message

        #rospy.logerr("cartographer test 1 %d",response.status)
        #如果停止子图不成功，则返回错误
        if not response.status == 0:
            rospy.logerr("cartographer停止子图失败，code = %d message : %s", cartographer_response_status.code,
                         cartographer_response_status.message)
            return response

        cartographer_response = self.writestate(pbs, True)

        cartographer_response_status = cartographer_response.status
        response.status = cartographer_response_status.code
        response.message = cartographer_response_status.message

        #如果cartographer保存地图不成功，则返回错误
        if not response.status == 0:
            os.system("rm " + pbsi + ".*")
            rospy.logerr("cartograph2r保存地图失败，code = %d message : %s", cartographer_response_status.code,

                         cartographer_response_status.message)
            return response

        cartofile_exist = os.path.exists(pbs)
        if (cartofile_exist == False):
            response.status = 100
            os.system("rm " + pbsi + ".*")
            response.message = "保存slam文件失败，slam文件不存在！"
            rospy.logerr("cartographer保存文件失败，slam文件不存在！！")
            return response

        cartofile_size = os.path.getsize(pbs)
        if (cartofile_size ==0):
            response.status = 101
            os.system("rm " + pbsi + ".*")
            response.message = "保存slam文件失败，大小为0！"
            rospy.logerr("cartographer保存文件失败，字节大小为0！！")
            return response

        #如果没保存成功则返回错误
        if not response.status == 0:
            os.system("rm " + pbsi + ".*")
            rospy.logerr("cartographer保持地图失败，code = %d message : %s", cartographer_response_status.code,
                          cartographer_response_status.message)
            return response

        #转换地图为mapserver可读(假定不可能失败)
        os.system("rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=" + pbsi + " -pbstream_filename=" + pbs + " -resolution=" + str(rospy.get_param(mapserver_param + "resolution", 0.05)))

        pgmfile_exist = os.path.exists(mappgm)
        if (pgmfile_exist == False):
            response.status = 102
            os.system("rm " + pbsi + ".*")
            response.message = "保存PGM文件失败，PGM文件不存在！"
            rospy.logerr("PGM文件转换失败，PGM文件不存在！！")
            return response

        pgmfile_size = os.path.getsize(mappgm)
        if (pgmfile_size ==0):
            response.status = 103
            os.system("rm " + pbsi + ".*")
            response.message = "保存PGM文件失败，大小为0！"
            rospy.logerr("PGM文件转换失败，字节大小为0！！")
            return response

        yamlfile_exist = os.path.exists(mapyaml)
        if (yamlfile_exist == False):
            response.status = 104
            os.system("rm " + pbsi + ".*")
            response.message = "保存YAML文件失败，YAML文件不存在！"
            rospy.logerr("YAML文件转换失败，YAML文件不存在！！")
            return response

        yamlfile_size = os.path.getsize(mapyaml)
        if (yamlfile_size ==0):
            response.status = 105
            os.system("rm " + pbsi + ".*")
            response.message = "保存YAML文件失败，大小为0！"
            rospy.logerr("YAML文件转换失败，字节大小为0！！")
            return response

        rospy.loginfo("Savemap, 保存新地图文件%s成功。", pbsi)

        # save the map information  保存地图信息
        self.check_map()  #as save map

        mapSetting = MapSetting()
        # 此时认为没有地图
        if not self.allMap:
            rospy.loginfo("No floor and map , Add map for build floor!")
        
            # 楼层
            floors = []
            floor = Floor()
            # 单层地图
            maps = []
            map = Map()

            map.setMapId(local_time)
            map.setMapName(str(request.map_name))
            map.setMapFile(pbs)
            map.setPoints(self.build_points)
            map.setSystemPoints(self.build_systempoints)
            if self.build_systempoints:
                for point in self.build_systempoints:
                    # point = MapsBeanPoint(poin)
                    if point.getPointName() == "充电桩":
                        map.setChargeId(point.getPointId())
                    if point.getPointName() == "初始点":
                        map.setInitialId(point.getPointId())

            # 该层单个地图
            maps.append(map)
            # 组装该层所有地图

            floor.setDefaultmap(map.getMapId())
            floor.setMaps(maps)
            floor.setFloorId(local_time)
            floor.setFloorName(request.floor_name)
            

            # 组装楼层
            floors.append(floor)
            mapSetting.setDefaultFloor(floor.getFloorId())
            mapSetting.setFloors(floors)
        else:  # 此时认为有地图
            rospy.loginfo("add one new map!")
            mapSetting = MapSetting(self.allMap)

            # 找到扩展之前的地图信息
            kdf_map = Map()
            for fll in mapSetting.getFloors():
                floor = Floor(fll)
                if floor.getFloorId() == mapSetting.getDefaultFloor():
                    for uu in floor.getMaps():
                        map = Map(uu)
                        if map.getMapId() == floor.getDefaultmap():
                            kdf_map = map
                            break
                    break


            # 逐条取出保存，方便后面添加
            
            saveFloorId = 0
            # 如果是保存当前楼层内 
            if request.type == 0:
                saveFloorId = mapSetting.getDefaultFloor()
            # 如果是保存其他已知的楼层内
            if request.type == 1:
                saveFloorId = request.floor_id

            # 如果是新楼层新地图保存
            if request.type == 2:
                floors = []
                for fll in mapSetting.getFloors():
                    ofloor = Floor(fll)
                    floors.append(ofloor)

                # 单层地图
                maps = []
                map = Map()

                # 如果是扩展地图 要把扩展之前的地图信息全赋值进去
                if self.map_status == 2:
                    map = kdf_map

                map.setMapId(local_time)
                map.setMapName(str(request.map_name))
                map.setMapFile(pbs)
                map.setPoints(self.build_points)
                map.setSystemPoints(self.build_systempoints)
                if self.build_systempoints:
                    for point in self.build_systempoints:
                        # point = MapsBeanPoint(poin)
                        if point.getPointName() == "充电桩":
                            map.setChargeId(point.getPointId())
                        if point.getPointName() == "初始点":
                            map.setInitialId(point.getPointId())

                # 该层单个地图
                maps.append(map)
                # 组装该层所有地图
                floor = Floor()
                floor.setDefaultmap(map.getMapId())
                floor.setMaps(maps)
                floor.setFloorId(local_time)
                floor.setFloorName(request.floor_name)
                floors.append(floor)
                mapSetting.setDefaultFloor(floor.getFloorId())
                mapSetting.setFloors(floors)
                
            else:
                # 如果是保存已有楼层内保存新地图
                floors = []
                for fll in mapSetting.getFloors():
                    floor = Floor(fll)
                    
                    if (floor.getFloorId() == saveFloorId):
                        # 设置默认楼层
                        
                        # 取出虚拟墙和全局路径，再次保存
                        old_maps = []
                        for uu in floor.getMaps():
                            odl_map = Map(uu)
                            old_maps.append(odl_map)
                        
                        map = Map()
                        # 如果是扩展地图 要把扩展之前的地图信息全赋值进去
                        if self.map_status == 2:
                            map = kdf_map
                        map.setMapId(local_time)
                        map.setMapName(str(request.map_name))
                        map.setMapFile(pbs)
                        map.setPoints(self.build_points)
                        map.setSystemPoints(self.build_systempoints)

                        if self.build_systempoints:
                            for point in self.build_systempoints:
                                # point = MapsBeanPoint(poin)
                                if point.getPointName() == "充电桩":
                                    map.setChargeId(point.getPointId())
                                if point.getPointName() == "初始点":
                                    map.setInitialId(point.getPointId())
                        old_maps.append(map)
                        floor.setDefaultmap(map.getMapId())
                        floor.setMaps(old_maps)
                    floors.append(floor)
                mapSetting.setDefaultFloor(saveFloorId)
                mapSetting.setFloors(floors)

        rospy.loginfo("所有map数据： %s", self.bean_to_dict(mapSetting))
        self.write_json(mapSetting, spirit_map_url)  # save map
        self.publishlocation_miss = 3
        self.check_map(True)    #save map
        # save current position
        self.save_position_flag = 1
        self.build_points = []
        self.build_systempoints = []
        self.wall_update_pub.publish(0)
        rospy.loginfo("保存地图成功。")
        return response

    def saveerase_map(self, request):
        rospy.loginfo("Service call : save erase map as %s" % request.map_name)
        response = SaveMapResponse()

        if (self.map_status == 1 or self.map_status == 2):
            response.status = 108
            return response

        mapfile = os.path.splitext(self.map_file)[0]

        os.system("rosrun map_server map_saver -f " + mapfile)
        rospy.loginfo("保存橡皮檫修改的地图成功。")
        return response
    
    def save_ext_map(self, request):
        rospy.loginfo("Service call : save ext map")
        response = SaveMapStringResponse()
        response.status = 0
        if (self.map_status == 1 or self.map_status == 2):
            response.status = 108
            return response
        base64_png = request.base64

        # rospy.loginfo("save_ext_map base64: %s",base64_png)

        img_data = base64.b64decode(base64_png)
        nparr = np.frombuffer(img_data, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)  # BGR格式
        
        height, width = img.shape[:2]
        
        # 翻转图像Y轴
        img_flipped = cv2.flip(img, 0)
        
        # 使用numpy进行高效的颜色映射
        b, g, r = cv2.split(img_flipped)
        data = np.full((height, width), -1, dtype=np.int8)
        
        # 空闲区域（白色）
        free_mask = (b == 0xFF) & (g == 0xFF) & (r == 0xFF)
        data[free_mask] = 0
        
        # 占据区域（特定颜色）
        occupied_mask = (b == 0xF9) & (g == 0x7F) & (r == 0x14)
        data[occupied_mask] = 100
        
        # 转换为列表并设置
    
        self.sendmapdata.data = data.flatten().tolist()

        self.mapupdate_pub.publish(self.sendmapdata)

        mapfile = os.path.splitext(self.map_file)[0]
        rospy.loginfo("save_ext_map mapfile: %s",mapfile)
        os.system("rosrun map_server map_saver -f " + mapfile)
        rospy.loginfo("save_ext_map 保存橡皮檫修改的地图成功。")

        return response


    def target_goal_plan(self, msg):
        rospy.loginfo("target_goal_plan 收到消息 type%d , 楼层 %d , 地图 %d , 导航 %d", msg.type, msg.floor_id, msg.map_id, msg.point_id)
        
        response = TargetGoalPlanResponse()
        response.status = -1
        response.distance = -1.0
        if not self.odom_ready:
            return response
        
        if msg.floor_id == 0 or msg.map_id == 0 or msg.point_id == 0:
            rospy.logwarn("target_goal_plan 导航信息错误")
            return response

        if self.need_navigation == False:
            # 上报一个告警，在安全模式下导航，报定位已丢失
            self.AndroidAlarm.androidmsg_navigationstatus = 84
            #self.androidNavigationStatus_pub.publish(84)
            rospy.loginfo("target_goal_plan 机器人初始化后没有进行重定位成功，进入旋转模式！")
        #  清空list
        self.task_stop(True)

        addoneGoal = TargetGoalEntry()
        addoneGoal.floor_id = int(msg.floor_id)
        addoneGoal.map_id = int(msg.map_id)
        addoneGoal.point_id = int(msg.point_id)
        self.need_gotodefaultPoint = True

        mapSetting_bean = MapSetting(self.allMap)
        for floor in mapSetting_bean.getFloors():   
            floorbean = Floor(floor)
            if addoneGoal.floor_id == floorbean.getFloorId():
                addoneGoal.floor_name = floorbean.getFloorName()
                for map in floorbean.getMaps():
                    mapbean = Map(map)
                    if addoneGoal.map_id == mapbean.getMapId():
                        addoneGoal.map_name = mapbean.getMapName()
                        for point in mapbean.getPoints():
                            pointBean = MapsBeanPoint(point)
                            if addoneGoal.point_id == pointBean.getPointId():
                                addoneGoal.point_name = pointBean.getPointName()
                                addoneGoal.floor_id_putdown = pointBean.getBindFloorId()
                                addoneGoal.map_id_putdown = pointBean.getBindMapId()
                                addoneGoal.point_id_putdown = pointBean.getBindPointId()
                                if pointBean.getLedWarnID():
                                    addoneGoal.led_warn_id = int(pointBean.getLedWarnID())
                                break
                        
                        if addoneGoal.point_id_putdown !=0:
                            for bindpoint in mapbean.getPoints():
                                bindpointBean = MapsBeanPoint(bindpoint)
                                if addoneGoal.point_id_putdown == bindpointBean.getPointId():
                                    addoneGoal.point_name_putdown = bindpointBean.getPointName()
                                    break
        if addoneGoal.point_id_putdown != 0:
            addoneGoal.type = 22
        else:    
            addoneGoal.type = 20        

        self.target_goal_lock.acquire()
        self.low_targetgoals.append(addoneGoal)
        self.low_targetgoals_backup = self.low_targetgoals
        self.target_goal_lock.release()
        if self.task_executing == 0:
            self.task_executing = 1
            self.get_nexttask = True
            self.task_runs_status = False

        rospy.loginfo("target_goal_plan low_targetgoals:")
        for goal in self.low_targetgoals:
            rospy.loginfo("target_goal_plan low_target goal: floorid: %d floorname: %s mapid: %d mapname: %s pointid: %d pointname: %s", goal.floor_id, str(goal.floor_name), goal.map_id, str(goal.map_name), goal.point_id, str(goal.point_name))
    
        return response
    

    def set_default_point(self, request):
        rospy.loginfo("Service call : set defaultpoint_name %s" % request.defaultpoint_name)
        response = SetDefaultPointResponse()
        return response

    def set_mode(self, request):
        
        rospy.loginfo("Service call : set mode %d" % request.mode)
        # 1: autocharge, 2: manumode
        response = SetModeResponse()
        if not self.odom_ready:
            return response

        if self.need_navigation == False:
            # 上报一个告警，在安全模式下导航，报定位已丢失
            self.AndroidAlarm.androidmsg_navigationstatus = 84
            #self.androidNavigationStatus_pub.publish(84)
            rospy.loginfo("机器人初始化后没有进行重定位成功，进入旋转模式！")

        if not self.local_systempoint_list:
            rospy.loginfo("机器人没有设置充电桩！")
            self.AndroidAlarm.androidmsg_navigationstatus = 52
            #self.AndroidmsgReportPub.publish(52)
            #self.androidNavigationStatus_pub.publish(52)  # 找不到充电桩
            return response

        if request.mode == 1:
            if self.current_jacking_status==1:
                rospy.loginfo("机器人处于顶升状态！")
                return
            rospy.loginfo("机器人设置为自动回充模式成功。")
            if (self.map_driverinfo.ChargeStatus > 0 and self.map_driverinfo.ChargeStatus <5):
                rospy.loginfo("机器人已经在充电中！！！")
                return response
            (isFind, changeTarget) = self.findChargeTargetGoalEntry() # set_mode
            if isFind:
                #  清空list
                self.task_stop(True)

                self.target_goal_lock.acquire()
                self.low_targetgoals.append(changeTarget)
                self.low_targetgoals_backup = self.low_targetgoals
                self.target_goal_lock.release()
                if self.task_executing == 0:
                    self.task_executing = 1
                    self.get_nexttask = True
                    self.task_runs_status = False
                self.waitfornavi = True

            else: # 如果没找到
                rospy.loginfo("机器人没有设置充电桩！")
                self.AndroidAlarm.androidmsg_navigationstatus = 52
                #self.AndroidmsgReportPub.publish(52)
                #self.androidNavigationStatus_pub.publish(52)  # 找不到充电桩

        elif request.mode == 2:
            rospy.loginfo("机器人设置为MANU模式成功。")
            # manu mode
            self.mode = 2
            self.waitfornavi = False
            self.motionmode_pub.publish(2)
            self.task_stop(True)
        return response

    def set_navi_cmd(self, request):
        if (int(request.cmd) == 1):
            rospy.loginfo("Service call : 下发前进命令, distance: %d" % (request.distance))
            if not self.testodom_switch:
                self.testodom_switch = True
                self.startLinear(float(request.distance)/100) #传过来的单位是厘米，转为米
        elif (int(request.cmd) == 2):
            rospy.loginfo("Service call : 下发后退命令, distance: %d" % (request.distance))
            if not self.testodom_switch:
                self.testodom_switch = True
                self.startLinear(-1*float(request.distance)/100)
        elif (int(request.cmd) == 3):
            rospy.loginfo("Service call : 下发左旋转命令, distance: %d" % (request.distance))
            if not self.testodom_switch:
                self.testodom_switch = True
                self.startAngular(0, request.distance)
        elif (int(request.cmd) == 4):
            rospy.loginfo("Service call : 下发右旋转命令, distance: %d" % (request.distance))
            if not self.testodom_switch:
                self.testodom_switch = True
                self.startAngular(0, -request.distance)
        elif (int(request.cmd) == 5):
            rospy.loginfo("Service call : 下发机器人停止命令, distance: %d" % (request.distance))
        elif (int(request.cmd) == 6):
            rospy.loginfo("Service call : 下发导航暂停命令")
        elif (int(request.cmd) == 7):
            rospy.loginfo("Service call : 下发导航重新启动命令")
        response = SetNaviCmdResponse()
        ## 1:  forwarding
        ## 2:  backwarding
        #  3:  left rorate
        #  4:  right rotate
        #  5:  stop
        #  6:  Navigation pause
        #  7:  Navigation start
        try:
            if (self.map_driverinfo.MotorInitStatus == 1):
                drivercmd = AndroidCmd()
                drivercmd.cmd = int(request.cmd)
                drivercmd.distance = request.distance
                valid_commands = {1, 2, 3, 4}  # 过滤移动命令不走驱动
                if int(request.cmd) not in valid_commands:
                    self.androidcmd_pub.publish(drivercmd)
                if (drivercmd.cmd == 5):
                    self.is_across_floor = False # 重置跨地图参数

                    # cancel navigation
                    self.task_stop(True)
                    self.task_executing = 0
                    self.task_runs_status = False
                    self.high_targetgoals = []
                    self.low_targetgoals = []
                    self.subtask_targetgoals = []
                    self.high_targetgoals_backup = []
                    self.low_targetgoals_backup = []

                    self.mode = 2
                    self.waitfornavi = False

                    self.motionmode_pub.publish(2)
                    #self.AndroidmsgReportPub.publish(83)
                    #self.androidNavigationStatus_pub.publish(83)
                    self.AndroidAlarm.androidmsg_navigationstatus = 83
                    rospy.loginfo("机器人导航取消！")

                    if self.testodom_switch:
                        #取消odom测试
                        self.testodom_pub.publish(10000)
                        self.testodom_switch = False

                else:
                    if self.laser_switch == 0:
                        self.laser_switch = 1
                        self.laserswitch_pub.publish(1)

            rospy.loginfo("Service call : set navicmd %s distance %d success!" % (request.cmd, request.distance))
            return response
        except:
            rospy.loginfo("mapmanager, set navicmd error")
            return response
    #上报wifi数据
    def wifiUpdate(self):
        wifiArray = WifiArray()
        for wifi_name in self.wifilist.keys():
            wifi = WifiEntry()
            wifi.wifi_name = wifi_name
            wifi.wifi_pass = self.wifilist[wifi_name]
            wifiArray.wifis.append(wifi)
            rospy.loginfo("机器人wifi上报 名称： %s， 密码： %s 成功。" % (wifi.wifi_name,  wifi.wifi_pass))
        self.update_wifi_pub.publish(wifiArray)

    def set_wifi(self, request):
        rospy.loginfo("Service call : set wifi %s, pass: %s" % (request.wifi_name, request.wifi_pass))
        response = SetWifiResponse()

        try:
            os.system("rm -rf /etc/NetworkManager/system-connections/*")
            os.system("nmcli device wifi rescan")
            if (request.wifi_name != ""):
                os.system("nmcli device wifi connect " + request.wifi_name + " password " + request.wifi_pass + " ifname wlp3s0")

                wifilist_tmp = dict()
                wifilist_tmp.clear()
                wifiname = request.wifi_name

                if not bool(self.wifilist):
                    wifilist_tmp[wifiname] = request.wifi_pass
                    self.wifilist = wifilist_tmp
                else:
                    self.wifilist[wifiname] = request.wifi_pass
                self.writ_patam(wifi_param_url,self.wifilist)
                rospy.loginfo("机器人配置wifi名 %s， 密码： %s 成功。" % (request.wifi_name, request.wifi_pass))
            time.sleep(15)
            self.connectwifi()
            return response
        except:
            rospy.loginfo("机器人设置wifi异常！！！")
            return response

    def list_driver_status(self, request):
        response = ListDriverStatusResponse()
        rospy.loginfo("Service call : list driver status")
        if not self.odom_ready:
            return response

        try:
            if (self.map_driverinfo.MotorInitStatus == 1):
                response.driverstatus = 3
            response.emergerncy = self.map_driverinfo.EmergencyStatus
            response.power = self.map_driverinfo.Power

            rospy.loginfo("Service call : list driver status success!")
            return response
        except:
            rospy.loginfo("mapmanager, list driver status error")
            return response

    def get_driver_status(self, request):
        response = ListDriverStatusResponse()
        if (self.map_driverinfo.MotorInitStatus == 99 or self.map_driverinfo.MotorInitStatus == 0):
            return response

        try:
            if (self.map_driverinfo.MotorInitStatus == 1):
                response.driverstatus = 3
            response.emergerncy = self.map_driverinfo.EmergencyStatus
            response.power = self.map_driverinfo.Power

            if (self.map_driverinfo.EmergencyStatus == 1):
                #self.AndroidmsgReportPub.publish(31)
                self.androidEmergencyStatus_pub.publish(31)
                self.AndroidAlarm.androidmsg_emergencystatus = 31
            else:
                #self.AndroidmsgReportPub.publish(32)
                self.androidEmergencyStatus_pub.publish(32)
                self.AndroidAlarm.androidmsg_emergencystatus = 32
            self.PowerReportPub.publish(self.map_driverinfo.Power)
            rospy.loginfo("Service call : app get driver status success!")
            return response
        except:
            rospy.loginfo("mapmanager, app get driver status error")
            return response

    def get_position(self, request):
        rospy.loginfo("Service call : get position")
        response = GetPositionResponse()

        if not self.map_ready:
            rospy.logwarn("map tf can not find")
            return response
        try:
            poseset = Point32()
            poseset = self.get_pos()
            response.pose = poseset

            rospy.loginfo("Service call : get position success!")
            return response
        except:
            rospy.loginfo("机器人获取位置异常！！！")
            return response

    def set_relocation(self, request):
        rospy.loginfo("Service call : set relocation")
        response = SetRelocationResponse()

        if not self.map_ready:
            rospy.logwarn("map tf can not find")
            response.result = 1
            return response

        self.initial_pose = request.point
        self.relocation_set(0)
        return response
    # 计算自由路线时候的长度
    def calculate_trajectory_length(self, pose_stamped_array):  
        total_length = 0.0
        # 确保数组有足够的点进行计算  
        if len(pose_stamped_array) < 2:  
            return total_length  # 如果没有两个点，长度为0  

        for i in range(len(pose_stamped_array) - 1):
            # 获取当前点和下一个点的坐标
            p1 = pose_stamped_array[i].pose.position
            p2 = pose_stamped_array[i + 1].pose.position
            # 计算相邻两个点之间的距离  
            distance = sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2)
            # 累加到总长度  
            total_length += distance  
    
        return total_length  

    # 计算 两个点的路径 传入 json 的 point 对象
    # 返回 PoseStamped[] 路线 和 路线长度
    def getMovebasePlan(self, startp, goalp):
        if startp == None :
            poseset = self.get_pos()
            startp = MapsBeanPoint()
            startp.setPositionX(poseset.x)
            startp.setPositionY(poseset.y)
            startp.setPositionYaw(poseset.z)
            if self.use_kua:
                startp.setPositionX(self.kua_p_x)
                startp.setPositionY(self.kua_p_y)
                self.use_kua = False

        start = PoseStamped()
        start.header.frame_id = 'map'
        start.header.stamp = rospy.Time.now()
        start.pose.position.x = startp.getPositionX()
        start.pose.position.y = startp.getPositionY()
        q = transformations.quaternion_from_euler(0.0, 0.0, startp.getPositionYaw()/180.0*pi)
        start.pose.orientation.x = q[0]
        start.pose.orientation.y = q[1]
        start.pose.orientation.z = q[2]
        start.pose.orientation.w = q[3]


        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = goalp.getPositionX()
        goal.pose.position.y = goalp.getPositionY()
        q = transformations.quaternion_from_euler(0.0, 0.0, goalp.getPositionYaw()/180.0*pi)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        rospy.loginfo(" call movebase start(%f,%f) , goal(%f,%f) ",start.pose.position.x, start.pose.position.y,goal.pose.position.x, goal.pose.position.y)

        response = self.getPlanMoveBse(start, goal)
        return response.path , self.calculate_trajectory_length(response.path)
    

    def calculate_orientation(self, p, q, r):  
        """判断点 p, q, r 的方向"""  

        val = (q.y - p.y) * (r['x'] - q.x) - (q.x - p.x) * (r['y'] - q.y)  
        if val == 0:  
            return 0  # 直线共线  
        elif val > 0:  
            return 1  # 顺时针  
        else:  
            return 2  # 逆时针  
    def calculate_orientation2(self, p, q, r):  
        """判断点 p, q, r 的方向"""  
        val = (q['y'] - p['y']) * (r.x - q['x']) - (q['x'] - p['x']) * (r.y - q['y'])  
        
        if val == 0:  
            return 0  # 直线共线  
        elif val > 0:  
            return 1  # 顺时针  
        else:  
            return 2  # 逆时针  

    def are_segments_intersecting(self, p1, p2, v1, v2):  
        """判断线段 p1p2 和 v1v2 是否相交"""  
        o1 = self.calculate_orientation(p1, p2, v1)  
        o2 = self.calculate_orientation(p1, p2, v2)  
        o3 = self.calculate_orientation2(v1, v2, p1)  
        o4 = self.calculate_orientation2(v1, v2, p2)  

        return o1 != o2 and o3 != o4  # 交点存在  

    def find_intersection_points(self, path, polygon):  
        """检查路径与多边形相交，并返回所有交点"""  
        intersections = []  
        
        for i in range(len(path) - 1):  
            start_point = path[i].pose.position  
            end_point = path[i + 1].pose.position              
            
            for j in range(len(polygon["points"])):  
                vertex1 = polygon["points"][j]  
                vertex2 = polygon["points"][(j + 1) % len(polygon["points"])]  

                if self.are_segments_intersecting(start_point, end_point, vertex1, vertex2):  
                    intersections.append((start_point, end_point))  # 记录相交的线段 和区域 
                    break  # 找到交点后退出内循环  
        return intersections  

    def get_pos(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf Error")
            return None
        euler = transformations.euler_from_quaternion(rot)
        currentpose = Point32()
        currentpose.x = trans[0]
        currentpose.y = trans[1]
        currentpose.z = euler[2] / pi * 180
        return currentpose

    def check_navigation_isOK(self):
        if not self.odom_ready:
            return

        if not self.scan_ready:
            return

        if not self.map_ready:
            return

        if not self.movebase_ready:
            return

        if not self.slam_ready:
            return

        if self.init_status == 1:
            return

        # 在充电中重启，直接初始化
        rospy.loginfo("上报充电状态：%d", self.AndroidAlarm.androidmsg_chargestatus)
        self.androidChargeStatus_pub.publish(self.AndroidAlarm.androidmsg_chargestatus)

        rospy.loginfo("上报超声波状态：%d", self.AndroidAlarm.androidmsg_sonarstatus)
        self.androidSonarStatus_pub.publish(self.AndroidAlarm.androidmsg_sonarstatus)
        rospy.loginfo("上报左电机状态：%d", self.AndroidAlarm.androidmsg_leftmotorstatus)
        self.androidLeftmotorStatus_pub.publish(self.AndroidAlarm.androidmsg_leftmotorstatus)
        rospy.loginfo("上报右电机状态：%d", self.AndroidAlarm.androidmsg_rightmotorstatus)
        self.androidRightmotorStatus_pub.publish(self.AndroidAlarm.androidmsg_rightmotorstatus)
        rospy.loginfo("上报雷达状态：%d", self.AndroidAlarm.androidmsg_lidarstatus)
        self.androidLidarStatus_pub.publish(self.AndroidAlarm.androidmsg_lidarstatus)
        rospy.loginfo("上报IMU状态：%d", self.AndroidAlarm.androidmsg_imustatus)
        self.androidImuStatus_pub.publish(self.AndroidAlarm.androidmsg_imustatus)
        rospy.loginfo("上报机器人位置状态：%d", self.AndroidAlarm.androidmsg_locationstatus)
        self.androidLocationStatus_pub.publish(self.AndroidAlarm.androidmsg_locationstatus)
        rospy.loginfo("上报里程计状态：%d", self.AndroidAlarm.androidmsg_odomstatus)
        self.androidOdomStatus_pub.publish(self.AndroidAlarm.androidmsg_odomstatus)
        rospy.loginfo("上报地图状态：%d", self.AndroidAlarm.androidmsg_mapstatus)
        self.androidMapStatus_pub.publish(self.AndroidAlarm.androidmsg_mapstatus)
        rospy.loginfo("上报虚拟墙状态：%d", self.AndroidAlarm.androidmsg_virtualmapstatus)
        self.androidVirtualmapStatus_pub.publish(self.AndroidAlarm.androidmsg_virtualmapstatus)

        self.init_status = 1
        #self.AndroidmsgReportPub.publish(12)
        self.androidInitStatus_pub.publish(12)
        self.AndroidAlarm.androidmsg_initstatus = 12

        if (self.map_status == 1):
            rospy.loginfo("处于新建地图模式, 开始记录rosbag ...")
            BAG_PATH = bag_param
            if not os.path.exists(BAG_PATH):
                os.mkdir(BAG_PATH)

            os.system("rm -rf "+BAG_PATH+"/*.*")

            recordbag = RecordBag()
            recordbag.type = 1
            local_time = self.getId()
            recordbag.bag_name = str(local_time)
            recordbag.topic_list.append("/imu/data")
            recordbag.topic_list.append("/tf")
            recordbag.topic_list.append("/tf_static")
            recordbag.topic_list.append("/scan")
            recordbag.topic_list.append("/raw_odom")
            #self.sendToRecordbag_pub.publish(recordbag)

        if self.list_driverstatus_srv is None:
            self.list_driverstatus_srv = rospy.Service('list_driver_status', ListDriverStatus, self.list_driver_status)
        rospy.loginfo("设置机器人速度值为：%d", self.dwaspeed)
        self.sendSpeed_pub.publish(self.dwaspeed)
        rospy.loginfo("机器人初始化成功，导航已就绪。")

        # re-connect wifi network
        self.connectwifi()

    #################
    ##   转圈代码  ###
    #################

    def quat_to_angle(self, quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]

    def normalize_angle(self, angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res

    def get_odom_angle(self):
        # 获取odom和基本框架之间的当前变换
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logerr("TF Angular Exception")
            return

        # 将旋转从四元数转换为欧拉角
        return self.quat_to_angle(Quaternion(*rot))

    def startAngular(self, ispub, dushu):
        # How fast will we check the odometry values?
        # 单位HZ
        rate = rospy.get_param(mapserver_param + "angular_rate", 20)
        # 小机器人0.35 大机器人0.2   估：小机器人0.3对应43.1
        speed = rospy.get_param(mapserver_param + "angular_speed", 0.4)
        # 如果是开机初始化,读取驱动参数转圈,如果是标定参数(0)则读取标定参数
        if ispub == 0:
            odom_angular_scale_correction = rospy.get_param(mapserver_param + "angular_scale", 1.0)
        else:
            odom_angular_scale_correction = 1.0
        rospy.logwarn("ispub = %d  odom_angular_scale_correction : = %f", ispub, odom_angular_scale_correction)

        # 允许误差，单位角度
        angular_tolerance = rospy.get_param(mapserver_param + "angular_tolerance", 1.0)

        r = rospy.Rate(rate)
        # The test angle is 360 degrees
        test_angle = radians(dushu)
        tolerance = radians(angular_tolerance)
        # 小机器人57.47 大机器人114

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # 从 tf 获取当前旋转角度
        odom_angle = self.get_odom_angle()

        last_angle = odom_angle
        turn_angle = 0
        error = test_angle - turn_angle
        # Alternate directions between tests
        move_cmd = Twist()

        response_timeout = 0
        while self.testodom_switch:
            # Rotate the robot to reduce the error
            move_cmd.angular.z = copysign(speed, error)
            self.cmd_vel.publish(move_cmd)
            r.sleep()

            # Get the current rotation angle from tf
            odom_angle = self.get_odom_angle()
            rospy.loginfo("odom_angle: %f, z=%f, turn_angle: %f, test_angle: %f", odom_angle, move_cmd.angular.z, turn_angle, test_angle)
            # Compute how far we have gone since the last measurement
            # check odom responsePlease check odom or driver
            if response_timeout > 10:
                rospy.logerr("robot is not move, please check odom or driver, rotate is abort!")
                self.Errormsg_pub.publish("robot is not move, please check odom or driver, rotate is abort!")
                return
            if abs(odom_angle - last_angle) <= 0.001:
                response_timeout = response_timeout + 1
            else:
                response_timeout = 0

            try:
                delta_angle = odom_angular_scale_correction * self.normalize_angle(odom_angle - last_angle)
            except:
                rospy.logerr("map manager startAngular caused  exception!")
                break
            # Add to our total angle so far
            turn_angle += delta_angle
            # Compute the new error
            error = test_angle - turn_angle
            last_angle = odom_angle
            self.testodom_pub.publish(round(turn_angle/6.28*360.0, 2))
            if abs(error) < tolerance:
                self.testodom_switch = False

        self.testodom_switch = False
        # Stop the robot
        self.cmd_vel.publish(Twist())

    def startAutoRelocation(self):
        r = rospy.Rate(50)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # Alternate directions between tests
        move_cmd = Twist()

        response_timeout = 0
        while self.testodom_switch:

            self.cmd_vel.publish(move_cmd)
            if response_timeout < 1000:
                move_cmd.angular.z = 0.4
            else:
                move_cmd.angular.z = -0.4

            if response_timeout > 2000:
                self.testodom_switch = False
            if self.location_missing == 3 or self.location_missing == 13:
                self.testodom_switch = False

            response_timeout = response_timeout +1
            r.sleep()

        self.testodom_switch = False
        # Stop the robot
        self.cmd_vel.publish(Twist())

    # lineral
    def get_position_linear(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logerr("TF Linear Exception")
            return

        return gemoetryPoint(*trans)

    def startLinear(self, value):
        test_distance = value  # meters
        # How fast will we check the odometry values?
        rate = rospy.get_param(mapserver_param + 'linear_rate', 20)

        # Set the distance to travel
        speed = rospy.get_param(mapserver_param + 'linear_speed', 0.2)  # meters per second
        tolerance = rospy.get_param(mapserver_param + 'linear_tolerance', 0.01)  # meters
        odom_linear_scale_correction = rospy.get_param(mapserver_param + 'linear_scale', 1.0)
        r = rospy.Rate(rate)

        rospy.loginfo("Bring up rqt_reconfigure to control the test.")

        position = gemoetryPoint()

        # Get the starting position from the tf transform between the odom and base frames
        position = self.get_position_linear()

        x_start = position.x
        y_start = position.y

        move_cmd = Twist()

        while self.testodom_switch:
            # Get the current position from the tf transform between the odom and base frames
            position = self.get_position_linear()
            # 计算与目标点的欧几里德距离
            distance = sqrt(pow((position.x - x_start), 2) +
                            pow((position.y - y_start), 2))

            # Correct the estimated distance by the correction factor
            distance *= odom_linear_scale_correction

            # How close are we?
            if (test_distance > 0):
                error = distance - test_distance
            else:
                error = (distance + test_distance)*(-1)
            rospy.loginfo("distance = %s, error = %s", distance, error)
            # Are we close enough?
            if abs(error) < tolerance:
                self.testodom_switch = False
            elif abs(error) > abs(test_distance * 1.2):
                self.testodom_switch = False
            else:
                # If not, move in the appropriate direction
                move_cmd.linear.x = copysign(speed, -1 * error)
            self.cmd_vel.publish(move_cmd)
            self.testodom_pub.publish(round(distance, 2))
            r.sleep()

        # Stop the robot
        self.cmd_vel.publish(Twist())


if __name__ == '__main__':
    reload(sys)
    sys.setdefaultencoding('utf8')

    rospy.init_node("mapserver_manager")
    spirit_dir = "/home/spirit/.ros/"
    move_base_param = "/move_base/"
    global_cost_param = "/move_base/global_costmap/"
    navfn_ros_param = "/move_base/NavfnROS/"
    local_cost_param = "/move_base/local_costmap/"
    dwa_planner_param = "/move_base/DWAPlannerROS/"
    robot_driver_param = "/robot_driver/"
    ydlidar_node_param = "/ydlidar_node/"
    sensor_imu_param = "/sensor_imu/"
    mapserver_param = "/spirit_server_manager/"
    log_param = spirit_dir + "maps/log"
    bag_param = spirit_dir + "maps/bag"

    move_base_param_url = spirit_dir+"params/move_base_params.yaml"
    global_cost_param_url = spirit_dir+"params/global_costmap_params.yaml"
    navfn_ros_param_url = spirit_dir+"params/global_planner_params.yaml"
    local_cost_param_url = spirit_dir+"params/local_costmap_params.yaml"
    dwa_planner_param_url = spirit_dir+"params/dwa_local_planner_params.yaml"
    robot_driver_param_url = spirit_dir+"driver_param/driver_config.yaml"
    robot_cartopose_param_url = spirit_dir+"carto/carto_config.yaml"

    mapserver_param_url = spirit_dir+"params/map_server_params.yaml"
    wifi_param_url = spirit_dir+"wifi/wifilist.yaml"

    spirit_map_url = spirit_dir+"maps/spiritMaps.json"
    version_url = spirit_dir+".spirit.json"
    CANCLEPOINTNAME = "c_j_x_line_point"

    mymapmanager = MapManager()
    rospy.spin()

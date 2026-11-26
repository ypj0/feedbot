# 底盘导航、运动、激光与回充完整链路审计

最新整合交付、覆盖范围与验证结果见 [RECOVERY_DELIVERY.md](RECOVERY_DELIVERY.md)。下文保留分阶段证据；最新完成程度以整合报告为准。

日期：2026-11-07。范围：车机现有文件、运行参数、ROS 图、离线 ELF/MCU 分析，以及一次只读激光消息采样。本轮未发布运动命令、改参数、重启服务或升级固件。

## 结论与证据边界

已取得车机管理程序和激光转发程序的可读 Python 源文件，以及规划、控制、回充、雷达和驱动的可分析二进制。这里的“源文件”指当前车机部署版本；没有与厂家原始发行版逐字节核验，不能将其中所有改动都归因于厂家。

当前实际导航是 ROS Melodic 下的 GlobalPlanner + 定制 DWA，不是根据目录中存在的 TEB 配置推断算法。libmove_base.so 内含充电控制与激光充电桩识别；不是全部使用未经修改的上游 navigation 包。

系统包含多层任务、模式、速度和近障判断。只替换全局规划，不能同时消除底层的限速、停止、自动回充或激光启停。需要统一状态与命令所有权；现有证据不足以把历次停车和卡顿归结为一个已证实的根因。

## 实际链路

```text
网页 / APP
  └─ ROS 服务与任务管理（mapserver_manager.py，另有 spirit_api_manager）
      ├─ 地图、初始位置、任务队列、低电回充、激光启停
      ├─ move_base
      │   ├─ global_planner/GlobalPlanner
      │   ├─ dwa_local_planner/DWAPlannerROS（定制）
      │   └─ ChargeController + DockPerception（定制回充）
      └─ 速度/模式/内部动作
          └─ robot_driver + librobot_driver_lib.so
              └─ 串口协议 → MCU 状态机/目标处理 → 电机目标帧

LT-R2 → /scan → Cartographer / 驱动近障判断 / 管理程序
             └─ republish_scan → /stable_scan → costmap
/wheel_odom + /imu/data → robot_localization EKF → /raw_odom → Cartographer
Cartographer / 原有 TF 发布者 → 地图定位与导航坐标变换
```

这是链路概览，不代表所有节点只通过图中一条接口通信。证据：`analysis/navigation_stack_inventory.json` 与 `analysis/chassis_stack/inventory.json` 的进程、ROS 图和参数快照。

## 各层能恢复什么

| 层 | 已取得内容 | 当前可确认 | 尚缺内容 |
|---|---|---|---|
| 管理 | mapserver_manager.py，可读部署源文件 | 任务/停止/回充/雷达启停/初始点逻辑 | 与厂家发行版的差异、所有并发路径 |
| 全局规划 | libglobal_planner.so、参数 | 当前插件为 GlobalPlanner | 上游版本和定制差异 |
| 局部控制 | libdwa_local_planner.so、libbase_local_planner.so | DWA 及额外速度相关接口 | 完整控制公式、分支、运行调用时序 |
| 回充 | libmove_base.so、管理源文件 | ChargeController、DockPerception、激光聚类与拟合函数 | 全部状态迁移、匹配阈值、终端速度律 |
| 定位建图 | Cartographer 配置/运行信息、EKF 参数 | 激光、里程计、IMU 接入和两条 TF 分支 | Cartographer 定制差异、各 TF 发布者精确责任 |
| 雷达 | LTME ELF、参数、转发源码、实测消息元数据 | 地址/型号/视野、转发行为 | 硬件协议全部细节、设备内部滤波实际设置 |
| Linux 驱动 | 安装包与 ELF | 部分控制模式、近障检测、协议边界 | 完整命令仲裁、全部整形与内部动作 |
| MCU | sub.hex、已恢复 C 模型 | 部分收包、调度、超时与电机帧链路 | 整机固件对应性、时基、完整硬件行为 |

二进制哈希见 `analysis/chassis_stack/binaries/manifest.json` 和 `analysis/reverse_artifacts/manifest.json`。当前有 5 个导航/雷达库或程序副本，可继续离线分析。

## 规划限制与狭窄通道

运行参数已核实：

- `/move_base/check_width=0.8`、`check_length=0.3`、`task_start_delay_time=1`。参数存在不等于已经证明其每个使用分支；1 秒参数也不能直接解释每次运动转换停顿。
- 车身 footprint 为 0.65×0.55m；全局/局部 footprint_padding 为 0.01m。
- inflation_radius 为 0.5m，这是代价扩散范围，不等于强制保持 0.5m 空隙。
- 全局分辨率 0.05m，局部分辨率 0.02m，两者 update_frequency 配置为 2Hz。代价地图更新频率不是速度控制频率。
- 本次所查命名空间没有 controller_frequency/planner_frequency 参数，不能用缺失参数断言运行频率。
- DWA 当前参数含 max_vel_x/max_vel_trans=0.55、max_vel_theta=1.0、min_vel_theta=0.4、sim_time=1.5、vx_samples=10、vtheta_samples=20，以及额外 H/M/L、PID、turn_len、stop_rotate 等字段。它们不是实车能力的测量结果。

重构应将硬碰撞可行性与偏好代价分开：车身及运动过程扫掠范围不能碰撞；靠墙、绕路、旋转次数属于可优化代价，不能隐式变成额外 15cm 的硬门槛。未知区域处理、观测新鲜度和制动距离另行建模，不通过删除所有检查解决通行问题。

几何上，0.55m 宽车可以沿 0.8m 直廊通行，但该矩形绕中心完整旋转的包络直径约 sqrt(0.65²+0.55²)=0.851m。因此“支持原地旋转”不等于在任意 80cm 通道位置都能原地掉头。规划必须检查旋转中的车身姿态以及相邻路径点之间的扫掠，而非仅检查路径点中心。

## 驱动层独立近障判断

`robot_driver::LaserScan_CB`（0x3cd10 起，证据 `analysis/chassis_stack/driver_scan_callback.txt`）遍历收到的 ranges：若某有效测距严格落在 range_min/range_max 内且小于成员阈值，则设置近障标志。运行参数 `/robot_driver/scan_obstacle_distance=0.3`。

这是整个收到的扫描范围内的距离判断，不是矩形 footprint 或当前轨迹的扫掠碰撞检查。同一函数还单独取正前方射线，不能把两者混为一个判断。该标志在 Androidcmd_CB/navi_cmd_CB 的条件分支使用，受模式/安全配置控制；不是“任何时候有 30cm 内障碍必停”。

它说明即使规划层判定路线可行，下游仍可能采用另一套近障语义。尚未以事件对齐证明它造成某次历史停车。迁移时需恢复标志的全部消费分支，再由单一制动/碰撞策略取代重复判定，不能直接关闭全部原厂保护。

## 连续运动与原厂顺滑性的证据

DWA 每个控制周期评估短时速度轨迹，能够同时输出线速度和角速度；这种局部控制形式允许连续弧线，不需要在每个离散全局路径点停下。全局路径离散与车辆必须离散运动不是一回事。

已识别 DWA 的 setLocalSpeed、setSpeedStop、getFakeAngularspeed、speeddwa_CB 等额外符号，但函数名不能当作完整语义证明。进一步跟踪发现：

- DWA `adjust_basespeed` 仅跳转到 `SimpleTrajectoryGenerator::setBaseSpeed`。
- 后者（libbase_local_planner.so 0x409d0）仅保存三个 double 和一个 bool 到对象成员，然后返回；它自身没有进行速度平滑。
- 真正消费这些成员的轨迹生成、速度选择，以及驱动的二次整形还需完整追踪。证据：`dwa_adjust_speed_excerpt.txt`、`base_speed_excerpt.txt`。

因此不能把原厂顺滑归功于某个名字叫“调整速度”的函数，也不能仅凭 C++/Python 语言判断卡顿根因。已有隔离实验表明驱动存在命令下游延迟及串口应答阻塞边界，见 `navigation_runtime/DRIVER_BOUNDARY.md`。这些是隔离环境结果，不是实车刹车时长。

新架构应对整条路线建立速度/角速度连续性和加减速约束，以滚动窗口闭环跟踪；只有不满足几何/动力学连续条件时才减速到零。需要原地旋转时，保持连续角速度控制直到完成，而不是反复发小角度动作。路线目标应同时考虑行程时间、旋转/倒车切换、曲率变化与碰撞，而非只追求最短几何长度。

## 激光处理与充电状态

运行设备：LT-R2，192.168.10.160，frame=scan，range 0.05–40m。一次实际 `/scan` 消息采样：

- angle_min=-1.65、angle_max=1.65rad，约 189°；1075 个测距。
- angle_increment≈0.003068rad。
- scan_time≈0.066398s，对应设备消息声明的约 15.06Hz；这不是多帧实测到达频率。
- time_increment≈32.42μs。不能只以收到消息时刻把整帧射线视为同一时刻。

`republish_scan.py` 只 deepcopy 后把 frame_id 改为 stable_scan；没有滤波、变换坐标数值或去畸变，保留原时间戳。相关静态安装变换含 x=0.27m、z=0.2m，但 scan 与 stable_scan 位于不同 TF 分支。必须验证两条完整链在同一采样时刻是否一致，不能仅凭改名断言雷达数据已变换或已错位。

观察到 TF：map→odom→base_footprint→scan，以及 map→odom_filtered→scan_link→stable_scan。本次页面快照不含每条变换的数值，不能证明两条链等价。也不能把 map 坐标跳变直接当作底盘瞬间位移。

管理源码位置：

- 3637 附近 scan_CB 维护雷达状态/就绪信息，不是车身碰撞算法。
- 3723 附近 navicmd_CB 在相应模式下收到导航速度消息就可能启用雷达，未要求速度非零。
- 5863 附近在 ChargeStatus 1–4 且雷达开启时累计回调计数，大于 3000 后发布 laser_switch=0。注释称 30 秒，但实现是计数，不是独立墙钟计时；需实测反馈频率才能换算时间。

这提供了“充电时纸箱变化不再显示”的机制依据，但不能证明所有充电扫描缺失都由此造成。ROS 图还显示 driver 也参与 laser_switch 发布/订阅，因此激光电源状态同样需要统一所有权。

## 定位和解锁推车

EKF 运行参数：frequency=50、two_d_mode=true、world_frame=odom，输入 wheel_odom 的 vx/vy 和 imu/data 的 yaw 角速度；未启用 wheel_odom 绝对位置和 IMU 绝对航向融合。输出重映射为 /raw_odom，Cartographer 结合激光继续定位。

Cartographer 当前运行 spirit_localization.lua，使用保存的 pbstream；tracking_frame=imu_link、published_frame=odom、provide_odom_frame=false、use_odometry=true、use_imu_data=true。配置 pose_publish_period_sec=0.005 不是定位独立观测每秒 200 次的证明。

解锁手推后，地图定位、轮速/IMU 积分和旧控制参考可能暂时不一致。新的实现应保留连续局部 odom 与地图修正的不同语义，用时间戳对齐传感器，并在地图/定位版本变化时重新变换路线和控制参考。对确实失效的状态自动重新定位/重规划；不能无条件重用旧误差，也不能未经判断每次扫描匹配微调都重启运动。

当前证据不能断言问题是底盘硬件，也不能证明前期自研融合全部正确。必须对齐原始轮速、IMU、map→odom、跟踪参考及最终输出，才能判断误差来自哪层。

## 回充、初始点与自主动作

mapserver_manager.py 的 4658–4709 附近：回充目标可以组合“充电桩”的朝向与“初始点”的位置；相距大于 1.5m 时改用充电桩位置；还将类型编码到 Pose.position.z。`/start_lidarcharge` 实际订阅方是 move_base。

libmove_base.so 中已确认：

- ChargeController::startGoCharing、approach、execute、stateCallback、stop。
- DockPerception::extract、fit、callback、getRobotPose。
- laser_processor::ScanProcessor::splitConnected/removeLessThan。

这些支持“导航接近 + 激光识别/拟合 + 近端回充控制”的功能分工。具体几何模板、拟合阈值和完整速度公式尚未恢复。目录中的 lidarcharge.launch/fetch_auto_dock 不足以证明该独立节点正在运行。

低电自动回充：管理源码约 1027 行按 Power<=powerThreashold、充电状态和队列标志添加回充任务。当前阈值为 20，恢复相关阈值为 90。这证实存在未经网页点击也能生成回充任务的路径；没有当时日志，不能断言它就是历史“自己回充”的唯一原因。

停止链路也不是仅发零速度：task_stop 清理任务并发布 go_target_op=3；NaviStop_CB(data=5) 还切换 motionmode=2。新实现应显式管理任务取消、模式退出、回充退出与最终零输出，统一记录指令来源。

## 建议采用的完整架构

1. RK3588 上运行 ROS2/Nav2 C++ 规划与控制；网页只负责目标、任务和可视化。周期控制不依赖网页、JSON/rosbridge 往返。
2. 底盘侧原生适配器负责时间戳、传感器反馈、命令租约与唯一输出；初期复用已验证硬件驱动。车机 Ubuntu/ROS1 与 RK 的 ROS2 通过明确数据契约连接，不能直接加载 x86 ROS1 库到 ARM ROS2。
3. 采用支持差速运动原语的 Smac State Lattice 候选：原语包含旋转/直行/弧线，并以连续扫掠验证。是否可直接用具体版本的内置原语需核实，不能默认任意配置天然支持原地旋转。
4. 局部跟踪以现有已验证 C++/RPP 为基线，与 Nav2 MPPI 在同一录制数据和约束下比较。MPPI 是候选而非已证实更快/更顺滑；按 RK 实测 CPU、延迟尾部、跟踪误差和停顿次数决定。
5. 规划后台异步执行；控制读取不可变轨迹/地图快照；传感器、控制、日志分离。不能因重规划而暂停有效控制，也不能在轨迹或观测失效时盲目沿旧路线行驶。
6. 单一仲裁器覆盖手动、导航、回充、脱桩、取消、失联和低电请求，最终硬件出口只有一个所有者。回充请求进入任务层，不能绕过授权状态自行接管电机。
7. 激光保留原时间信息、外参和覆盖范围；评估转弯时去畸变。位置发布与速度估计分开，消除地图修正对速度闭环的直接冲击。
8. 将碰撞、制动和数据时效作为清晰独立的约束；不设置任意固定 15cm 通行门槛。速度相关可停止距离按实测延迟/制动能力计算。

目标控制周期可从已有 50Hz 仲裁框架起步，但不是已获得的性能保证。0.5m/s 在一帧约 66ms 期间行进约 3.3cm，在 2Hz 更新间隔内行进约 25cm；需要最新扫描的控制碰撞检查，不能只依赖旧代价地图。制动距离应含测量延迟、执行延迟和 v²/(2a)，其中 a 必须来自实测。

## 后续恢复优先级与验收

优先恢复两个完整边界，而非追求所有反编译文件数量：

1. **命令到车轮**：DWA 最终输出、驱动模式/近障/整形、MCU 目标及超时覆盖。验证实际命令所有权和断流行为。
2. **扫描到回充**：DockPerception 的输入/输出与拟合判定、ChargeController 状态机、接触/充电/脱桩反馈。先保留已验证原厂行为，再形成可替代实现。

MCU 已恢复部分纯 C 语义模型，873 项核心测试和 405 项电机输出对照通过；范围和外部调用替身见 README，不能称为可刷写完整固件。当前 sub.hex 与设备版本号相符但尚未逐字节读取 Flash 对应验证。

离线验收应覆盖任意路线类型：直线、连续小曲率弧线、原地转向、窄廊、掉头、解锁位移、定位修正、动态障碍、暂停续行及回充控制权切换。报告规划时间 p50/p95/p99/max、控制 deadline miss、命令间隔、非必要零速度持续时间、横向误差及扫掠碰撞结果；不能以单条成功路线或平均耗时验收。

本轮交付是完整证据链审计与新增二进制/激光元数据，不声称已完成上述架构部署或达到 0.5m/s 实车验收。

# 首版 IMU / EKF 集成

## 范围与证据

Jazzy；轮式 `/wheel/odom` 只选 vx、vyaw，`/imu` 只选 vyaw；
`two_d_mode=true`，EKF 独占 `odom → base_link`。
基础运动已有用户现场确认，精度未量化，不调整轮半径 0.085 m / 轮距 0.455 m。
建图、定位和导航不属于本轮验收。

IMU 直接复用 `navigation_runtime/replacement/imu/wit_decoder.hpp`，
只把新收到的陀螺仪包转成 ROS2 消息，不因加速度/姿态包到达而重发旧角速度。
陀螺仪独立发布不依赖三通道同时 ready；未提供的姿态/加速度按消息约定标记协方差 -1。
复用既有 `SerialPort` 的独占、115200 8N1 和释放，不调用 MCU 握手、不写 IMU。
每 2 ms 读取，队列超过 1024 字节或 200 ms 无新陀螺仪数据时明确失败退出，
不补数据或自动重连。时间戳为有效陀螺仪包接收时间，**绝对时间对齐未验证**。
`/diagnostics` 提供帧率、有效包数、最大陀螺仪接收间隔及 `decoder_rejected_bytes`。
拒绝字节包含冻结解码器不支持的包类型及同步/校验拒绝，不能解释为坏帧数或纯校验失败数。

## 协方差来源与假设

`config/wheel_covariance.yaml` 显式设置 vx 方差 0.04 (m/s)²、vyaw 方差
0.04 (rad/s)²，分别采用 σ=0.2 m/s、0.2 rad/s 的保守建模初值。
来源仅为用户确认方向、停车和距离转角基本一致；没有残差统计，不能称实测噪声。
其余未融合分量用 1000 表示低置信度；pose 不参与首版 EKF。

IMU 角速度对角方差 0.01 (rad/s)²，σ=0.1 rad/s，是零偏/噪声未量化时的
保守建模初值，不是传感器精度或已测零偏。解码单位为 rad/s；实车转向符号需单独核验。
两类初值均标记 **协方差待地面标定验证**，不能用“非零”宣称已标定。

## TF 与运行

原厂 `analysis/chassis_stack/home/spirit/spiritdeb/launch/robot_driver.launch`
记录 footprint→base_link 为 (0,0,0.5)，`imu/wit.launch` 记录 footprint→imu_link
为单位变换。坐标统一后以 `base_link` 为 URDF 根和 EKF `odom`
子坐标；`base_footprint` 是其地面投影，与 `base_link` 零位姿重合，
因而两者的固定边为 base_link→base_footprint。不反向发布，避免
`base_link` 同时从 `odom` 和 `base_footprint` 获得两个父节点。

统一后 base_link→imu_link 为零平移、零旋转。左右轮中心保持
y=±0.2275 m，z=0.085 m 由现有名义轮半径与地面原点推导。
这些是基于原厂值的起步几何，尚未实车测量安装外参；本轮只融合
角速度，平移不参与该量的旋转转换。旧静止 EKF/TF 证据采集于
此坐标变更前，不能作为新 TF 树已运行验证的证据。

先验证 IMU 新鲜度及人工转车体的 yaw 符号，再运行：

```sh
ros2 launch feedbot_chassis_ros2 chassis.launch.py covariance_file:=/integration/config/wheel_covariance.yaml
ros2 run feedbot_imu_ros2 imu_node
ros2 launch /work/agv_workspace/integration/ekf/ekf.launch.py
```

上述是分别启动示例，禁止重复启动已有串口所有者。运行时核对 `open_loop=false`、
`position_feedback=true`、`enable_odom_tf=false` 和实际消息协方差。
通过 view_frames、`ros2 topic info /tf -v` 和 C++ `tf_edge_probe` 的 RMW publisher_gid
对齐具体边及端点，不能仅凭 default_authority 或发布者数量判定独占。
当前镜像 Python MessageInfo 只有时间戳和序号，不提供 publisher_gid；
`observe.py` 负责消息统计，`tf_edge_probe` 负责实际 TF 消息的端点来源。
当前阶段不生成 map→odom；雷达 TF 随雷达集成补齐，不用虚假单位变换补树。

### EKF 执行线程调度

`build.sh` 将 `feedbot_ekf` 安装至 `/work/install/lib/feedbot_ekf/`，launch 使用此入口。
入口直接链接已安装的 robot_localization `rl_lib`，保留原初始化和单线程 spin；
初始化 DDS/TF 线程之后、spin 之前，只将调用线程设为 CFS nice=-10。
容器须提供 `--ulimit nice=30:30`（允许的最低 nice 为 -10），不需要 SYS_NICE；
设置失败明确退出，不回退为普通优先级。不能用启动前 `nice` 替代，否则 DDS 子线程会继承提权。
这针对已捕获的 DDS 队列解锁唤醒后 runnable 调度延迟，不是滤波算法修正或硬实时保证。
ASYNC、50 Hz、滤波参数和失败行为不变；实车持续效果仍需验收。

## 故障解释与验收边界

此前 MCU 运行字段异常：**已观测 1 次，后续静止和运动测试未复现，发生频率未知**。
本轮另记录 1 次反馈超时，前邻 read 周期 258.7 ms；两类故障的关联尚未证明。
见 `agv_workspace/acceptance/runs/20261122_imu_ekf_01/FEEDBACK_TIMEOUT.md`。
read ERROR、控制器停用、串口释放和里程计停止更新的边界不变。
分类计数只记录：无有效反馈超时、同步/校验拒绝字节、运行字段异常；
异常原始帧仍保存在诊断 message 的 frame_hex 中。计数为当前串口会话累计，
诊断 header 为车机 ROS 时间，monotonic 字段为车机单调时间，不与 Mac 直接相减。

若故障再现，EKF 将从现有滤波状态预测，不会把旧轮测量当作新观测重复融合。
前向速度观测中断会降低可观测性；输出异常需与诊断时间戳对照，不能直接归因参数或
自动断定输入中断就是全部原因。未解决故障不能标为已修复。

IMU 静态验收须包含频率、轴向/单位和人工转车体的方向；EKF 须包含 TF 端点独占、
静止漂移观测及转向跟随。静止首次观测采用 60 s、位移≤0.02 m、航向变化≤0.02 rad
的初步工程检查门槛，不代表用户确认的导航精度标准；完整验收仍需人工转向验证。

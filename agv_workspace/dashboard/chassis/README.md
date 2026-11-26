# 人工操作底盘测试网页

入口：`http://127.0.0.1:8088/`，由 Mac 的 ADB 转发到 RK3588 的回环监听口。
RK 托管静态页面及固定车机代理；ROS2 桥、ros2_control 插件、diff_drive_controller
运行在 x86 车机的冻结 Jazzy 容器。没有替换 MCU 协议或增加 cmd_vel 直通串口。

## 操作范围

用户已确认离开充电桩、空地及现场看守，并将本轮改为地面受控测试，不要求架空。
网页提供前进、后退、左右转、停止、记录起点和保存当前读数。
运动必须由操作人点击：线速度 ±0.05 m/s、角速度 ±0.10 rad/s，默认指令脉冲 0.5 秒。
地面标定可选 5 或 10 秒；10 秒指令估算直行 0.50 米或转向约 57°，不是定距/定角停车。
关闭网页或通信断开不会取消已接受的定时脉冲；车机到期归零，现场仍须确认停车。
不排队、不延长当前脉冲、不自动重放。网页“停止”只请求零速；实际停车能力、
制动延迟和距离仍须现场验证，不能替代实体急停。

运动请求使用当前服务会话、递增序号及新鲜状态时间，过期请求明确拒绝。
反馈中断或急停/充电/报警状态出现时终止当前脉冲；反馈恢复不会恢复旧指令。
桥退出后控制器的 `cmd_vel_timeout=0.2` 仍生效，但不声称宿主故障时 MCU 必然停车。
只允许同源网页控制；RK→车机使用启动时生成的私有 token，不提交到源码或证据包。
网页仅监听 RK 回环，车机端仅监听车内地址 `10.7.5.88:8098`，不要映射公网。

## 协方差与验收边界

2026-11-22 首轮四方向短脉冲测试：用户确认方向正确、最终停稳、没有明显回摆；
由于行程过短，未独立测量距离/转角。这是方向和停车的现场证据，不等于地面标定通过。
标定时在地面标记轮轴中点的起始投影和车体朝向，网页点击“记录起点”；选择时长，
再人工点击一个方向。停稳后用卷尺测量同一参考点位移、用独立角度基准测量转角，
点击“保存当前读数”。每次重新记录起点，分别采集前后与左右转的多次结果。
记录实际方向、实测值、测量分辨率和对应网页读数；不可用命令乘时间作为实测真值。

`factory_covariance.yaml` 原样继承本轮读取的原厂 `/wheel_odom` 对角线：
twist `[0.01, 0.01, 1e-6, 1e-6, 1e-6, 1e-6]`，pose 六项均为 1000。
来源为 `../../acceptance/runs/20261122_chassis_web_01/factory_baseline.log`。
这是有来源的原厂临时配置，**不是本次实测标定结果**；尤其原厂航向来源与
候选轮式里程计不同，不能把其 yaw 方差直接视为候选轮式里程计的可信误差模型。
本轮不运行 EKF/Nav2，仍须地面误差记录后重新确定融合协方差。
网页轨迹是轮式里程计，不是地图定位。数据显示、按钮可用均不代表运动验收通过。

## 部署与恢复

- 车机：`/opt/feedbot-chassis-web/releases/20261122-v2/chassis/`。
- 驱动：`/opt/feedbot-chassis/releases/20261122-ros2-v1/`。
- RK：`/opt/feedbot-dashboard/releases/20261122-chassis02/chassis/`。
- 临时服务：车机 `feedbot-chassis-web.service`，RK `feedbot-chassis-dashboard.service`；
  单元文件仅在 `/run/systemd/system/`，`Restart=no`，不设开机自启。
- 容器只映射底盘串口，使用宿主 PID 命名空间与 SYS_PTRACE 执行既有串口持有者审计。
  实机证据显示默认 AppArmor 拒绝宿主 `/proc/*/fd`（EACCES），因此仅此测试容器使用
  `apparmor=unconfined`；不使用 `--privileged`，不修改宿主 AppArmor 服务或冻结审计代码。
- 车机日志：`/var/tmp/feedbot-chassis-calibration-web-20261122-01/trace.jsonl`，记录命令、
  结束原因及约 10 Hz 快照。时间戳未跨主机绝对对齐。

准备运行前必须停原厂 `spirit.service` 并确认真实串口释放；不能两套驱动并行。
测试期间原厂雷达/导航也停止。本轮原厂恢复需显式执行，不在操作人测试途中自动切栈：

```sh
systemctl stop feedbot-chassis-web.service
python3 /opt/feedbot-chassis-web/releases/20261122-v2/chassis/serial_owners.py && systemctl start spirit.service
```

Docker 映射的设备节点与宿主节点 inode 不同，`fuser` 不能单独证明串口释放。
上述工具在宿主按字符设备 major/minor 查持有者，仅无持有者时退出 0。
恢复后核对 `/scan`、
`/wheel_odom`、`/driver_info` 数据新鲜及原厂串口持有者。失败须记录，不宣称恢复完成。
Mac 转发命令：`adb -s 8e4d907d5d9d70a9 forward tcp:8088 tcp:8088`。

## 验证

### 日志与控制隔离

`event_log.py` 由独立线程按入队顺序执行文件write/flush，控制回调只交接已序列化的记录，不等待磁盘。部署网页桥时必须一并包含该文件；建图桥复用同一实现。

队列固定上限128条，不提供额外配置。写入失败或积压达到上限会明确报错，拒绝新的运动请求；停止请求仍先发布零速再报告日志错误。定时回调发现错误后沿原有finally退出并请求零速，不重试、不回放。日志异常请求返回HTTP503，不能将失败后的记录当成完整验收证据。

正常退出排空日志；等待写入线程最多2秒，超时明确失败，且不从主线程并发关闭仍在使用的文件。这里是日志退出边界，不修改底盘反馈或速度指令超时。run41本地和Jazzy隔离验证通过，未部署至实车会话、未做运动验收；不是对历史MCU/IMU故障的修复声明。

```sh
python3 -B -m unittest discover -s agv_workspace/dashboard/chassis/tests -p 'test_*.py' -v
python3 -B agv_workspace/dashboard/chassis/tests/browser_fixture.py /path/to/chrome /tmp/feedbot-browser-test
```

`tests/ros_fixture.py` 在不映射真实设备的容器中复用现有 PTY 模拟器，贯通
HTTP→ROS2→diff_drive_controller→插件→模拟 MCU。不得在实车设备映射容器运行该夹具。
`tests/browser_fixture.py` 使用真实页面脚本、模拟 fetch 和独立 Chrome 配置目录，验证时长选择与停止按钮，不访问实车 API。
`run_car.sh` 随任一主进程结束关闭另一进程；控制器异常退出由 launch 传播为非零退出，
不留下只有网页在线、控制器已崩溃的伪运行状态。
脚本启用 Bash job control，使后台 ROS launch 不继承非交互 shell 的 SIGINT 忽略状态；
退出时仍通过 SIGINT 请求 launch 清理其子进程。夹具必须同时验证串口释放，不能只看动作断言。

# 原生 LT-R2 驱动交付状态

## 当前可以验证的结论

- C++ ROS2 `LifecycleNode`已实现：configure、activate、deactivate、cleanup、shutdown/error。
- SDK仅处理TCP控制；完整扫描数据由独立、无SO_REUSEADDR的UDP接收器处理，
  不通过ROS1、rosbridge或RK3588。生产节点不需要NET_RAW或抓包权限。
- 激活必须等到新完整帧；休眠、断流和撤销控制权都会撤销ready。
- 原厂管理程序的启动/自动重启与新的驱动控制权通过专用所有权模块统一。
  充电产生的`/laser_switch`只作为休眠/唤醒意图送给监督器，不能越过所有权直接启停新驱动。
- 监督器执行停止原厂→验证TCP/UDP释放→发放权限→启动新节点→配置→激活→检查就绪。
  任一步失败，撤销权限、停止新进程、验证端口释放，才恢复原厂。
  新进程无法停止时进入rollback_failed，禁止启动第二个控制者。
- 中断的事务有journal，下一次监督器启动先恢复原厂。导航禁止状态不会因雷达回退自动解除。
- 缺少导航禁止钩子时，预检查直接拒绝，绝不开始切换；示例配置不是可直接启用的生产配置。

## 文件职责

|文件|职责|
|---|---|
|packet.hpp / udp_input.hpp|完整UDP边界、CRC、帧重组、内核接收时间|
|sdk_control.hpp / lifecycle.hpp|厂商SDK控制与操作状态机|
|managed_node.cpp|标准ROS2生命周期、scan发布、ready与健康状态|
|factory_owner.py|原厂管理程序的雷达启动权、重启权和充电意图|
|prepare_factory_manager.py / prepared/|基于已核实原文件生成完整改造版本与可审阅diff|
|supervisor.py|独占交接、睡眠意图、健康监控、故障回退和事务恢复|
|native_container.sh / lifecycle_command.py|容器启动/退出及ROS2生命周期服务调用|
|wait_ports_free.py|拒绝在原控制连接或UDP端口仍占用时启动第二驱动|
|run_acceptance.sh|断网容器内完整验收入口|

静态安装变换和全局定位仍属于独立定位/TF组件，雷达节点不伪造map→odom或base位姿。
默认扫描几何适配已核实LT-R2、360°协议和当前±1.65rad裁剪，非通用雷达驱动。
硬件绝对采样时间仍未标定，当前时间戳为内核接收时间和块时间差的估计。

## 验收覆盖

内部车机Jazzy容器network=none：
1. C++状态机与厂商SDK真实TCP封装/CRC测试通过。
2. 监督器7项测试通过：交接、休眠/唤醒、预检查无副作用、启动阶段失败回退、
   新驱动停止失败不启动原厂、进程中断后事务恢复、事务锁与原厂watchdog隔离。
   测试函数中部分项目合并，具体见test_supervisor.py。
3. 实际ROS2 ChangeState服务＋模拟LDCP控制端＋真实格式录制UDP数据通过：
   无权限拒绝、锁竞争拒绝且不覆盖旧状态、端口竞争拒绝、配置/激活、等待完整新帧、
   休眠/唤醒、断流撤销ready、错误清理、重新配置/激活、撤销权限。
4. 改造后的完整原厂管理程序及所有权模块在车机Python2中仅编译、不导入、不安装。
   原源码SHA256强校验，准备版本与本地逐字节哈希匹配。

模拟控制测试的帧由录制数据产生，不声称已对真实雷达执行启停。
事务单元测试的底层进程操作使用假设备，不能代替真实停止/回退演练。
原厂spirit.service仍active、PID916；原厂管理程序与雷达节点未被本轮替换。

## 实车启用前的明确未完成项

`supervisor.example.json`中的两个site钩子尚无实车实现，故意指向不存在的路径：
- `navigation_inhibit_and_verify`：持续禁止所有底盘运动来源，并确认停止。
- `verify_factory_scan_and_navigation_inhibit`：回退后验证原厂扫描恢复，同时确认运动仍被禁止。

不能用一次`cmd_vel=0`、单次里程计静止或一个手工标志文件替代这些条件。
逆向已证实原厂驱动有内部回充/动作路径，现阶段没有已验证的统一执行联锁。
这不是需要放宽的“限制”，而是实车替换时必须落实的执行权边界。

因此目前交付的是已测试的雷达驱动与切换组件，不是已经可在行驶中无缝接管的生产方案。
不要直接启用示例配置，不要将prepared文件覆盖到原厂后随意重启。
完成统一运动联锁后，才执行一次有备份、带回退的管理程序安装和实车切换验收。

## 源码与复现

本地：navigation_runtime/replacement/lidar；车机：/opt/feedbot-nav2/lidar。
准备版本仅在本地prepared/，未覆盖/opt/ros/melodic/lib/spirit_main/mapserver_manager.py。
固定SDK见stage_sdk.py和原始provenance；不改厂商固件或持久化设置。
执行 `bash /opt/feedbot-nav2/lidar/run_acceptance.sh` 可重新验证隔离软件链路。
证据在analysis/body_navigation_20261107/lidar_native/control及acceptance。

## 后续真实静止交接结果

已在用户按下实体急停的条件下完成真实交接与原厂恢复，详见../ONBOARD.md末节及
analysis/body_navigation_20261107/readiness/。真实设备控制面型号为LT-R2-B，
现有UDP数据目标为广播255.255.255.255:2105；驱动默认值已据此校正，未改雷达配置。
10秒内原生ROS2发布计数增加151帧，期间57次采样全部健康就绪；之后原厂扫描恢复。
维护监测同时要求新鲜的急停状态和零速度反馈，不授权行驶。
上述早期“未实车交接”描述属于历史状态；统一运动联锁和行驶验收仍未完成，
生产supervisor.example.json的site钩子依然不能用静止维护钩子替代。

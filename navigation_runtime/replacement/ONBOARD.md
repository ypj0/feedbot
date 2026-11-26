# 内部车机 Nav2 部署

目标平台：底盘10.7.5.88，Intel i5-5200U、4GB、Ubuntu18.04.6、Linux5.4。
宿主系统保留；独立Docker27.5.1运行amd64 Ubuntu24.04/ROS2 Jazzy。
源码参考固定在upstream/manifest.json；正式使用apt提供的官方Nav2二进制包，安装版本另存证据。

## 路径与隔离

- 引擎程序：`/opt/feedbot-nav2/docker`。
- 引擎套接字：`/run/feedbot-nav2-docker.sock`，无TCP管理端口。
- 引擎数据：`/var/lib/feedbot-nav2-docker`；不会覆盖原厂ROS安装目录。
- 引擎服务：`feedbot-nav2-engine.service`；当前没有启用开机启动。
- 构建上下文、镜像元信息和日志：`/opt/feedbot-nav2`。
- 测试结果：`/opt/feedbot-nav2/results`。

Docker禁用网桥、iptables修改、IP转发修改和端口转发。
测试容器network=none、移除capability、只读根目录、无真实设备挂载，限制1个逻辑CPU配额及1200MB内存。
输入仅合成地图和虚拟里程计；唯一速度订阅者是容器内虚拟小车。
官方planner_server和controller_server独立进程启动，通过标准ROS2 action/topic连接。
ROS_DOMAIN_ID=88；没有rosbridge调用、RK实时计算或向底盘发布运动命令。

## 操作

车机执行`bash /opt/feedbot-nav2/run_onboard_shadow.sh`启动一次有限时间测试，结束后自动删除测试容器。
`bash /opt/feedbot-nav2/stop_onboard_shadow.sh`只停止该测试容器。停止引擎使用：

```
systemctl stop feedbot-nav2-engine.service
```

这些命令不启动或停止原厂spirit.service。恢复原厂应用与系统恢复的边界见README。
当前阶段仅做软件兼容性与仿真集成验证，不是实车接管，也不能把模拟行走距离当实际运动。

## 安装传输

车机无可用外网DNS，临时通过RK转接USB下载代理。ADB reverse在该连接上不可用，
使用adb forward加有限下载隧道；它只用于安装，不是未来控制链的一部分。
下载文件在车机核对SHA256；镜像层由Docker校验digest，deb包由apt签名仓库校验。
宿主不安装ROS2 apt包，不重装系统；所有apt变更发生在容器镜像构建层内。
下载代理环境只放在/run下的引擎临时drop-in；完成后移除并重启独立引擎验证离线运行。

## 已完成的验收（2026-11-07）

- 官方amd64 Jazzy基础镜像、Nav2 1.3.13、nav2_bringup已安装；原厂宿主仍为Ubuntu18.04.6。
- 新镜像：feedbot-nav2-onboard:jazzy，ID见下方本地image证据；独立目录安装，未启用开机自启。
- 限制1个逻辑CPU配额的断网容器：80cm通道规划10.444486ms，FollowPath成功完成，
  模拟终点误差2.85cm、最终线/角速度均0，逐周期独立车身扫掠无碰撞。
  这些是单次合成场景和理想运动模型结果，不是实车精度、p99或0.5m/s认证。
- 50cm通道拒绝；60cm仍START_OCCUPIED，列为未解决项，production_ready仍false。
- 官方C++碰撞API探针成功编译/运行。无膨胀的合成地图上，60cm通道的直接轮廓与
  Smac轮廓检查均通过（y=1.99/2.00/2.01）。观察到栅格中心转换，但它单独未复现
  完整规划的拒绝。应继续核对完整costmap、实际footprint和膨胀代价；不能将坐标偏移
  当作已经证明的根因。探针为强制完整footprint检查有意禁用膨胀早退，与生产配置不同。
- 初次探针因临时目录noexec无法执行；改为只在隔离探针容器使用exec tmpfs后完成。
  保留probe_initial.log，不把该安装环境错误伪装为导航算法失败。
- 镜像构建期间仍保留原厂spirit.service，主PID916未变化。
- 已移除引擎下载代理drop-in并重启独立引擎；断网新容器成功创建Jazzy ROS2节点，
  环境无proxy变量。临时RK传输服务及ADB下载转发均已停止；网页原有转发保留。
- 验证结束无运行中的新容器，未连接底盘真实执行接口、未发送实车运动命令。

证据：analysis/body_navigation_20261107/onboard/：
validation.json、results/shadow_stack.json、probe.log、container_packages.txt、
base_image.json、nav2_image.json及完整构建/控制/规划日志。
下载代理移除后的证明：analysis/body_navigation_20261107/onboard_offline_check.json。
所有从车机导出的文本逐项比对SHA256。

下一步是本地C++ ROS1驱动→ROS2标准消息适配、真实数据只读验证、
统一执行所有权和停止/失联验证；不让网页/RK/rosbridge进入高频控制链。

## 本地真实传感器适配验证（2026-11-07）

已新增 `adapter/`：ROS1 C++ → 车机 Unix socket → ROS2 C++ 标准消息。
宿主订阅原厂 scan、raw_odom、wheel_odom、imu/data、map、tf、tf_static；
完整传递双方共有字段，不使用旧网关的简化扫描/融合位姿，不接控制输出。
原始 Header.seq 没有 ROS2 对应字段，不映射；传输序号独立计数。

35 秒有限采集 session.beI7bH：发送/接收共 16,907 条，各类计数完全一致，
每条接收消息重新编码后与原负载逐字节相同。ROS2 独立观察者确认：
激光约15.16Hz（1075束，1075强度，扫描时间/束间时间均保留），
IMU约200Hz，raw/wheel里程计均约50Hz，地图75×226、分辨率5cm。
激光回调开始到接收端延迟 p50=0.17063ms，p99=4.27332ms，max=8.27882ms。
这些不是驱动采样到控制输出的端到端延迟，也不是高负载保证。
ROS2 观察到扫描原始时间戳年龄中位约70.54ms，包含雷达扫描及原厂处理时间。

首次 scan-time TF 查询失败：启动时历史比扫描时间晚约15.85ms；之后479次
对map、odom、base_footprint的查询均成功。保留这一失败作为就绪联锁需求，
不对扫描改时间戳或拿最新TF代替。地图为静态锁存，旧时间戳本身不代表地图失效。
原厂TF边包含前导斜杠；本轮tf2可查询，但生产化仍需校验统一命名和逐边发布权威。
观察到相机重复发布tf_static，适配器保留数据并汇总静态集合。

原厂 spirit.service 验证前后均 active、MainPID916。测试完成后只读进程及容器退出。
第一次验证因ROS2环境脚本在shell nounset下失败，日志保留在session.qdbEZH；
已将环境加载与脚本严格模式分开，后续运行exit0。
源码、协议与局限见adapter/README.md；证据见
analysis/body_navigation_20261107/onboard_adapter/validation.json及逐项SHA256日志。
此实现仍是有限只读验证，不是常驻生产适配：逐源失效联锁、队列丢包审计、重连恢复、
执行权独占、真实停止验证和60cm完整规划拒绝仍须解决，production_ready=false。

## 原生雷达解析（后续进展）

已确认LT-R2实时扫描走UDP2105，TCP2105为原厂持有的控制连接。
新增lidar/原始数据解析与ROS2回放节点，无ROS1订阅；车机断网Jazzy容器编译运行成功。
被动抓包607个UDP包重组75完整帧，15.15Hz、每帧裁剪1075束，ROS2发布/candidate/scan。
CRC、包长、缺块/重复块及恢复测试通过ASan/UBSan。原厂spirit仍active、PID916。
尚未替换实际硬件驱动：真实数据逐束对照、时间同步、控制面和独占切换仍未完成。
详情lidar/README.md；证据analysis/body_navigation_20261107/lidar_native/。

## 原生雷达同步对照及实时观察（后续完成）

181帧/194,575束全部匹配原厂扫描，强度完全相同，距离最大浮点差4.77e-7m。
原始采集与匹配证据：lidar_native/pair_1794045504（位于上述analysis目录）。
裁剪第487束的时刻偏移解释了约15.6ms stamp差异，绝对时间同步仍未标定。
直接网卡→C++→ROS2实时观察15秒通过：1818包、发布226帧；独立ROS2订阅者
收到210帧，每帧1075束、时间戳无倒退。两进程启动时间不同，不能以此宣称零丢帧。
原厂service前后均active、PID916，退出后无新容器运行。
证据：analysis/body_navigation_20261107/lidar_native/live/validation.json。
新入口只是被动实时观察，雷达启停仍由原厂控制面维持，未完成独立生产驱动接管。

## 雷达控制面生命周期核心

新增lifecycle.hpp、sdk_control.hpp和SDK回环模拟器：启动应答后等待新帧才ready，
休眠/停止首先撤销ready，停止部分失败保持Fault，重连配置不自动激活，epoch隔离旧扫描。
本机ASan/UBSan状态机测试和内部车机断网容器SDK模拟协议验证均通过。
模拟验证使用厂商SDK实际TCP封装/CRC，设备地址硬编码回环；没有真实雷达启停。

实机确认原厂雷达PID1487由管理程序PID1296启动，并占用UDP2105。
在用管理源码哈希与备份一致，scan_status超限会自动重启雷达。
正式交接须同时整合原厂管理程序的启动/重启权和充电休眠请求；单停雷达进程不构成独占。
普通C++生命周期核心尚未接ROS2 LifecycleNode，也没有生产级交接监督程序；
不能把模拟协议通过视为独立驱动替换完成。代码边界和证据见lidar/README.md、
analysis/body_navigation_20261107/lidar_native/control/。

## 完整雷达软件组件的隔离验收

已实现标准ROS2 LifecycleNode、普通UDP独占接收（无抓包权限）、厂商SDK控制、
原厂管理程序所有权扩展及完整准备版本、事务式交接/回退监督器。
在车机断网Jazzy容器跑完run_acceptance.sh，exit0；生命周期/SDK协议、7项监督器测试、
ROS2服务+UDP集成的权限/锁/端口冲突、休眠唤醒、断流、错误恢复与权限撤销均通过。
原厂管理程序准备版本在车机Python2编译通过，并与本地准备版本SHA256一致。
这些是隔离软件验收，不是实车交接；原厂manager SHA256未改变、spirit主PID916仍active。
验收结束无新容器运行。证据：analysis/body_navigation_20261107/lidar_native/acceptance/。

实车切换仍依赖未落实的统一运动联锁（原厂内部回充可绕过cmd_vel）。
监督器示例的site钩子故意未提供假实现，缺少钩子直接在切换前拒绝。
原厂所有权改造文件仅准备、未安装，不会自动替换原厂管理程序。
当前交付范围和部署条件统一见replacement/lidar/DEPLOYMENT.md。

## 实机静止雷达交接验收（2026-11-07 后续）

用户按下实体急停后，连续只读监测原始DriverEntry和wheel_odom，确认急停=1、速度为零。
进行了有独立systemd恢复任务的临时管理程序安装、雷达交接、原厂恢复。
最终一次真实交接通过：57份健康样本全部ready/healthy，10秒内发布计数从3增至154。
原厂扫描恢复到1075束，manager SHA256恢复为原始1a00b171...65884，spirit.service active。
证据：analysis/body_navigation_20261107/readiness/stationary_trial.json、restored.json和collection.json。

真实控制面核实型号是LT-R2-B、OOB目标255.255.255.255:2105，先前LT-R2/单播假设不准确。
驱动使用显式型号/目标参数匹配，不改变设备持久化配置。宿主Python3.6调用接口已适配；
部署保留原可执行权限；原厂systemd停止可能超过25秒，恢复等待与独立截止任务已统一。
当前正式原厂代码已恢复；临时所有权模块已移除，独立恢复定时任务已清理。

此次是实体急停下的真实雷达启停/数据/回退验证，不是运动接管验收。
没有发送行驶指令，没有验证MCU独立失联制动，原生底盘串口执行器及全栈行驶链路仍未完成。
实体急停的反馈证明维护条件成立，不替代真实运动下的制动测试。


## 2026-11-07：真实底盘串口静止交接

`chassis/stationary_probe.cpp`已在内部车机用GCC7构建，并在独立systemd维护任务中
独占`/dev/rikibase`完成115200握手。10秒收发各494帧，最大反馈间隔21.3695ms。
仅发送握手和核对的零速度完整帧；物理急停始终按下，反馈速度为零。
程序不包含arm/速度/回充等接口，不能据此宣称运动执行器完成。

原厂服务已恢复。首次自动核对因只读观察器保留旧ROS master订阅而超时；
恢复流程已将重建观察器作为服务生命周期的一部分，实机重新核对成功，
扫描1075束，驱动/里程计新鲜且静止，原manager SHA256一致，恢复timer已取消。
初始失败日志和后续成功证据一起保存在readiness/chassis_stationary证据目录。
下一阶段仍须完成正式运动所有者、原生里程计/IMU与TF接入及低速物理停车验证。

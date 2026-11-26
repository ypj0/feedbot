# 底盘逆向恢复交付报告

2026-11-07。本次工作为只读提取、离线恢复和验证，未下发运动、修改运行参数、重启服务或写入固件。

## 交付结论

已形成覆盖任务管理→规划/控制→回充感知→激光→Linux驱动→MCU→电机目标的离线分析资料及关键算法恢复模型。**尚未恢复可整体重编译、替换车机或刷写MCU的完整原工程**。编译后二进制不能无损还原被优化删除的源码、注释及所有类型；批量伪代码也不是这个工程的替代品。

最新扩展结果见 [FULL_RECOVERY_STATUS.md](FULL_RECOVERY_STATUS.md)，运动链和雷达详解分别见 [MOTION_CHAIN_RECOVERY.md](MOTION_CHAIN_RECOVERY.md)、[LIDAR_RECOVERY.md](LIDAR_RECOVERY.md)。下面1726函数/17622测试数字保留为前一阶段快照。

本报告把可读部署源码、真实安装头文件、开源参考、自动伪代码和经过原指令验证的模型分别交付。后续可以基于已确定的协议与模型重构，不需要把所有反编译函数强行拼成原厂工程。

## 文件入口与覆盖范围

所有路径相对 /Users/ypj/feedbot。

| 内容 | 位置 | 性质 |
|---|---|---|
| 系统拓扑、实际插件/参数、定位和激光链路 | navigation_runtime/reverse/NAVIGATION_STACK.md | 运行快照与静态证据 |
| 车机部署文件、配置等 | analysis/chassis_stack | 提取资料；具体来源见清单 |
| 53份C++安装头文件 | analysis/chassis_headers/opt/ros/melodic/include | 实际安装源码，含原成员名和消息定义 |
| APP业务Java文件 | analysis/app_reverse_sources | 1146个反编译业务文件，不是APP原工程 |
| 导航、驱动、雷达、MCU伪代码 | analysis/decompiled/README.md | 8个模块、1726个导出函数 |
| 原始二进制及MCU镜像 | analysis/chassis_stack/binaries、analysis/reverse_artifacts | 各自有SHA256清单 |
| 上游参考源码及差异 | analysis/upstream | 固定提交；不冒充厂家同版源码 |
| 可编译核心模型 | navigation_runtime/reverse/recovered | 纯C离线语义模型，无生产硬件适配 |
| 一键差分验证 | navigation_runtime/reverse/verify_all.sh | 编译C并与原二进制指令执行比较 |

自动导出细分：move_base 349、DWA 170、base_local_planner 179、global_planner 63、robot_driver 101、driver库107、LTME雷达444、MCU313。核心模式导出算法类及匿名辅助函数；没有宣称所有运行库、模板实例、未识别函数和间接调用均已恢复。728个文件含Ghidra警告；0个导出失败仅代表工具完成导出。

各目录 readable.tsv 提供解码后的C++名称、原地址、伪代码链接、哈希及警告数；calls.tsv是静态调用关系。ELF加载偏移为0x100000，例如原0x103bc0在导出中为00203bc0；MCU保持0x0800xxxx地址。corpus_manifest.json可核验文件完整性。

## 可编译、经过验证的核心

| 模型 | 原指令对照范围 | 用例数 | 结果 |
|---|---|---:|---|
| charge_core.c 原有核心 | 搜桩、充电反馈字节、接近与最终入桩速度切片 | 3944 | 一致 |
| charge_core.c 整函数平面模型 | approach/startGoCharing几何、控制及状态更新 | 2400 | 容差1e-11内一致，最大差4.44e-16 |
| driver_smoothing.c | smooth_pwm_auto的双轴平滑、时间限幅、打包输出 | 10000 | 状态和输出逐位一致 |
| sub_core.c | MCU周期任务、帧校验、任务倒计时 | 873 | 一致 |
| motor_output.c | 目标更新至下游电机帧构造链 | 405 | 一致 |

合计17622组。整函数测试与切片测试有覆盖重叠，不把用例数当作分支覆盖率。ROS、libm和外设调用按测试说明截获，验证不包括真实线程竞争、硬件、串口时延和实际制动。MCU磁盘镜像版本吻合不等于实机Flash逐字节相同。

运行要求：C编译器、Python及unicorn/pyelftools；全部测试离线执行：

```sh
sh navigation_runtime/reverse/verify_all.sh /tmp/feedbot_reverse_venv/bin/python
```

验证结果保存于 analysis/chassis_stack/*verification.json 和 analysis/reverse_artifacts/*verification.json。测试程序绑定原ELF/镜像SHA256，不能悄悄换二进制继续沿用结果。

## 对运动重构最有用的发现

### 平滑来自多层状态连续，不能只换规划算法

实际导航插件是GlobalPlanner与定制DWA，回充另用ChargeController/DockPerception。taskFSM向控制器传入跨周期保存的Twist；接近成功可以在同一次execute中进入最终入桩，并非每个阶段必须多停一拍。控制线程读取controller_frequency，构造默认20Hz；这是代码配置，不是实测调度保证。

Linux驱动还有smooth_pwm_auto：其内部保留两轴历史值并计算平滑结果。但后续完整发送函数验证已确认：只使用返回的线速度，返回的角速度未采用；详见MOTION_CHAIN_RECOVERY.md。零目标采用另一组参数。原实现使用进程静态历史值，不是每个对象独立状态；恢复模型把它显式传入，便于测试和未来隔离。正负增量分支及精度转换不完全对称，重构前应先理解再决定是否保留。

这里已实际发现自动反编译的不准确表达：原指令对ROS秒/纳秒先作double换算再转float，而伪代码显示的中间类型不可靠。C模型按汇编恢复，10000组测试逐位一致。

因此，自研导航若在规划器、控制器、驱动各层重复减速或重置状态，会让期望速度与最终输出不同。后续应统一速度/加速度约束和时间基准，保持控制状态跨路径段连续，明确谁拥有最终输出。

### 原驱动确实存在重复纯旋转指令的随机缩放

navi_cmd_CB原地址0x40d03判断：imu_compensate启用、当前线速度等于上次且为零、当前角速度等于上次时，跳到0x4159e调用rand，角速度乘以 `0.7 + 0.03 * (rand() % 11)`，范围0.70～1.00。该计算按double中间值执行再回写float。构造代码读取imu_compensate默认true；另有服务命令可更改它。

这是汇编支持的实际分支，不是名称猜测。但没有证明用户历史测试当时此标志状态及最终输出，也不能把所有卡顿都归咎于它。本次没有调用开关服务或修改参数。证据：driver_navi_callback_disassembly.txt和自动导出的00140a20.c、0014fc90.c。

### 日志中的“停前进”不一定对应那个局部分支实际写零的轴

近障条件的一个分支满足模式、近障标志及速度条件后将局部角速度清零，日志却写“stop forwarding”。同一回调其他分支确实可以清两轴，因此不能靠这条文字判断整车停车原因，更不能据此移除所有碰撞检查。完整回调已保留，后续仲裁实现应按实际字段及控制流建模。

### 充电反馈、到位和失败已能区分

实际DriverEntry.h确认反馈字节是ChargeStatus。stateCallback将“非0且非5”记为charging_。tasker.h确认WORKING=0、FAIL=1、FINISH=4；搜桩使用的另一组枚举中AVOID=1、SUCCESS=4，不能把不同枚举混为一谈。

完整execute静态核对：未初始化失败；charging_优先完成并清零；搜索超时可跳过搜桩继续；接近成功同周期进入最终入桩；最终控制返回false发布失败。最终入桩中“进展不足”退出不是“已经充上电”。+0x77d是xy_reached_，+0x77e是yaw_reached_。细节见CHARGE_RECOVERY.md。

### MCU超时零目标不是所有模式下的最后一次目标调用

原周期任务某状态组合可先调用零目标，再因pending_action=0x385调用target(300,0,5000)。873组中的对应调用轨迹与C一致。该状态组合在实机的可达性及下游互锁仍未证明，不能宣传为已复现真实故障，也不能把前面的零调用当作可靠的最终停车保证。

电机输出帧具有CANopen 0x60FF目标速度SDO形状，节点1/2、目标符号与发送顺序已通过405组对照；物理单位、比特率、完整驱动器状态机与制动效果未恢复验收。

## 上游来源与可复用性

已保存Fetch open_auto_dock melodic-devel，提交1a4baf0a474aff4267a53e067f3f8e43b3f1f564，以及ROS navigation melodic-devel，提交c81a8b1da643a58fe31957b6da2aa02f1ddc3fc7。精确来源见analysis/upstream/provenance.json。

安装版本中dock_candidate.h、icp_2d.h、linear_pose_filter_2d.h与该Fetch提交逐字节一致；perception.h和laser_processor.h有实质差异：桩位设置/机器人姿态接口、全局桩位和发布标志、采样字段由float改double、增加点差值、聚类接口变化等。差异已生成到vendor_header_diffs。头文件相同不能证明对应实现二进制相同。Fetch源码保留LGPL及原版权；ROS navigation各文件/包声明也须保留，不因仓库顶层没有LICENSE就当作无许可约束。

雷达已导出节点/协议相关函数，转发程序和实际配置也已取得；尚未证明设备内部固件、全部参数语义及硬件数据时序。定位建图已取得Cartographer/EKF配置与运行链路，没有宣称完成Cartographer定制二进制的完整恢复。

## 尚不能交付的内容及工程结论

当前没有厂家原始完整C++工程、匹配调试信息、MCU全部类型/外设初始化工程或实机Flash读回。嵌入ELF的/home/spirit/deliverRos/src路径是编译时路径，检查的move_base/src目录并不存在。有root权限只允许读取现存文件，不会生成已不存在的源码。

这批成果足以为独立驱动适配、回充控制及ROS2迁移提供可追溯依据，但不能直接写入车机当成量产替换。正确的工程边界是：原生C++驱动适配与统一命令仲裁→稳定里程计/定位接口→ROS2/Nav2规划控制→独立回充任务；网页只传任务、展示状态，不进入高频控制闭环。优先解决最终命令所有权与连续控制状态，不能靠更换Smac名称或移除安全判断保证顺滑。

完整可替换系统仍需实现未恢复的适配/初始化、离线回放与集成测试，再做有明确目标的硬件验收。这是明确剩余工程量，不以1726份伪代码冒充已经完成。

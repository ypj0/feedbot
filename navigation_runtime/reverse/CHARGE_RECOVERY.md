# 回充核心恢复：搜索、接近速度与状态衔接

2026-11-07。本轮全部离线，无设备命令或参数修改。绑定 libmove_base.so SHA256
`971b0895544c43f5ece5fd5cb73ac0ae03d5895046c373ffd01e4a94209e837e`。

## 可编译交付与验证

- `recovered/charge_core.c`：完整 foundChargePoint 函数、stateCallback 叶函数，以及 approach 中两个入口的速度计算切片。
- `verify_charge_core.py`：Unicorn 执行原 x86-64 指令，与本机编译 C 模型逐字节比较对象内存和 Twist，检查返回值以及搜索函数的外部调用顺序。
- `audit_charge_constants.py`：按 ELF 指纹提取已确认地址处的参数名、默认值和数值常量。
- 结果：foundChargePoint 896 组，stateCallback 256 组，approach 切片 1396 组，首轮共 **2548 组全部通过**；本次新增最终入桩切片1396组，累计 **3944组全部通过**。

模型用对象偏移保留尚未确认的私有字段名。getRobotPose 和 Time::now 在搜索验证中被替换为指定返回值，前者不模拟写入 pose；因此验证边界不包含感知、TF、时钟和硬件。此前速度切片只验证有限数值输入；本次另增加完整 approach/startGoCharing 的平面姿态验证，见末节，仍不等于整个回充模块等价。

复现：

```sh
cc -std=c11 -Wall -Wextra -Werror -shared -fPIC navigation_runtime/reverse/recovered/charge_core.c -o /tmp/feedbot_charge_core.dylib
/tmp/feedbot_reverse_venv/bin/python navigation_runtime/reverse/verify_charge_core.py /tmp/feedbot_charge_core.dylib
/tmp/feedbot_reverse_venv/bin/python navigation_runtime/reverse/audit_charge_constants.py
```

证据：`analysis/chassis_stack/charge_core_verification.json`、`charge_constants.json`。

## 搜索阶段：不是凭名称推断

完整函数 foundChargePoint 从 0x103af0 开始：

1. 对象标志 +0xe4 非零，或 DockPerception::getRobotPose 返回真：记录当前时间到 +0x10c，返回数值 4。
2. 感知失败且 +0x6f0 非零：返回 4，不在此函数里改写传入 Twist。
3. 其余情况增加 +0x6ec 的 32 位计数，清零 linear.x 和 linear.y，按下表设置 angular.z，返回 1。

| 自增后的有符号计数 | angular.z |
|---|---:|
| ≤149 | -0.3 |
| 150–449 | +0.3 |
| 450–749 | -0.3 |
| >749 | 计数改为150，输出+0.3 |

函数没有清空 Twist 的所有六个分量，只修改上述三项。计数是调用次数，不是秒；未取得 execute 的真实调用周期，不能直接换算搜索时间或转角。

execute 的 0x106ac0 分支另外比较 now 与 (+0x10c 时间 + +0x118 时长)。超时后清零 x/y/yaw 速度、置 +0x6f0=1，再进入后续控制分支。init 在 0x105d2c 写入 +0x118=30.0，区别于构造参数 charge_timeout 默认180.0。完整任务超时语义仍需追踪其他消费位置。

因此不能把 foundChargePoint 的返回 4 单独解释为“当前一定识别成功”：超时标志也可以走同一返回值。

## 接近阶段：已恢复的速度公式

两个已验证入口为 0x1056e2 和 0x105768，退出边界为 0x1055d0。入口接受上游已经计算/归一化的角度误差 e；第二入口额外置对象标志 +0x77f=1。此切片不包含前面的几何计算；新增整函数模型已包含平面几何计算。

构造函数已对应出：

| 成员偏移 | 参数名（保留原拼写） | 构造默认值 |
|---|---|---:|
| +0x708 | abort_angle | 0.0349 rad，约2° |
| +0x720 | max_rotatespeed | 0.3 |
| +0x730 | forword_speed | 0.08 |
| +0x728 | backword_speed | -0.07 |

当前快照中 max_rotatespeed=0.3；其余三项未出现在所查 /move_base 参数快照，默认值不能视为已直接读取当前对象值。

对有限输入，令 a=abs(e)、w=abs(传入Twist.angular.z)：

- 若 a≤abort_angle：linear.x=min(传入linear.x+0.03, forword_speed)，angular.z=0。
- 否则 linear.x=0，并计算角速度幅值：
  - a>max_rotatespeed：min(w+0.06, max_rotatespeed)。原指令确实直接比较这两个数值，未替厂家改写量纲。
  - 否则 a>0.5：以每次0.06增减，使 w 向0.6靠近。
  - 否则：向0.15靠近，增加步长0.06、减小步长0.09。
- 最后按 e 的正负给角速度加符号。

当 max_rotatespeed=0.3 时，第二个“a>0.5”分支在这段条件组合下不可达；测试仍覆盖更大的参数值，避免只按当前配置恢复。

这证明局部代码有基于传入速度的渐变处理，但**还不能证明传入值始终是上一个周期的输出或真实反馈速度**，其来源需沿上层调用者继续核实。也不能把每调用+0.03/+0.06直接称为 m/s² 或 rad/s²。

迁移时值得保留的是速度状态连续与误差闭环；固定调用步长应改成明确 dt 的加减速约束。不能把回充的低速数值原样作为任意路线导航参数，更不能由此承诺整个导航已顺滑。

## 执行状态衔接：已确认的控制流

execute 的反汇编见 `charge_execute_disassembly.txt`，以下是静态确认，尚未做全函数差分验证：

- +0x6e8 标志非零的分支发布 Int16(0)，清零 x/y/yaw，停止 DockPerception，并将结果字节写为0x56，再返回4。安装头文件已确认 Int16(0) 发布到 charge_fail，状态86发布到 navigation_status；返回4为 FINISH。
- 未触发上述分支时，先调用 foundChargePoint。
- 搜索继续时检查上述30秒时间窗口；搜索函数返回非1或超时后进入后续控制。
- +0x748 为零时调用 approach；返回0后置 +0x748=1 并停止 DockPerception。
- 接着可以在**同一次 execute 调用内**进入 startGoCharing，并不是必然先退出、停一周期、再进入下一阶段。
- approach 返回-1会走停止感知及结果发布分支。

stateCallback 叶函数实际计算“栈上传入消息字节 !=0 且 !=5”，写入 +0x6e8。其测试穷举全部256种字节；现已取得原始 DriverEntry.h，确认该字节为 ChargeStatus；这仍不是整个充电状态协议的硬件验证。

原厂同样有状态切换，区别在于切换不一定额外插入停顿。后续自研控制应明确哪些切换可在本周期衔接，哪些因几何、状态确认或实际制动必须停下。

## 激光桩识别：确认二维 ICP 调用

DockPerception::fit 的原指令在 0x126a8f、0x12701a 调用 icp_2d::alignICP；同一库还定义了 computeCorrespondences、getRMSD、alignPCA、alignSVD、transform 等函数。fit 中存在 rand 调用；具体随机初始化/重试条件尚未完整恢复，不能仅凭符号声称所有算法分支都已理解。

二进制构建路径含：

- `/home/spirit/deliverRos/src/navigation/move_base/src/charge_controller.cc`
- `/home/spirit/deliverRos/src/navigation/move_base/src/fetch_auto_dock/perception.cpp`
- `/home/spirit/deliverRos/src/navigation/move_base/src/fetch_auto_dock/laser_processor.cpp`

这些是**编译时路径字符串**，不是当前磁盘中已找到上述源码的证明。fetch_open_auto_dock 为待比对候选上游；本轮访问 GitHub/API 因 DNS 解析失败，没有下载成功，不声称已确认具体上游版本或许可。

DockPerception::set_dockpose 则已经静态确认：复制传入位姿，把内部 position.z 清零，并用原始 z 是否非零设置 +0x492。这与管理程序借 position.z 传模式类型的逻辑相呼应，不能将原始 z 当成真实桩高度。

## DWA 对照发现与剩余边界

- getFakeAngularspeed（0x2f760）仅返回成员 +0x990，不是角速度生成器。
- getOdomVel 调用 OdometryHelperRos 后，将返回 PoseStamped 的 position.x 乘以成员 +0x988。该成员由 revertspeed_CB 等路径设置，需完整确认符号/模式语义。
- setSpeedStop（0x31dc0）清零输出 x/y/yaw，以及对象 +0x890/+0x898/+0x8a0，随后重置一段内部姿态记录。完整调用条件、这些内部值的全部消费者尚未验证，不能据此断言某次卡顿根因。

下一层重点是：传入 Twist 的周期来源与最终输出；startGoCharing 的速度律；ICP 模板/匹配阈值；Linux 驱动的模式、近障和二次整形。当前恢复成果是离线可验证模型，未替换生产回充模块。


## 本次追加：最终入桩速度律与跨周期来源

新增 `recovered_charge_final_slice`，恢复 startGoCharing 两个入口：旋转分支0x103f65、倒车分支0x104092，统一退出到0x1040f4。上游几何、模式选取及后续进度判定不在这两个切片测试范围内。1396组随机/边界输入逐字节对照通过，原有2548组同时通过。

**最终对准与接近阶段的速度参数并不完全一样：**

- 最终对准仍基于传入角速度幅值渐变；大误差分支按0.06递增并限幅。
- 在相应小误差分支，目标幅值为0.05，增减步长0.04；前一接近阶段则以0.15为目标，不能把两段公式混用。
- 倒车分支设置成员+0x77e=1，令 `linear.x=max(传入linear.x-0.01, backword_speed)`，同时置 angular.z=0。backword_speed构造默认-0.07；不是实测当前车速。
- 若当前几何距离大于上次距离，增加+0x778计数；随后保存当前距离到+0x768。此处并没有因为一次距离增大立即停止。

静态分支显示+0x77e影响是否重新进入旋转：标志置位后，相关分支跳过回到对准旋转的路径。这是阶段保持机制，不是全导航都应忽略航向误差的依据；其余重置路径仍需按整机任务分析。

### 速度结构来源已进一步确认

1. taskFSM状态跳转表0x142a84中，状态7直接进入0x8aae0。
2. 0x8aaf5取`MoveBase对象+0x9d0`，在0x8ab13作为Twist引用传给虚函数调用0x8ab1c。
3. ChargeController虚表的+0x20槽，通过0x379648重定位确认对应`ChargeController::execute`。
4. execute将该引用继续传给foundChargePoint、approach和startGoCharing；发布路径0x8af02使用同一结构。
5. 状态0对应0x8a518，清零这个结构的x/y/yaw；taskStop、publishZeroVelocity也有明确清零位置。状态7入口没有每次初始化为零。

因此现在可以把它确定为**跨调用持久保存的命令状态结构**，不是在这些回充数值切片内读取的实测轮速。常规执行可继承此前写入值；其他停止/模式分支仍可修改它，尚未宣称排除了所有并发写入。证据见task_fsm_disassembly.txt、charge_execute_disassembly.txt、charge_command_provenance.json。

### 控制周期：从缺失参数追到了构造默认值

构造函数0x87165起读取controller_frequency，默认20.0，存入MoveBase+0x108；threadTaskExecute在0x8cc77读取该成员构造ros::Rate，并在循环中调用sleep。先前只查参数服务器，没有查到显式值；现在补充了二进制中的默认值证据。

这不等于实测20Hz，也不排除参数加载时值与快照不同。若实际恰好20Hz，倒车每调用减0.01才相当于约0.2m/s²的命令斜率；不可直接当作物理加速度。新控制器应使用dt表达斜率，避免改变线程频率时运动响应也改变。

### 最终入桩的进度退出条件（静态恢复）

距离由传入两个位姿的xy差计算欧氏距离。进入chargedis_threshold范围后，若相对+0x770记录值的变化绝对值小于0.025，就增加+0x764计数；自增后大于20时该函数返回false。变化达到阈值时清计数并更新距离基准；范围外则返回true。

这是以几何进展与调用次数为依据的退出条件，不是单凭该返回值认定接触成功或电池已充电。execute对startGoCharing返回false走停止/结果分支，真正反馈标志另有独立优先判断。本段现已纳入新增的整函数平面姿态差分测试。

### 本轮尚未完成的边界

Linux驱动收到命令后的所有模式、近障和二次整形分支仍未完整恢复；本轮没有证明历次异常停车的唯一根因，也没有更换生产执行器。已明确可用于新架构的机制是：跨周期保存命令状态、阶段内渐变、必要的阶段保持，以及同周期可衔接的状态转换。

## 本次整函数恢复及原始头文件核对

新增 recovered_approach_planar 与 recovered_final_planar，分别执行原 approach
(0x104710) 与 startGoCharing (0x103bc0) 整函数对照，各1200组通过。
包含平面几何、前进/旋转/倒车、阶段锁存和进展退出。保留原厂3.14、6.28、
3.1415926等常量，不用更精确常量悄悄改变算法。最大差值4.44e-16，比较容差1e-11。
验证截获ROS日志/字符串及libm调用，输入限定有限平面姿态；不包含任意三维姿态、
感知回调并发、TF实时性和硬件效果。verify_charge_planar.py先运行3944组原回归。

53份安装头文件位于 analysis/chassis_headers。charge_controller.h确认
+0x77d=xy_reached_、+0x77e=yaw_reached_、+0x77f=notpermit_back_move_。
tasker.h确认 ExecuteResult: WORKING=0, FAIL=1, CLEAR=2, RECOVERY=3, FINISH=4；
GPDState: FAIL=0, AVOID=1, GO_WAITING=2, IN_WAITING=3, SUCCESS=4。

execute完整伪代码位于 analysis/decompiled/libmove_base.so/002066a0.c：
未初始化返回FAIL并清零；charging_优先返回FINISH；搜索超时置skip_dock_pose_后
同周期继续；approach成功后同周期进入最终入桩；approach=-1或最终入桩false
发布charge_fail=5和navigation_status=87并返回FAIL。此编排为静态核对，
新增charge_execute.c现已对execute正常返回控制流做768组差分，截获几何控制、超时谓词及发布边界；不包括ROS字符串/日志/异常。stop()自身仅停止感知，清零责任需看上层。

原头文件与Fetch上游的dock_candidate、icp_2d、linear_pose_filter_2d头文件
逐字节一致；perception和laser_processor存在实质修改。上游仅可作为参考，
完整差异及提交指纹在 analysis/upstream/vendor_header_diffs 与 provenance.json。

本次统一交付入口为 [RECOVERY_DELIVERY.md](RECOVERY_DELIVERY.md)。

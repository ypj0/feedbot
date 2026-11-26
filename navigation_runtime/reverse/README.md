# APP、Linux驱动和MCU固件交叉逆向

最新全量清单和恢复工程见 [FULL_RECOVERY_STATUS.md](FULL_RECOVERY_STATUS.md)。下文保留分阶段证据；最新完成程度以整合报告为准。

只读提取和离线分析。没有升级固件、访问调试口、读取MCU Flash、发布运动指令或替换生产驱动。

完整导航、定位、激光、速度与回充链路审计见 [NAVIGATION_STACK.md](NAVIGATION_STACK.md)。
该报告区分实际运行参数、已恢复行为和未验证推断，并给出 ROS 2 重构边界。

回充搜索、接近/最终入桩速度公式及 3944 组原指令对照验证见
[CHARGE_RECOVERY.md](CHARGE_RECOVERY.md)，可编译模型为 recovered/charge_core.c。

## 可恢复程度

从底盘磁盘取得的是编译产物，不是原始C/C++源码。robot_driver为stripped ELF，仍有
动态函数名；包含gnu_debuglink指向`8de001f7afd7501279ade510acdc8197663dac.debug`，
但本轮检查的/usr/lib/debug中未找到匹配BuildID 3f8de001...的文件。驱动deb仅有
二进制、配置和包元数据，未包含源码。安装包内driver与实际安装driver逐字节相同。
可恢复协议、分支、近似伪代码；不能承诺原变量名、注释、类型和可编译原工程。

提取文件和SHA256见analysis/reverse_artifacts/manifest.json。离线文件保留在本地，
可以继续交给反编译器分析，不必反复读取运行中的机器。

## 新找到的MCU固件

`/home/spirit/stm32deb/sub.hex`：109078字节Intel HEX，2426条记录校验通过，
连续有效数据38764字节，地址0x08005000–0x0800e76b，初始SP 0x20002418，
复位向量0x08005145。符合Cortex-M Thumb固件结构；未据此识别具体芯片型号。

主机函数writeFiletoMotor(true)在0x4d56a引用该路径，0x4d574打开文件；
另一个分支打开/home/spirit/mcu.hex（本轮磁盘清单未发现此文件）。没有调用升级函数。
固件0x0800d0fc有`V4.41`，0x0800d108写入数值441；与先前实时反馈版本441一致。
版本吻合不等于Flash逐字节身份验证，也不能保证引导程序相同。

## 恢复的关键逻辑（人工命名的伪代码，不是原源码）

```cpp
// 收包类型1和类型2处理分支，0x0800d8d6 / 0x0800d8f4
copy_received_frame(..., 40);
ram_u16[0x200000bc] = 0;  // 两个分支均明确清零
ram_u8[0x200000c1] = 1;
// 类型2后续：按控制/充电等状态处理。
// 0x0800db6a：两个有符号大端目标字段
int16_t a = (frame[10] << 8) | frame[11];
int16_t b = (frame[12] << 8) | frame[13];
// 分支满足条件时送入共用目标例程0x0800d170；不是无条件透传。

// 周期任务0x0800c1f8内的部分逻辑，省略其他动作分支
++ram_u16[0x200000bc];
if (ram_u16[0x200000bc] > 18) {
    ram_u16[0x200000bc] = 15;
    ram_u8[0x200000c1] = 0;
    target_routine_0800d170(0, 0, 0);
    response_routine_0800dee4(0, 0);
}
```

同一个target_routine在0x0800d26a正常分支将输入写到目标结构0x20001e68；
它还包含状态判断和另一条渐变处理分支，因此上面的零调用不能等同于电机瞬时停止。
response_routine从7f7f/28帧头构造回复，不把它误称为电机输出函数。

SysTick向量指向0x0800b21c，该函数调用任务倒计时0x0800b7fc。主循环
0x0800b7c0遍历8个任务并调用到期函数。Flash候选任务表0x0800e658的第二项为
0x0800c1f9（Thumb地址），周期100 ticks；仍需完整验证启动时表装载、tick时基、
任务阻塞和各模式后续覆盖，不能直接宣称“1.9秒实车刹停”。这是MCU程序中的
失联处理线索，不是独立硬件看门狗的证明。

审计脚本按固件指纹绑定，提取9组有边界的函数片段和任务表到
analysis/reverse_artifacts/sub_timeout_evidence.json。全镜像线性反汇编也包含
数据/常量被当作指令的伪影，仅供索引，不作为全部函数边界正确的证据。

## APP与驱动交叉核对

从已有反编译zip提取1146个业务Java文件到analysis/app_reverse_sources。
JlNaviManager.cmd_vel将linear.x设为调用参数，将angular.z设为角度参数的负值，
通过ROS话题发布；这不是UART编码，不能直接复制APP角速度符号到ROS控制器。
setOutofChange发布Int16(1)，且APP有连接与初始化前置条件。

原厂Linux Motionmode_CB：输入1在内部0x45d条件允许时设置0x570/0x71c=1；
输入3设置为3；其他输入设置为2。没有在此回调里实现最终输出独占。
Androidcmd_CB的可达分支会写0x718及0x71a控制字段。结合此前getdatafromdriver
和内部动作审计，最终网关仍须覆盖模式及内部动作，不能只拦两个速度字段。
函数片段保存于analysis/reverse_motion_mode_callback.txt、reverse_android_callback.txt。
APP本轮未发现能直接证明MCU看门狗期限的实现。

## 复现与接下来的明确缺口

```sh
python3 -m venv /tmp/feedbot_reverse_venv
/tmp/feedbot_reverse_venv/bin/pip install capstone pyelftools
/tmp/feedbot_reverse_venv/bin/python navigation_runtime/reverse/inspect_hex.py analysis/reverse_artifacts/sub.hex analysis/reverse_artifacts
/tmp/feedbot_reverse_venv/bin/python navigation_runtime/reverse/audit_sub_firmware.py
```

下一步沿MCU的收包校验→命令分支→目标结构→下游电机写出完成交叉引用，
查明上面的失联零目标是否会被其他任务覆盖，并核实调度时基。已获得可离线分析
的固件，不再将“必须先向厂家拿到源码”视为唯一推进路径。当前仍不启用实车独占
授权；固件匹配和实测制动是独立的最终验收事项。

## 可编译恢复代码与原指令对照

新增recovered/sub_core.c：恢复0x0800c1f8周期任务、0x0800ceb6帧校验和
0x0800b7fc任务倒计时三个函数。它是可在本机编译的语义模型，不是可刷写固件。
外部函数用带地址的调用记录替代；未恢复的函数没有伪装成可工作的硬件实现。

verify_recovery.py使用Unicorn执行原固件的Thumb指令，与C恢复模型比较内存和
外部调用顺序。273项周期任务状态组合、400项40/80字节帧、200项任务表组合
全部一致。外部例程被截获，未模拟硬件、并发中断或外部例程反过来修改当前状态。
因此只证明这些输入下三个函数在该边界的等价性，不证明整个固件恢复完成。

重要新发现：周期任务的超时分支之后没有直接返回。注入silence_count=18和
pending_action=0x385时，原指令依次调用target(0,0,0)、response(0,0)、
target(300,0,5000)。原指令和恢复模型的调用轨迹一致。这说明零调用不是该函数
本次执行中无条件的最后一个目标；该组合在实际运行中的可达性、下游互锁及
最终电机响应仍需追踪，不能直接宣称已复现真实车辆失联后继续运动。

帧校验函数本身只核对7f7f帧头、0d0a帧尾与累加校验，不检查byte2或命令族。
上游调用者是否另外检查长度/类型需继续核实。恢复模型拒绝长度<3的输入，
这是模型的调用边界，不宣称原函数有此防御。

复现（macOS）：
```sh
/tmp/feedbot_reverse_venv/bin/pip install unicorn
cc -std=c11 -Wall -Wextra -Werror -shared -fPIC navigation_runtime/reverse/recovered/sub_core.c -o /tmp/feedbot_sub_core.dylib
/tmp/feedbot_reverse_venv/bin/python navigation_runtime/reverse/verify_recovery.py /tmp/feedbot_sub_core.dylib
```
证据：analysis/reverse_artifacts/recovered_core_verification.json。

底盘管理接口已只读核实为uid=0、gid=0，见analysis/chassis_admin_identity.txt。
继续提取不依赖破解root密码；未读取密码哈希，也未修改密码或SSH配置。

## 电机最终目标输出核心

新增recovered/motor_output.c，恢复连续调用链0x0800b0e8→0x0800ab00→0x0800ab84，
截获底层发送边界0x08005bfc，不访问硬件。该链先保存反馈和旧目标，再写入目标A、
对目标B作16位取负；方向变化标志按ARM的32位乘法回绕规则处理。对应状态字段
为0x37时，先发节点2，再发节点1。帧ID为0x600+node，8字节载荷为
23 FF 60 00及小端有符号32位目标，形状符合CANopen的0x60FF目标速度SDO。
此识别尚不证明电机型号、物理单位、CAN比特率或完整CANopen状态机。

verify_motor_output.py运行原Thumb指令，并比对恢复C的寄存器状态块、方向标志、
发送顺序、ID和全部载荷。405组正负/零/边界目标及状态组合全部通过；前述873项
回归也通过。发送函数为记录替身，不能当作CAN总线或电机制动验收。

构建与验证：
```sh
cc -std=c11 -Wall -Wextra -Werror -shared -fPIC navigation_runtime/reverse/recovered/sub_core.c navigation_runtime/reverse/recovered/motor_output.c -o /tmp/feedbot_sub_core.dylib
/tmp/feedbot_reverse_venv/bin/python navigation_runtime/reverse/verify_motor_output.py /tmp/feedbot_sub_core.dylib
```

## 后续ROS 2边界

恢复代码保持纯C/C++与无硬件离线测试，不依赖ROS1/ROS2消息。目标是用一个驱动
适配层提供反馈与命令，然后由ROS2/Nav2管理规划控制。现底盘为ROS Melodic，
RK为ROS2候选环境；原厂定位/建图/回充尚未移植，不能只换消息类型就声称原生ROS2。
ROS2接入与最终串口/CAN接管是两个独立工程边界。百度Apollo主要面向道路车辆，
不作为本差速AGV重构的基础平台；算法设计可参考，不因此引入整套道路自动驾驶栈。

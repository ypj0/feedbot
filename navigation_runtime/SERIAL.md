# 串口记录解码与离线回放（禁止视为已完成电机网关）

本模块只读取调用者提供的二进制记录，输出JSON分析结果，没有串口打开、
ROS发布、网络发送、帧编码或转发接口。输入经stdin提供；程序不接触设备路径。
`motion_authorized=false`、`forwarded=false` 在所有结果中固定保留。

## 当前核实的范围

仅覆盖已审计 `robot_driver` 的 `getdatafromdriver()` 普通主机→MCU发送路径，
不是MCU返回帧解析器，也不是整个设备协议（固件升级、其他命令和模式含义尚未覆盖）。
驱动指纹见 `DRIVER_BOUNDARY.md`；本轮另核查辅助库：
`librobot_driver_lib.so` SHA256
`2aa530305ec9f1e30fc8dc7ce95a3dc9bfec0ce46f287806c5f99c53f319461c`。

|字节偏移（从0开始）|已核实格式或来源|解释边界|
|---|---|---|
|0–1|`7F 7F`|仅此普通发送帧族|
|2|`28`，即40字节|不接受其他长度|
|3|1、2、3|对应已审计函数的三个分支，不能推断哪个分支无副作用|
|4–36|载荷|除下列字段外全部保留原始十六进制，不猜测业务含义|
|10–11 / 12–13（类型2）|两个大端有符号16位目标字段|名为target_a_raw/target_b_raw，不标成m/s、左右轮速度或PWM单位|
|4（类型2）|来自内部控制/模式字段的条件写入|原始值0不等于运动禁止|
|32 / 33 / 36（类型2）|分别来自内部导航、到点、顶升字段|保留原始值，不把任何组合判定为停车确认|
|37|前37字节累加和的8位取负值|前38字节之和mod256应为0；不是CRC或鉴权|
|38–39|`0D 0A`|固定帧尾|

证据：`analysis/driver_boundary_disassembly.txt` 的 `0x47885–0x478e4`
初始化、`0x47ac9–0x47b55` 目标编码、`0x47cef–0x47d5b` 导航字段与校验/发送；
`analysis/serial_helper_disassembly.txt` 的 `0x9010–0x9041` 确认字节顺序。
不同固件/驱动版本不得自动复用此解释。

## 解码器和回放工具

`SerialObserver` 固定40字节缓冲，支持任意分包/连续多帧；不保存无限增长的队列。
头、长度、帧族、尾或校验错误都会锁定错误，后续正确字节不会自行恢复。
EOF时半帧报告截断。上限128MiB用于离线记录资源预算，不是导航距离限制。
不做模糊重同步，也不把损坏字节解释成下一条运动指令。

`serial_replay` 从stdin读记录，stdout写逐帧JSON，stderr写汇总；失败退出码2。
它不是串口代理、MCU模拟器或实际运动授权器。校验有效只代表本格式完整，
既不保证完整业务语义，也不能检测所有多字节抵消错误、重放或恶意输入。

构建及本机示例：
```
cmake -S navigation_runtime -B /tmp/feedbot_serial_build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/feedbot_serial_build -j4
ctest --test-dir /tmp/feedbot_serial_build --output-on-failure
/tmp/feedbot_serial_build/serial_replay < navigation_runtime/tests/fixtures/serial_tx_synthetic.bin
```
RK隔离验证：`bash navigation_runtime/tools/build_serial_observer.sh`。
容器network=none，无硬件设备挂载；只输出到独立serial_review_build目录。

## 验证结果及下一步

本机6项CTest通过，ASan/UBSan通过；RK ARM64隔离构建与回放通过。
新测试包含10,285项断言场景，其中10,200种单字节损坏全部被拒绝，另覆盖所有
单帧切分位置、逐字节输入、三种帧族、截断、未知类型、关闭后输入，以及
“两个目标为零但模式字段非零”仍无授权。CLI的4帧样本是明确标注的人工合成数据，
不是UART抓包、实际电机试验或MCU看门狗验证。

下一阶段需要取得与当前版本对应的真实发送/返回记录，先用本工具对照实际帧。
采集方式必须避免抢读生产串口、改变串口配置或暂停驱动；不能把另开一个串口
读进程当成无影响旁路采集。随后补齐接收协议和原厂驱动隔离运行环境，才可以
用模拟串口验证代理断连/恢复。真实MCU失联制动仍需官方证据或用户启动的实测。

## 真实系统调用记录验证（2026-11-06）

已通过临时eBPF观察取得当前driver PID1679/FD13的3秒双向记录，未打开串口、
暂停驱动或发送运动指令。驱动使用单iovec的readv/writev。独立socketpair自测
196项收发事件通过后，实机记录749次调用，零丢失/探针故障/不支持调用，采集已结束。

TX：150次writev均成功40字节，共6000字节；现有C++ serial_replay全数解码通过。
RX：300次成功readv合计6008字节，另299次返回EAGAIN（非阻塞无数据，不是串口故障）。
原始流开头8字节是采集边界上的前帧尾段，保留未丢弃；随后150个40字节块均符合
7F7F/28头、0D0A尾及前38字节累加校验。RX业务字段仍未核实，不使用TX字段解释。

前后生产连接、stopped状态、任务ID、feedback_epoch及回充流程状态一致。
这是静止时系统调用缓冲区的观察，不是物理线缆抓包，也未验证动态目标单位、
MCU制动、看门狗或输出代理。下一步从接收解析汇编逐字段对照这份RX记录，
建立离线反馈模型，再验证隔离驱动和最终命令边界。

证据：analysis/serial_driver_real.summary.json、serial_driver_real_analysis.json、
serial_driver_real_tx_replay.json及production_before/after快照。
工具范围与复现方法见capture/README.md。

## RX运行反馈分支（2026-11-07）

新增显式`SerialDirection::McuToHostRuntime`，与默认TX解释共用严格帧完整性检查，
但不共用业务字段。当前RX只接受type=2且byte4=0x60的已审计运行反馈分支；
未知分支锁存拒绝，不猜测、不自动重同步。调用方式：

```
serial_replay --rx-runtime < frame_aligned_rx.bin
```

该工具要求调用者提供帧对齐的离线记录。本次原始6008字节保持不变，显式从偏移8
生成临时回放输入；前缀`00023801b9b60d0a`保存在审计报告，不在解码器内静默丢弃。

已核对`read_odom`栈中返回数组基址为rbp-0x120，以下偏移均相对完整40字节帧：

|字段|接收解析行为|证据地址|
|---|---|---|
|byte4|成功返回且值为0x60才进入该解析分支|0x4840b–0x48416|
|11–12、13–14|大端有符号16位，写入成员0x578/0x57c，再经比例参数处理用于ODOM参数|0x488c6–0x48955|
|29–30、31–32|大端有符号16位，写入0x580/0x584，并作为ODOM参数|0x48924–0x48996|
|35–36|大端有符号16位，写入0x588|0x48a17–0x48a31|

输出仅命名为`feedback_raw`，按11、13、29、31、35顺序排列；不将原始值直接标为
m/s、rad/s、电压或停车确认。特别是前两个数在驱动中还经过比例处理，不能直接
接入CommandArbiter的StopSample。状态字节的完整含义、单位和有效性仍需继续核实。

150帧真实静止记录全部通过，逐帧原始字节与独立Python大端解码一致；前四项均0，
第五项均441。证据为`analysis/serial_feedback_audit.json`、
`serial_driver_real_rx_decoded.jsonl`及`serial_driver_real_rx_status.json`。
测试包含所有单帧切分、10200种单字节损坏、重新计算校验后的错误分支、截断和故障锁存。
本机9项CTest通过，RX ASan/UBSan通过，RK ARM64隔离编译的10282项检查通过
（`analysis/serial_feedback_rk_verification.log`）。未接触实际串口或改变生产驱动。

这完成了运行反馈的有限离线模型，不是完整MCU模拟器或最终执行网关。
下一步仍需核实反馈单位/状态、补齐其他输出分支，再构建隔离驱动的模拟接收端；
MCU失联制动和可强制执行的最终控制权边界尚未验证。

## 已命名反馈字段与版本缩放（2026-11-07，后续核查）

重新读取实机驱动SHA256，与既有审计一致；读取安装的DriverEntry.msg/.h，对照
成员写入地址，现解码器额外输出`driver_fields`，原始字节和`feedback_raw`仍完整保留。

|帧偏移|DriverEntry字段|
|---|---|
|5、6、7、8|EmergencyStatus、MotorInitStatus、Power、ChargeStatus|
|9、10、27|LeftAlarm、RightAlarm、MotorEnabled|
|11、13（各2字节）|LeftEncoder、RightEncoder|
|29、31、35（各2字节）|Speedx、Speedth、Version|

byte33单独输出`wire_mode`：驱动对Mode有条件更新，并允许内部回调改写；不能
将该字节直接解释为网页最终Mode。byte28单独输出`wire_shutdown`：驱动内部标志
可以将发布的Shutdown覆盖为2。枚举名字不等于值的语义已核实；当前不生成驱动
启用/急停许可，也不直接生成CommandArbiter的StopSample。

`publish_ODOM`中Version≤437时速度除数为100，Version>437时为1000；
内部计算的速度输入分别为Speedx/divisor、-Speedth/divisor。
当前静止捕获Version=441，故该分支使用1000。这里核实的是代码中的数值换算，
不是对最终发布twist的所有分支或实车物理标定作保证。编码器先经过
left_pwm_scale/right_pwm_scale处理；此次ROS参数读值均1.0，不能代替运行中私有内存核验。

新增字段用不同哨兵值验证偏移，保留未知状态值、不擅自归一为布尔量；
真实150帧通过独立Python逐字段核对。Mac 9项CTest、ASan/UBSan、RK隔离10283项检查通过。
可复现审计：`python3 navigation_runtime/tools/audit_feedback_fields.py`。
证据：`analysis/driver_feedback_definition.txt`、`driver_feedback_scale_values.txt`、
`driver_feedback_semantics.json`、`serial_feedback_named_rk_verification.log`。

输出协议仍只覆盖普通40字节帧的结构。符号表还存在sendMotorReset、sendMotorStart、
sendMotorEnd、sendMotorFlashData、sendMotorJumpToApp等更新路径；这些名称本身不能
证明运动副作用，也不能忽略其存在。执行代理需要默认拒绝未审计帧，并在隔离环境
验证正常驱动启动是否依赖这些路径，之后才可设计可回滚的实机接管。

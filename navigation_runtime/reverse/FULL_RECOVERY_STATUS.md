# 全量逆向资料与核心恢复工程：最终清单

2026-11-07。本轮在上一阶段基础上继续扩展实际运行依赖，完成全量导出、关键函数手工恢复和离线验证。未发送运动命令、修改运行参数、启动原厂升级程序、重启生产服务或刷写固件。

## 当前交付到什么程度

**现有文件范围的全量反编译已完成；完整原厂工程及所有硬件行为尚未恢复。** 这两件事不能混为一谈：被优化删除的源码无法从二进制恢复，自动生成的类型/控制流也需要验证。本轮不以“0导出失败”宣称所有源码已经等价恢复。

- 27个导航及其设备/管理相关模块，共18371个已识别非外部、非跳板函数，全部导出。
- 其中6573个伪代码文件含工具警告；原地址、符号、调用关系、排除项和哈希均保留。未识别函数、间接调用及缺失类型仍可能存在。
- 10个C实现文件形成统一可构建的离线恢复库，附C/C++可用头文件。
- 35790组原二进制差分测试通过，另完成启动RAM复制/清零与SysTick设置的原指令执行核对。
- 取得8个开源参考仓库并固定提交；对安装导航头文件的45项比较中，16项一致、15项改变、14项未在对应上游路径找到。
- 保留53份原车机C++头文件，另提取12份雷达服务头文件，以及原有部署Python源码、配置和APP业务反编译文件。

这些数字表示文件处理和指定验证范围，不是“恢复率100%”，也不是实车通过率。

## 全量模块

| 层 | 模块 | 导出函数 |
|---|---|---:|
| 定位建图 | cartographer_node | 8820 |
| 定位融合 | ekf_localization_node、libekf、libukf、libfilter_base、libfilter_utilities、libros_filter、libros_filter_utilities | 1066 |
| IMU | imu_node | 50 |
| 代价地图 | libcostmap_2d、libcostmap_prohibition_layer、liblayers、libvoxel_grid | 2637 |
| 导航控制 | move_base、libmove_base、libglobal_planner、libbase_local_planner、libdwa_local_planner | 2642 |
| 激光 | ltme_node、liblaser_geometry、update_firmware | 694 |
| Linux驱动 | robot_driver、librobot_driver_lib、librobot_clean_lib | 1199 |
| 管理/交互 | spirit_api_server、spirit_robots_cost | 948 |
| MCU | sub_image.bin | 315 |

librobot_clean_lib是实际驱动的加载依赖，保留其资料不表示本轮操控其他设备。通用ROS、Boost、libc及操作系统实现不作为厂家自定义算法恢复范围。地图/激光图像HTTP服务的名称不能当作导航控制接口证据：spirit_api_server的入口注册了get_map_img/get_scan_img两个端点。

完整索引：[analysis/decompiled_full/README.md](../../analysis/decompiled_full/README.md)。每个模块有inventory.tsv、index.tsv、readable.tsv、calls.tsv、coverage.txt。全部伪代码文件的SHA256在corpus_manifest.json；源ELF身份分别在chassis_stack/binaries、reverse_artifacts、remaining_artifacts、loaded_artifacts、imu_artifacts清单中。

robot_localization的两处loadParams初次45秒反编译超时，经240秒预算单函数重试成功；原始失败日志、重试结果均保留。MCU启动复制/清零的两个间接调用函数由初始化表补充发现并导出，不能将最初自动发现列表当作所有真实函数的保证。

## 已构建的恢复模型与验证

| 核心 | 验证用例 | 边界 |
|---|---:|---|
| 回充搜索/反馈/速度切片 | 3944 | 原函数/指定切片，截获感知与时间等外部调用 |
| 回充完整平面接近与最终入桩 | 2400 | 完整函数，有限平面姿态；libm/ROS辅助调用截获 |
| 回充execute编排 | 768 | 正常返回路径、控制/发布调用顺序和状态；外部控制效果受控 |
| 驱动smooth_pwm_auto | 10000 | 历史状态及输出逐位一致；时间获取被截获 |
| 驱动40字节组帧 | 2400 | 完整正常返回路径，捕获串口输出及状态，读结果注入 |
| DWA线/角PID | 5000 | 完整数值函数，状态及返回值逐位一致 |
| 雷达OOB校验 | 2160 | 完整校验函数及缓冲区恢复；非负长度0..4096 |
| MCU周期任务/校验/调度 | 873 | 外设调用为记录边界 |
| MCU目标处理 | 2340 | 完整目标函数及原软件浮点比较 |
| MCU差速换算 | 4000 | 完整函数、门控、计数、原软件浮点运算 |
| 电机目标输出 | 405 | 状态、符号、顺序、帧ID与载荷 |
| MCU目标至最终帧集成链 | 1500 | 中间函数使用原指令，仅截获最终外设发送 |

测试集存在覆盖重叠，不应把累计数当作分支覆盖率。平面回充用1e-11数值容差，观察最大差4.44e-16；其他指定数值/内存比较按对应脚本逐位检查。未覆盖硬件电气、实际制动、实机中断时序和任意异常路径。

源码：[recovered](recovered)。统一接口recovered_models.h；CMake目标feedbot_recovered_models。本机已构建analysis/recovered_build/libfeedbot_recovered_models.dylib，只供本机离线使用，不是Linux/MCU部署二进制。

```
cmake -S navigation_runtime/reverse/recovered -B analysis/recovered_build
cmake --build analysis/recovered_build -j 2
sh navigation_runtime/reverse/verify_all.sh /tmp/feedbot_reverse_venv/bin/python
```

验证解释器需要unicorn、pyelftools；C编译禁用浮点乘加融合，不使用fast-math。日志在analysis/chassis_stack/recovery_validation_full.log；源码、头文件、脚本、验证证据指纹在analysis/recovery_delivery_manifest.json。

## 已查清、对重构直接有用的行为

1. **速度不是只由规划器决定。** 定制DWA包含基于内部历史输出的PID；回充保留跨周期Twist，并允许同周期阶段衔接。
2. **实际串口输出没有采用双轴平滑结果。** smooth_pwm_auto计算双轴，但发送函数仅使用平滑后的线速度；角速度直接读取另一个字段并限幅。已通过完整组帧执行验证，修正上一阶段仅看平滑函数的概括。
3. **特定条件下纯旋转有随机缩放。** imu_compensate等条件满足时，重复角速度请求乘0.70～1.00随机因子。不能据此直接判定过去每次卡顿的原因。
4. **内部动作与速度同帧仲裁。** 动作字段改变的帧速度为零，且一次性字段会被消费。只接管两个速度字段不能保证任务所有权独占。
5. **MCU目标至电机帧已连贯验证。** 包含目标接受、渐变、局部互锁、混合精度差速换算、符号调整及最终帧。仍没有将未知校准字段擅自命名为实测物理参数。
6. **固件任务表和名义时基得到进一步确认。** 原启动代码确实将任务表复制到RAM；SysTick按核心时钟/1000设置。主循环阻塞和due标志合并仍使名义周期不能等同于保证刹停时限。
7. **雷达控制有明确开源参考和车机定制。** 公开LDCP SDK已在本机构建；车机额外/laser_switch回调修改休眠请求，实际启停在run执行。OOB校验不能替代负载长度验证。

详细证据见 [MOTION_CHAIN_RECOVERY.md](MOTION_CHAIN_RECOVERY.md)、[LIDAR_RECOVERY.md](LIDAR_RECOVERY.md)、[CHARGE_RECOVERY.md](CHARGE_RECOVERY.md)。

## 还不能声称已经完成的部分

- 18371份伪代码并未全部人工转写和逐函数验证，尤其Cartographer、代价地图、ROS消息与模板辅助部分。参考上游源码并不证明厂家未改动。
- 完整可重编译的原厂ROS工程、MCU启动/外设工程、链接脚本和全部原始类型尚未重建；现有模型不是可刷写固件。
- MCU磁盘镜像版本与反馈一致，但没有实机Flash读回的逐字节比对；雷达设备内部固件不在已提取主机文件里。
- 未证明所有硬件模式、异步回调、中断、异常路径与历史实车故障的因果关系。

因此，本次可交付的是全量可追溯逆向资料、明确验证的核心恢复库和参考源码工程。不能诚实地把它称为“厂家全部原源码已完整恢复”，也不能直接部署为完整替代系统。

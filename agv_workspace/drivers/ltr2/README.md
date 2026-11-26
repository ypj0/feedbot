# LT-R2-B 导航输入候选实现

## 范围与当前限制

本目录重写并收紧原始数据解析，在独立工作区验证，不修改归档或原厂进程。
此前已验收的是**数据面候选实现，不是已获准接管的完整生产驱动**。型号依据已有真实查询记录：
`architecture/原厂底盘硬件接口与逆向资料汇总_2026-11-21.md`。
协议参照固定资料中的 `ldcp_sdk` 和 `navigation_runtime/replacement/lidar/`；
不把后者的历史实车通过记录算作新实现已经通过。

以下原有入口仅含离线回放和 Linux ROS1 被动接收。两者没有 TCP 控制连接、UDP 2105 端口占用、
启停/休眠命令、速度发布、TF 广播或原厂 `/scan` 替换。实时入口仍依赖原厂让雷达保持测量。
新增车机独立 TCP/UDP 驱动及所有权适配见 `MANAGED.md`，其验证结果单独记录。
目标已明确为驱动运行在 x86 车机；RK3588 仅负责通信和后续网页，不部署雷达驱动。

## 数据契约

- 仅支持本机已观测格式：IPv4、`192.168.10.160:2105` → UDP 2105，
  OOB 1040 字节、flags `0x0101`、每圈 8 块、每块 256 束。其他格式拒绝，不能泛称通用 LTME 驱动。
- 校验网络长度、IPv4 校验和、源地址/端口、OOB CRC、块布局和组帧时序。
  不接收 IP 分片；不以重复块、跨帧块、缺块或零填补生成扫描。
- 距离编码乘以 0.002 转为米；原始零值为 `+inf`、对应强度为零。
  `range_min=0.05`、`range_max=40` 是继承的驱动范围配置，不是实测精度保证；下游须丢弃范围外数据。
- 当前保留 ±1.65 rad 的裁剪意图，输出实际光束边界 ±1.647495366 rad、1075 点；
  `angle_max = angle_min + (N-1)*angle_increment`。不通过补点掩盖几何问题。
- 时间戳是内核接收时刻结合设备块间隔、裁剪起点的估计，**不是校准后的硬件采样时间**。
  设备计数回绕与帧序号回绕有独立检查；设备重启后应退出并重新启动候选进程，而非放松序号校验。
  单次帧序号前进最多 1024、跨帧设备计数间隔最多 153.6 秒；超出界限应重新启动验证会话。
- 接收年龄 200 ms、未来容差 20 ms、组帧 200 ms、无完整帧 300 ms 为候选软件阈值，
  不是经现场验证的停车或导航性能标准。不得据此保证避障安全。
- 没有新完整帧就不发布 LaserScan，绝不刷新旧帧时间戳。诊断中的 `fresh_input` 仅表示数据新鲜，
  `navigation_accepted` 始终为 false；诊断不是运动使能或安全联锁，下游还必须检测诊断本身过期。
- 新鲜度按末块内核接收年龄加单调时钟已过时间计算，不因出队而重置年龄。
  单调基准在墙钟年龄采样前取得；消息构造完成后、发布前再次检查墙钟与累计年龄。
  这不是硬实时保证：线程仍可能在最后检查后或 ROS 传输中暂停，消费者必须自行检查消息过期。
  截断报文被丢弃并计数，不因无关大包终止接收；该丢弃计数包含非雷达流量。

## 构建与离线验证

从资料包根目录执行；依赖 CMake、C++17 编译器。无 ROS 时也能构建解析器、回放器和测试。

```sh
cmake -S agv_workspace/drivers/ltr2 -B /tmp/ltr2-build -DLTR2_SANITIZE=ON
cmake --build /tmp/ltr2-build -j 2
ctest --test-dir /tmp/ltr2-build --output-on-failure
/tmp/ltr2-build/ltr2_replay analysis/body_navigation_20261107/lidar_native/pair_1794045504/raw.bin
```

回放器读 `<little-endian double receive_time, uint32 packet_length, Ethernet packet>` 记录，
stdout 为逐帧 JSONL，stderr 为计数；`null` 表示无测距。输出明确标注 `replay_not_live`，
不向 ROS 发布。历史记录无单调时钟，只能验证记录内相对时间，不能验证实时接收年龄。
丢弃计数统计已开始且被中断的帧；采集窗口首尾的半帧不输出，也不自动计作网络丢包。
空输入或解析错误退出非零，详查计数，不以命令成功替代验收。

## 有界旁路实测

在有 ROS Melodic、原厂雷达仍测量的 Linux 车机上构建（不是在 Mac 运行原始套接字）：

```sh
source /opt/ros/melodic/setup.bash
mkdir -p /tmp/ltr2-live-build
(cd /tmp/ltr2-live-build && cmake /tmp/ltr2-source -DLTR2_BUILD_ROS1=ON -DPYTHON_EXECUTABLE=/usr/bin/python2)
cmake --build /tmp/ltr2-live-build -- -j2
/tmp/ltr2-live-build/ltr2_passive_ros1 _duration:=15 _interface:=enp2s0
```

需要 root/CAP_NET_RAW；不启用混杂模式。话题为 `/candidate/ltr2/scan` 和
`/candidate/ltr2/diagnostics`，非 latched。运行前确认候选话题没有其他发布者、生产导航没有订阅它，
不得通过 ROS 重映射将候选输出接入生产 `/scan`。最长 60 秒，不安装自启动服务。

Linux 另构建 `ltr2_passive_input_test`，需 root/CAP_NET_RAW 单独执行，不加入默认 CTest。
它只向本机 `127.0.0.1` 临时端口发送一个 65507 字节 UDP 包，验证截断接收被计数而不退出；
不向雷达或底盘发送数据。物理雷达断流、控制面失联与停车测试不能由它替代。

## 正式接管前必须完成

1. 新采集原始包与原厂输出逐束、一对一比较；检查强度、无效值、角度和所有拒绝计数。
2. 以标靶实测距离、左右方向、遮挡、安装外参；依据该型号正式规格冻结容差。
3. 标定时间基准并验证 IMU/里程计/TF 配合、运动畸变、主机负载和长时间丢包。
4. 复用并复审已有 TCP 生命周期与所有权方案，先在模拟环境测试，再于满足 G3 的维护窗口
   验证停原厂、防原厂自动拉起、独占端口、断流、退出、恢复；不通过直接 kill 进程抢占控制权。
5. 验证休眠/断联期间导航停止使用数据、恢复需显式授权，再接入用户实现的导航算法。

## 证据

本轮结果放在 `agv_workspace/acceptance/runs/20261122_ltr2_01/`，区分本地测试、
历史回放、当前实车回放和候选 ROS 发布。未执行项目明确保留，不能从已通过项目推断全部通过。

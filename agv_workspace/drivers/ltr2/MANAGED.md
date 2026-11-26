# 车机独立 LT-R2-B 驱动

## 架构与交付边界

雷达 TCP/UDP → x86 车机 ROS Melodic 驱动 → ROS 消费者。
RK3588 仅承担通信中转和后续调试网页，不运行雷达驱动、不承担驱动所有权或安全联锁。
`managed_ros1.cpp` 是独立控制/接收入口；原 `passive_ros1.cpp` 只用于对照，未继续叠加控制功能。

复用固定厂商 SDK 的 TCP 协议，不修改 SDK、雷达固件或持久化网络设置；
UDP 使用独占普通套接字，不需要抓包权限。沿用已验证的数据解析和发布前时效检查。
诊断中的 ready 只表示新完整扫描可用，`navigation_accepted` 始终 false。

## 生命周期接口

以下 ROS1 `std_srvs/Trigger` 服务由车机监督器调用：

| 服务 | 行为 |
|---|---|
| `/ltr2_driver/configure` | 验证所有权、占用 UDP、连接 TCP、核对 LT-R2-B 与 OOB 设置；分别确认停止数据流和测量后才进入 inactive |
| `/ltr2_driver/activate` | 清理旧帧、启动测量和数据流；收到控制应答之后的新完整帧才 ready |
| `/ltr2_driver/deactivate` | 先撤销 ready，再分别停止数据流和测量 |
| `/ltr2_driver/cleanup` | 清理连接与接收资源；停止未获确认时返回失败，不冒充物理已停 |

默认输出 `/candidate/ltr2/managed_scan` 和 `/candidate/ltr2/managed_diagnostics`。
生产 `/scan` 切换须另行批准和验证，当前不能同时启动两个生产发布者。
断流或控制查询失败进入故障；恢复须 cleanup → configure → activate，不自动重连并恢复测量。
每秒查询一次控制端配置；故障 RPC 可能阻塞主循环，消费者必须检查消息和诊断 TTL。
同步 RPC、ROS 传输及普通 Linux 调度不提供硬实时保证。
初始停止任一应答失败即配置失败，不允许启动；身份或 OOB 核对失败时不发送控制写命令。
因此配置成功但未启动的交接，以及配置后立即 cleanup，均以已确认停止为前提。

## 构建与隔离验证

在车机上使用同一个版本目录的 `source/` 和 `build/`：

```sh
source /opt/ros/melodic/setup.bash
mkdir build && cd build
cmake ../source -DLTR2_BUILD_MANAGED=ON -DLTR2_BUILD_ROS1=ON \
  -DLDCP_SOURCE=/opt/feedbot-nav2/lidar/vendor/ldcp_sdk \
  -DPYTHON_EXECUTABLE=/usr/bin/python2
cmake --build . -- -j2
ctest --output-on-failure
python2 ../source/tests/test_managed_ros1.py ./ltr2_managed_ros1 /path/to/recorded/raw.bin
```

隔离测试使用自建 ROS master、回环 TCP 模拟雷达和基于录制载荷重写序号/计数/CRC 的 UDP，不使用生产 ROS master，
不把模拟启停、断流和恢复算作真实雷达验收。SDK 源码在上传/构建时单独保存哈希。

## 所有权与安装

不重新实现事务监督器：复用 `navigation_runtime/replacement/lidar/supervisor.py` 的
“禁止运动 → 停原厂及 watchdog → 确认端口释放 → 发许可 → 启动/配置/激活”流程。
失败时撤销许可，停止新进程并确认端口释放之后才恢复原厂。
本目录提供 ROS1 启动/服务适配器和配置示例；车机版本目录的 `source/deployment` 存放适配器，
`build` 存放可执行文件，批准后才将 `/opt/feedbot-ltr2/current` 指向整个版本目录。

驱动的 permit 和本地锁只约束协作进程，不能独立阻止未改造的原厂 watchdog；
必须由监督器完成实际交接，不能手写 permit 抢占设备。
配置中的两个 site 钩子仍需现场实现和验证，不能直接启用示例或伪造成功返回。
安装源码和可执行文件不等于服务已启用，更不等于允许生产接管。

## 按顺序执行的剩余验收

1. 车机完整软件隔离测试与新鲜证据归档。
2. 维护窗口按既有静止交接条件验证真实查询、唤醒、休眠、独占、退出和原厂恢复。
3. 标靶距离/左右方向、机械外参、时间基准，以及 IMU/里程计/TF 配合。
4. 长时间运行、负载和实物断流/恢复验收；完成后启用生产服务。
5. RK3588 接入车机诊断与扫描可视化，控制按钮通过车机监督器执行，不直接连接雷达。

本阶段实测结果另见同目录 `MANAGED_RESULTS.md`；此前被动观测结果见 `RESULTS.md`。

后续已完成一次真实独占驱动及 RK 网页链路测试，结果见 `REAL_LIDAR_RESULTS.md`。
原厂现已恢复；本轮自动恢复出现过 ROS 客户端端口误判，不能把模块通过等同整轮或生产验收通过。

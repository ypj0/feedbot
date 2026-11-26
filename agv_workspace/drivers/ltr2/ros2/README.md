# LT-R2-B ROS 2 发布适配

直接包含相邻 `../include/ltr2/core.hpp`、控制层和 UDP 输入；它们是 managed-v3 实车对照基线，本适配不修改解析或组帧。归档 `navigation_runtime/replacement/lidar/packet.hpp` 不参与构建。

## 发布与时间

- `/scan`：`sensor_msgs/msg/LaserScan`，SensorDataQoS，`frame_id=scan`，冻结裁剪 ±1.65 rad，实际离散端点约 ±1.64749537 rad、1075 点。
- `header.stamp`：完整帧组装完成时的节点系统接收时间，不使用核心的采集时间估计或雷达原始时钟映射。**绝对时间对齐未验证**；它不是已标定的首束采集时刻。
- `scan_time`、`time_increment`、光束数组及角度字段直接转换核心输出。
- `/diagnostics`：发布帧率、累计拒绝帧数、过期数据拒绝数及坏包数；不补帧、不重发旧数据。

## 生命周期与独占

正常扫描及周期诊断通过 `RealtimePublisher::try_publish` 交给独立发布线程；
收包循环不等待 DDS。拒绝交接累计 `scan_handoff_drop_count` /
`diagnostic_handoff_drop_count`，不重发。`received_frames` 和 `frame_rate_hz`
表示有效完整帧接收，`scan_handoff_count` 表示交接成功，不表示 DDS 完成。
非 active 阶段的强制诊断仍同步发布；激活完成的诊断也使用非阻塞交接。

型号/OOB 的四次 TCP 查询仅在 configure/activate 边界执行，不再每秒进入收包回调。
SDK 单次查询等待上限 500ms，不能与 300ms 完整帧截止时间共用采集循环。
原有 200ms 输入时效、300ms 完整帧超时、独占许可和失败行为不变。
运行中外部修改配置不再靠周期查询发现；帧中断仍触发原超时。
扫描发布线程仅在 active 阶段创建，停止硬件、释放 UDP 后销毁并等待结束；
deactivate 返回之后不再提交新扫描。异步发布不保证 DDS 时延，始终保留接收时间戳。

节点名 `/ltr2_driver`。使用现有 supervisor 的 permit 和硬件锁，原厂 `spirit.service` 必须分时退出。

1. `configure`：获取锁、连接并验证控制端、停止测量和数据流。
2. `activate`：先重新核验型号/OOB，再独占绑定 UDP 2105、启用已有控制逻辑，收到完整新帧后进入 active。
3. `deactivate`：停止测量及数据流，释放 UDP；再次 activate 重新绑定。
4. `cleanup`：释放 TCP、控制对象及硬件锁。

失败返回失败状态或非零进程退出并记录错误，不自动重试。除了既有 `owner_directory`、`owner_token`，没有新增驱动配置项；默认所有权目录 `/run/feedbot-lidar`。不得与原厂或官方对照节点同时激活。

## 构建与测试

在已冻结的 Jazzy 镜像中运行，`/work/source/ltr2` 为本目录的父目录，`/sdk` 为既有官方 LDCP SDK 的只读挂载：

```sh
source /opt/ros/jazzy/setup.bash
cmake -S /work/source/ltr2/ros2 -B /work/build -DLDCP_SOURCE=/sdk
cmake --build /work/build -j2
ctest --test-dir /work/build --output-on-failure
```

五项测试涵盖 ROS2 消息转换、接收时间戳、几何与数组保持，以及既有核心、时效、生命周期、UDP 独占测试。`tests/trial.py` 是本次固定部署路径的实车验收脚本，不是常驻服务；它会停原厂服务、测试两次激活、恢复原厂并验证数据及端口。仅在明确批准的实车窗口执行。

验收探针将 DDS 首帧发现时间与首帧后 6 秒的数据窗口分开记录；验收帧率范围为 14–16 Hz、最大帧间隔 0.2 秒。订阅端接收延迟逐帧留存，不将它当作冻结核心的 UDP 输入时效检查，也不丢弃首帧。CLI 取证显式指定消息类型，并使用 `--full-length`，避免类型发现竞争与默认数组截断。

部署位置：车机 `/opt/feedbot-ltr2/releases/20261122-ros2-v1`。不在 RK3588 运行驱动，不修改原厂服务或自动启动配置。

新增第六项 `scan_handoff` 回归测试：占用发布互斥锁时立即拒绝交接，不改写待发帧；
解锁后数组与接收时间戳保持，销毁发布线程后不重发。它不等同于实车启停验收。

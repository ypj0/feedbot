# Run28 手推地图保存

状态：**实车静态通过（当前地图导出与文件核验）**；建图/定位整阶段未通过，地图质量和重载定位尚未验收。用户报告已经手推完成目标区域扫描。

保存当前运行中 `slam_toolbox` 的 `/map`，不加载地图、不切换服务、不发送运动指令。

- 车机持久目录：`/opt/feedbot-maps/20261123_run28/`。
- 本机副本：本目录 `feedbot.yaml`、`feedbot.pgm`。
- 61×213 栅格，0.05m/格；覆盖矩形约 3.05×10.65m，不等于可通行面积。
- YAML 引用同目录 `feedbot.pgm`；PGM P5、最大灰度255，12993个栅格字节与宽高一致。
- PGM SHA256：`2d4ba3ce81f6cb724f404e3a4d2b75fa8d8b48c979ea99c62a241cd9cc2f372e`。
- YAML SHA256：`7f332d28a3c70311d0b09df5f76eaa46f504808b32251d8763915966c04b02bf`。
- 两端文件散列一致；地图会话继续运行，未执行原厂恢复。

首次 CLI 保存默认 2 秒内未收图，报 `Failed to spin map subscription`；调用现有 `/slam_toolbox/save_map` 返回 255，其日志显示内部同样启动默认 2 秒的 map_saver。直接无 daemon 的 5 秒发现确认 `/map` 发布者为 slam_toolbox，RELIABLE / TRANSIENT_LOCAL；此前 daemon 查询显示 0 发布者，不能作为停发证据。

使用官方 CLI 的 `save_map_timeout:=15.0` 后单次保存成功：开始 1795385230.169，收到地图 1795385232.545，耗时约 2.38 秒。没有修改 DDS、驱动 200ms 阈值或失败行为，没有自动重试；该证据说明默认保存等待窗口不足，不单独区分发现延迟和数据传输延迟。成功原始日志见 `save_with_discovery_window.log`，首次失败日志见 `save.log`。

归档网页的保存流程可复用，但其 `/save_map` 是原厂 ROS1 服务，不能保存本次 ROS2 SLAM 的地图。下一步将 ROS2 保存操作与结果接入新网页；重载和 AMCL 定位另行执行，不能与当前建图同时发布 map→odom。

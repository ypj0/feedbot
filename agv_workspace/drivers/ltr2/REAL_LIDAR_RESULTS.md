# LT-R2-B 实物驱动与可视化验证

日期：2026-11-22。证据根目录：
`agv_workspace/acceptance/runs/20261122_ltr2_real_01/`。

本页主体保留上述首次实物试验及其恢复异常。后续官方分时对照已完成，
状态为“实车静态通过（雷达对照）”，且原厂自动恢复验收通过；见
`agv_workspace/acceptance/runs/20261122_ltr2_comparison_01/RESULTS.md`。
该后续试验未校时、未发送运动指令，也不代表导航可用。

## 结论与当前状态

独立 ROS1 v3 驱动已安装到 **x86 车机**，并实际独占 LT-R2-B 完成
配置、启动、停止、再次启动和清理。RK3588 仅运行网页及通信代理。
实际扫描与网页链路通过；整轮自动恢复未通过，随后人工停止只读桥并调用
同一恢复工具成功恢复原厂。保留原始 `trial.json` 的 `passed=false`，不改写历史。

当前原厂 `spirit.service` 与 `/ltme_node` 正常运行；新驱动、私有 master、
只读桥及恢复定时器均已停止。未设开机启动、未建立 `current` 链接、未替换生产 `/scan`。
这不是永久生产接管，也不是导航验收。

RK 网页仍可通过 `http://127.0.0.1:8086/` 访问（依赖当前 ADB 转发）；
车内地址为 `http://10.7.5.234:8086/`，该地址由 DHCP 分配，可能变化。
当前显示 OFFLINE 是预期状态：页面不会拿原厂扫描或历史帧冒充独立驱动实时数据。

## 实测证据

| 项目 | 结果 |
|---|---|
| 初始配置、休眠 | 成功；配置后 1 秒无扫描 |
| 第一次启动 | 10 帧实测；短窗口接收频率 18.29 Hz |
| 停止、再次启动 | 停止后 1 秒无扫描；再次收到 10 帧，16.15 Hz |
| 帧结构 | 每帧 1075 距离与 1075 强度；角度几何与点数一致 |
| 距离数据 | 两组采样均无 NaN、负无穷或有限越界距离；正无穷表示无回波 |
| 诊断 | `ready=true`、`healthy=true`、`navigation_accepted=false` |
| 真实网页 | 20 次 HTTP 实时响应；首个网页快照有效回波 1030/1075 |
| 浏览器停流 | 自动清空旧点、显示 STALE，无页面脚本错误 |
| 原厂恢复 | 恢复工具返回成功；3 帧新原厂扫描，TCP/UDP 由 `ltme_node` 独占 |

频率仅为各 10 帧窗口观测，不是标称频率或长期稳定性结论。
现场未移动测距标靶，不能据画面声明距离精度、安装方向或外参已经标定。

实时截图：`web_live.png`；停流截图：`web_after_trial.png`；浏览器记录：
`browser_real.json`、`web_first_live.json`、`web_last_live.json`。
车机原始记录及当时执行源码保存在 `car_evidence.tar` 和
`car/feedbot-ltr2-real-20261122-01/`。

## 恢复异常与根因修正

停止私有 master 后，网页桥留下指向远端 `11372` 的 CLOSE-WAIT 客户端连接。
旧检查同时匹配本地、远端端口，把客户端误当成占用 master 端口的服务，阻止恢复。
实际没有雷达双占用。停止只读桥后，原恢复工具完成恢复；原厂管理器与归档监督器
源码哈希未改变，恢复后的 `/driver_info` 与 `/wheel_odom` 速度仍为零。

修正版按 socket 端点语义判定：私有 master 检查**本地**端口，雷达仍检查
本地 UDP 及远端 TCP 2105。新增实际故障形态回归，Mac 与车机各 10/10 通过。
修正版单独暂存 `/var/tmp/feedbot-ltr2-validation-v3/`，没有覆盖本轮执行源码，
也没有再次停止原厂服务验证完整恢复闭环。

## 软件与部署路径

- 车机驱动：`/opt/feedbot-ltr2/releases/20261122-managed-v3/build/ltr2_managed_ros1`。
- 车机/RK 网页代码：`/opt/feedbot-dashboard/releases/20261122-lidar01/lidar/`。
- RK 本次运行单元：`feedbot-lidar-dashboard.service`；不是开机自启服务。
- 网页本地及 RK Python 测试各 9/9；车机桥测试 5/5，Python 2 实际 ROS 消息类型测试通过。
- Chrome 合成数据测试单列 `browser_fixture.json`，不计作实物证据。

下一步：修正版完整恢复复测、长期采样与丢帧统计、实物距离/左右方向/时间及 TF
标定，再解决原厂 watchdog 与独立驱动的长期所有权交接，才进入生产服务部署。

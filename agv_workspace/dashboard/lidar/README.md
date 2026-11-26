# LT-R2-B 只读可视化

## 边界与状态

本目录提供两段只读链路：x86 车机上的 Python 2 ROS1 桥只订阅
`/candidate/ltr2/managed_scan` 与 `/candidate/ltr2/managed_diagnostics`；RK3588 上的
Python 3 服务只代理固定上游 `http://10.7.5.88:8096/api/scan` 并托管本地静态页面。
它没有应用级 ROS publisher、ROS service client、运动按钮、雷达控制入口、任意 URL 代理、rosbridge、CDN 或第三方依赖；rospy 自身可能发布 `/rosout` 日志。

2026-11-22 已安装到车机与 RK3588 的 `/opt/feedbot-dashboard/releases/20261122-lidar01/lidar/`，
并完成真实独立驱动到浏览器的短时显示测试。RK 网页当前运行，车机桥随维护窗口结束已停止，
故当前应显示 OFFLINE。没有设置开机启动。完整证据与恢复异常见 `../../drivers/ltr2/REAL_LIDAR_RESULTS.md`。
页面明确标记 `managed candidate`，不把候选话题冒充原厂 `/scan`，也不表示导航已经接受该数据。

车机快照使用消息到达回调时的单调时钟计算源端年龄。超过默认 0.5 秒时返回 HTTP 503，且响应不含旧 ranges；
RK 无法连接车机时返回 HTTP 502。浏览器使用有超时的单次轮询，并把源端年龄、请求耗时与本地接收后时间相加；
总年龄超过 0.5 秒时即使请求仍阻塞也会清空旧点。暂停只冻结本地历史帧，并持续显示
`PAUSED · NOT LIVE`，不会把冻结帧标为实时。ROS 的 header 时间戳仅供下载检查，不参与源端新鲜度判定。

## 车机启动（真实驱动验收后）

要求 Ubuntu 18.04、ROS Melodic、Python 2.7，且车机确实拥有 `10.7.5.88`。桥本身不启动 ROS master。
私有隔离 master 与将来的生产 master 二选一，必须显式设置；不要在不清楚 master 所属时启动。

```sh
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11372
python2 /opt/feedbot-dashboard/lidar/car_bridge.py --bind 10.7.5.88 --port 8096
```

将来若批准连接生产 master，先把环境变量改为经现场确认的 URI，再执行同一条 Python 命令。不要同时连接两个 master，
不要重映射至 `/scan`。HTTP 端点是可信车内 LAN 上的无认证只读接口；没有控制能力，但不应暴露到公网。

本机只读检查：

```sh
curl --max-time 1 http://10.7.5.88:8096/api/scan
```

HTTP 200 表示扫描仍在源端新鲜窗口内；503 表示等待或过期，不能当作雷达在线。

## RK3588 启动

要求 Ubuntu 20.04 与 Python 3。服务默认监听 8086，固定读取车机 `10.7.5.88:8096`，没有可配置的任意代理目标。

```sh
python3 /opt/feedbot-dashboard/lidar/rk_server.py --bind 0.0.0.0 --port 8086
```

从能通过 ADB 到达 RK3588 的 Mac 转发并打开：

```sh
adb forward tcp:8086 tcp:8086
open http://127.0.0.1:8086/
```

RK 的 8086 同样是无认证只读服务，仅适合可信维护网络或 ADB 转发。页面上的缩放、暂停和下载只在浏览器本地操作；
下载内容为当前显示的有界 JSON 快照，不向雷达或 ROS 发命令。

## 离线验证

从资料包根目录执行，不需要 ROS、硬件或网络：

```sh
python3 -m unittest discover -s agv_workspace/dashboard/lidar/tests -p 'test_*.py' -v
node agv_workspace/dashboard/lidar/tests/test_ui.js
```

第二条仅在本机已安装 Node.js 时运行，不需要 npm 包。Python 测试使用伪 ROS 消息，覆盖单调时钟过期、非有限值转 JSON
`null`、1075 点几何、固定代理目标、上游不可达及无效上游响应。

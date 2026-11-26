# 手推建图仪表盘

网页参考归档建图工具的地图主视图、手推流程与状态栏，使用现有独立 ROS2 后端，不调用原厂 rosbridge 建图服务。提供地图/激光/车体叠加、缩放平移、全图与跟随、最近 3000 个页面位姿采样的轨迹，以及轮位置、轮式速度、原始 MCU 状态和数据年龄。

网页读取实时数据，提供“完成扫描，保存地图”和 YAML/PGM 下载，不提供脉冲控制、软件解锁、回充、脱桩或加载按钮。实体解锁需由操作员确认；`motor_free` 原始值不套用原厂 APP 的 33/34 枚举，不据此宣称已解锁。

车机 Jazzy 容器中运行：

```sh
FEEDBOT_TEST_TOKEN_FILE=/key/token FEEDBOT_TEST_LOG=/results/manual_trace.jsonl \
  python3 /work/agv_workspace/dashboard/mapping/car_bridge.py
```

桥接固定监听 `10.7.5.88:8098`，订阅 `/map`、`/scan`、TF、底盘反馈，并且是本仪表盘唯一的 `/wheel/cmd_vel` `TwistStamped` 发布者。不要同时启动独立底盘网页桥接；本程序不访问串口。

日志使用 `../chassis/event_log.py` 的独立写入线程，部署时需包含该文件。磁盘write/flush不占用控制回调或Runtime锁；日志失败显式退出、停止先发零速、不自动重试。日志容量及退出边界见底盘网页README。run41仅代码与Jazzy隔离测试通过，尚未部署至实车会话。

RK 上运行：

```sh
FEEDBOT_TEST_TOKEN_FILE=/key/token python3 agv_workspace/dashboard/mapping/rk_server.py
```

浏览器只访问 `http://127.0.0.1:8088/`。RK 服务使用同一 token 代理固定车机端点。`GET /api/state` 返回控制和底盘新鲜度，`GET /api/map` 返回最新完整地图及其年龄，并且只返回未过期的地图坐标系激光点和机器人位姿。TF 缺失显式显示，不伪造单位变换。原有 `POST /api/command` 后端接口保留，本网页不调用；这不是移除了后端运动能力。

静态 HTML/JS/CSS 由 RK 每次请求读取，更新三文件后刷新浏览器即可，无需重启车机桥接、驱动或 SLAM，避免丢失正在建图的会话。断线时清空实时读数；轨迹保留历史段但不跨断流连线。

## 地图保存

车机上的独立 `map_store.py` 在 Jazzy 容器监听 `10.7.5.88:8099`，RK 只代理 HTTP。保存服务不访问串口，不发布速度或 TF，不管理 SLAM 生命周期。将该文件放到车机 `/opt/feedbot-map-store/map_store.py`，执行 `bash start_map_store.sh`。容器 `feedbot-map-store` 复用已验证的 UDP-only DDS 配置，地图持久挂载 `/opt/feedbot-maps:/maps`。不设置自动重启或自动保存。

- `GET /api/save`：最近请求的 idle/saving/saved/failed 状态、错误与文件校验结果。
- `POST /api/save`，JSON `{"confirm_save":true}`：异步提交一次保存，返回202；保存中重复提交返回409。
- `GET /api/maps/<id>/feedbot.yaml` 或 `feedbot.pgm`：下载结果，两个文件放在同一目录。

保存使用 `nav2_map_server map_saver_cli`、`save_map_timeout:=15.0`；此等待窗口已在 run28 验证，不改变驱动200ms阈值。每次创建唯一目录、留存 `save.log`；CLI成功后检查PGM长度及YAML引用，记录SHA256才显示已保存。失败不重试、不加载、不停止建图。服务重启时读持久状态；未结束的保存标为中断失败，不当成成功或自动恢复。只展示最近一次请求，不做地图管理。

首次部署需要启动车机保存容器并重启 **RK网页服务** 以启用代理路由；不要重启车机建图容器。已有 `20261123_run28` 地图已核验并登记为初始结果，没有覆盖。连接恢复后已完成部署及一次网页同接口保存/下载验证，证据见 `agv_workspace/acceptance/runs/20261123_mapping_manual_28/web_save/RESULT.md`。

离线验证：

```sh
python3 -m py_compile agv_workspace/dashboard/mapping/car_bridge.py agv_workspace/dashboard/mapping/rk_server.py agv_workspace/dashboard/mapping/map_store.py
python3 -m unittest discover -s agv_workspace/dashboard/mapping/tests -v
python3 agv_workspace/dashboard/mapping/tests/browser_fixture.py '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome'
```

地图保存自测需要 PyYAML（Jazzy 镜像已提供；本机可安装到独立测试venv）。各阶段实车证据以 `agv_workspace/acceptance/runs/` 为准；本网页不将数据就绪标为建图验收通过。Run28 命令行和网页同接口保存均已核验文件一致；网页保存项实车静态通过，地图质量和重载定位仍待验收。

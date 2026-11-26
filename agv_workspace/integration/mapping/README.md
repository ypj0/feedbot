# 建图与已知地图定位入口

AMCL 运动更新步长显式设为 `update_min_d=0.05m`、`update_min_a=0.05rad`。
run94 使用 run74 同一段真实 scan/odom/初始粒子，在原生 AMCL 库中逐项对照：默认
0.25m/0.2rad 将运动积累成大步预测；缩短更新步长增加真实激光约束，不修改 alpha、
测量模型或定位核验门限。这是更新节奏修正，不是噪声标定或实车运动通过；静止时仍不
自动更新，不增加强制刷新/重试。候选回放仍有短暂等待收敛，不能宣称持续导航授权。

新冷启动使用 `../bringup/run.sh` 的独立底层会话；启动上层前等待底层同轮 `sessionready.json`。`run.sh` 只连接已运行的 `feedbot-base`，不接管设备。停止建图/定位不停止底层，公共 frames 也不退出。宿主托管的单向依赖及恢复顺序见 `../bringup/README.md`。历史 `static_session.py` / `manual_session.py` 仍是组合验收入口，不要与独立底层并行使用。

模式launch只启动地图层，不启动雷达、底盘、IMU、EKF、规划、控制或自动运动。运行前必须已有 `/scan`、`odom -> base_link`、公共静态外参和系统时间一致的数据。本入口在建图模式只启动 `slam_toolbox`，定位模式只启动 AMCL 作为 `map -> odom` 的预期发布者；固定容器名仅防止经本入口同时启动两个模式，不能阻止外部节点发布同一变换。启动前检查已有发布者，运行时以 `/tf` 端点信息和实际 `map -> odom` 消息的发布者 GID 核验唯一来源；定位前必须先停止建图。

## 雷达外参来源

本阶段采用用户确认的方案 B：`base_link` 与其地面投影 `base_footprint` 重合，底层已有的 `robot_state_publisher` 从 `chassis.urdf` 发布原厂配置给出的 `base_link -> laser_link = (0.27, 0, 0.20), rpy=(0, 0, 0)`；固定 TF 不再单独启动多个 DDS 参与者。该决定取代 `mapping_preflight_01/COLLECTION.md` 中基于旧坐标定义换算出的 `z=-0.30` 结论；历史归档保持不改。原厂值与用户现场粗略观察的距轴心约 27 cm、距地约 20 cm 吻合，但不是精确实测；允许据此开始受控建图，不得据此宣称完成精确标定或实车建图验收。

已验收 LTR2 驱动固定发布 `header.frame_id=scan`。因此公共frames入口另外发布 `laser_link -> scan` 的身份变换；它只为同一个雷达坐标系提供名称别名，不是安装外参，也不替代上述六自由度实测。`static_stack.sh` 在传感器侧启动frames一次，建图与定位模式不再分别发布静态外参。独立使用 `run.sh` 前也须已有公共frames。

## 运行

两个模式使用同一个容器名 `feedbot-mapping-mode`。若另一模式仍在运行，Docker 会明确失败；入口不会停止它，也不会重试。

切换模式时，操作员必须先以 `Ctrl-C` 停止当前唯一容器并确认其退出，再启动另一个模式；不得并行运行建图和定位。

```sh
agv_workspace/integration/mapping/run.sh mapping
agv_workspace/integration/mapping/run.sh localization /absolute/path/map.yaml
```

所有节点设置 `use_sim_time=false`。建图使用异步 `slam_toolbox`，帧为 `map/odom/base_link`，扫描话题为 `/scan`；不修改雷达接收时间戳。定位显式加载给定地图 YAML，并由 lifecycle manager 激活 `map_server` 与 AMCL。

车机同机 DDS 统一使用 `config/dds_loopback.xml`：仅 UDPv4、仅 `127.0.0.1`，保持同步发布和既有话题 QoS。`ROS_LOCALHOST_ONLY=0` 与 `ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT` 让固定版本 RMW 使用 XML，不代表允许外网通信；网卡范围由 XML 限定。不要改回 `ROS_LOCALHOST_ONLY=1`：`rmw_fastrtps 8.4.4` 的该分支会额外添加 SHM，绕过单独设置 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 的意图。静态、手动、独立地图层及临时 CLI 容器必须使用相同环境与 XML 挂载；网页仍通过车机桥接，不把 DDS 暴露给 RK3588。

此配置针对已捕获的 Fast DDS SHM 打开端口路径持有 participant 发送锁并休眠 333ms 的机制，不修改串口协议、驱动超时或失败行为。无设备 run24 已确认无 SHM 线程、相关 UDP 套接字全部绑定回环、四路数据频率与间隔达标；原始整轮结果因 EKF 启动诊断 `No events recorded.` 未通过，历史记录不改写。证据见 `agv_workspace/acceptance/runs/20261123_udp_loopback_24/RESULT.md`。

用户随后确认区分初始化与运行期。仅当 EKF 输出频率诊断的消息为 `No events recorded.`、级别 2、两项事件计数均为 0，且诊断源时间戳早于首个已观测 filtered 输出时，归档为 `startup_diagnostics`（未就绪），不作为运行错误；完整原始诊断仍在 samples 中。首个输出之后再出现、没有任何 filtered 输出、其他节点错误、计数缺失或非零均不豁免。`all_streams_observed_monotonic` 与 `runtime_observed_s` 明确所有必需流首次到齐后的观察时长；频率、间隔、几何、协方差、非零指令与 TF 检查仍覆盖全部采样，不裁掉启动期的数据异常，也不以话题出现替代验收通过。

部署后的 run27 **实车静态通过**：15.004s 采样（全部流到齐后 9.992s），数据/TF/原厂恢复均通过，完整容器日志未复现 333ms 相关超时。此轮没有初始化诊断豁免。保留 IMU 14 次 handoff 拒绝和雷达绝对时间未标定告警；尚未验收人工建图、地图保存和重载定位。详见 `agv_workspace/acceptance/runs/20261123_udp_static_27/RESULT.md`。

建图启动由同一 launch 中的 `activate_mapping.py` 单次执行 configure → 确认 inactive → activate → 确认 active，不再依赖原厂 launch 的状态事件自动激活。完整转换结果输出到容器日志；服务失败、状态不符或超时均使激活进程和顶层 launch 非零退出，不重试。该路径同时用于独立入口、静态会话和手动建图会话。Jazzy 回归已验证成功激活与失败退出码传播；这不表示 IMU 历史超时已修复。

AMCL 激活后仍需操作员提供地图中的初始位姿。可在 RViz 使用 `2D Pose Estimate`，或显式发布一次 `/initialpose`；在初始位姿和真实扫描到达前，不得声称已经定位。

Run29 从运行中手推会话切换时，使用 `deactivate_mapping.py` 单次 deactivate→cleanup，核对inactive/unconfigured及3秒内没有新map→odom，然后启动独立定位容器。旧会话的静态TF进程继续作为唯一公共外参来源，不重启传感器；新的公共frames拆分只用于后续冷启动，本次不向运行中容器重复启动frames。不得直接kill旧mapping launch，它退出会触发整个旧会话停止。

`initialize_localization.py SEED_JSON` 仅发送一次初始位姿；`localization_probe.py SEED_JSON OUTPUT_JSON` 只读观察，不初始化、不运动。seed取自停稳时网页 `/api/map` 的robot字段；初始化协方差为0.25m²、0.25m²、(15°)²，是搜索分布而非实测精度。沿用旧SLAM估计不是独立真值。结束本次链路时先停定位容器，再按旧manual session退出流程恢复原厂，不能在AMCL仍运行时重新激活SLAM。

Run29后续用户已确认车体位置/朝向与实物一致、激光基本贴墙，并完成手推跟随验证；状态为**实车运动通过（保存、重载与基础手推定位跟随）**。精度未量化，AMCL后续x/y/yaw标准差约0.313m/0.321m/19.04°，不标收敛或自主导航通过。证据见 `agv_workspace/acceptance/runs/20261123_localization_29/post_push/RESULT.md`。

建图时由另一个终端用镜像内现成的 `map_saver_cli` 保存地图；输出目录必须显式挂载为可写，例如：

```sh
mkdir -p /var/tmp/feedbot-map
bundle_root=$(pwd)
/opt/feedbot-nav2/docker/docker -H unix:///run/feedbot-nav2-docker.sock run --rm \
  --name feedbot-map-saver --network host --ipc host \
  -e ROS_DOMAIN_ID=89 -e ROS_LOCALHOST_ONLY=0 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e RMW_FASTRTPS_PUBLICATION_MODE=SYNCHRONOUS \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/work/agv_workspace/integration/mapping/config/dds_loopback.xml \
  -v "$bundle_root:/work:ro" \
  -v /var/tmp/feedbot-map:/maps:rw --entrypoint /bin/bash feedbot-navigation:jazzy-20261122 \
  -c 'source /opt/ros/jazzy/setup.bash; exec ros2 run nav2_map_server map_saver_cli -f /maps/feedbot'
```

以上保存命令从资料包根目录执行，XML 随 `/work` 只读挂载。

定位前检查 YAML 引用的图像文件与 YAML 位于同一挂载目录。运行后用 `ros2 param get` 核实系统时间参数，并检查 TF 时间警告。LTR2 完整帧的接收时间戳配合正 `time_increment`，并不等同于首束激光的真实采集时刻；出现 TF extrapolation 时必须核查实际时间关系，不得先修改时间戳、容差或阈值掩盖问题。现场建图以人工遥控低速 `|vx| <= 0.3 m/s`、`|wz| <= 0.5 rad/s` 进行，并记录墙面直线度、直角接近 90 度和重复经过区域的重合情况。软件单测不构成静态实车、运动实车、外参实测或地图重载验收。

## 统一静态接管窗口

`static_session.py RESULTS_DIRECTORY` 是待主控审核后才可执行的实车静态入口。它复用冻结 supervisor 的事务锁、permit 与恢复流程，分时停止一次 `spirit.service`，在单一临时容器中启动候选 chassis、独立 IMU、EKF、已验收 LTR2 ROS2 节点和 async mapping。它不启动网页、`cmd_vel` 发布者、规划、控制或自动运动；底盘激活清零路径与窗口内 `command_nonzero_count=0` 均保存证据。

入口必须由宿主 transient unit 承载，使 SSH 断开后 Python `finally` 仍能执行，例如使用 `systemd-run --property=Type=simple --property=Restart=no --property=TimeoutStopSec=180`。每条外部命令已有超时，不叠加覆盖整轮的 120 秒运行限制，以免在恢复原厂时中断清理。不要使用宿主不支持的 `Type=exec`。会话先执行 LTR2 deactivate/cleanup，再反向停止容器栈，释放串口、2105 和 permit，最后恢复 `spirit` 并验证原厂 scan、wheel、driver 频率及端口/串口 owner。任一清理或恢复错误均使会话失败。

静态 probe 的固定接口为 `static_probe.py OUTPUT_JSON DURATION_SECONDS`，本入口传入 15 秒。会话同时保存 C++ `tf_edge_probe` 的实际边/GID、`/tf` 与 `/tf_static` endpoint 信息和 `view_frames`。该窗口只验证真实静态数据、TF/端点所有权与恢复，不等于运动建图或定位通过。LTR2 继续从 `/opt/feedbot-ltr2/releases/20261122-ros2-v1` 只读复用；不得把旧 `tests/trial.py` 当常驻服务。

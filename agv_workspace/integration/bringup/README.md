# 独立底层会话

`session.py` 独占底盘、IMU、雷达，启动 EKF 与公共静态 TF。它不启动 SLAM、AMCL、网页桥接或运动指令发布者。`stack.sh` 的退出组只包含这些底层进程；建图/定位进程退出不在该退出组内。

共享 `ownership.py` 保留原来的原厂交接、LTR2 permit、生命周期清理和恢复验证；`lifecycle.py` 保留一次启动、失败报错及 finally 恢复。旧静态/手推建图入口改为复用这些逻辑，但仍是历史组合入口，不用于新的独立会话。所有入口共用 `/run/feedbot-mapping-static` 事务锁，目录名为兼容既有所有权记录保留，不能同时运行。

## 部署契约

- 在同一版本目录提供 `install/` 下的底盘和 IMU 安装产物、`build/tf_probe/tf_edge_probe`、本资料包中既有 supervisor/validation 工具和配置。
- LTR2 继续只读使用 `/opt/feedbot-ltr2/releases/20261122-ros2-v1`，解析不变。
- 加载 `../security/feedbot-serial-audit.apparmor`；容器使用 host PID、SYS_PTRACE、专用 enforced profile，保持设备白名单和冻结的回环 DDS 配置。
- 容器提供 `--ulimit rtprio=50`，使controller_manager能设置实际FIFO50线程；不添加SYS_NICE或privileged。run38隔离实控制器对照已验证，实车调度与抖动验收仍待完成。
- 公共固定 TF 统一由底层已有的 `robot_state_publisher` 从 `drivers/chassis_ros2/config/chassis.urdf` 发布，不再启动三个独立静态 TF 进程。`base_link→imu_link`、`base_link→laser_link`、`laser_link→scan` 数值不变，发布者改为 `robot_state_publisher`；雷达外参仍为原厂配置、未精确实测。建图/定位及 EKF launch 不再提供固定 TF，依赖完整底层会话；单独启动 EKF 的调用方须提供相同机器人描述。部署必须同步安装 URDF 与 chassis launch，再启动新 stack，不能混用旧安装产物。

## 运行与退出

在车机已授权接管窗口内执行，结果目录必须全新：

```sh
bash agv_workspace/integration/bringup/run.sh /var/tmp/feedbot-base-NEW_RUN
```

宿主 `feedbot-base.service` 不自动重启。systemd-run 返回成功只表示托管成功；必须看到结果目录的 `sessionready.json`，并核对同轮 `base_probe.json`、TF 端点和原始帧证据，才表示本轮底层检查完成。SIGTERM 走 LTR2 deactivate/cleanup、进程退出、串口/端口释放、恢复 spirit.service 和原厂数据流验证。SSH 断线不终止宿主托管，也不触发再次激活。

底层及上层临时服务使用 `--collect`，结束后释放unit名称，失败仍保留journal和会话报告。已生成最终 `session.json` 的会话以该报告判定完成和恢复结果，不再对已卸载unit重复执行stop；没有最终报告的运行会话仍按原路径停止。unit卸载不表示验收通过。

上层沿用 `mapping/run.sh`，启动前要求 `feedbot-base` 容器运行。通过 systemd 托管上层时明确单向依赖，使停止底层时先停止消费者；停止消费者不停止底层：

```sh
bash agv_workspace/integration/mapping/start.sh mapping
systemctl stop feedbot-map-layer
```

确认上层已退出再启动 localization，传入已有地图 YAML；两个模式仍共用 `feedbot-mapping-mode` 容器名，不同时运行。不得为切换模式重新启动底盘、IMU、雷达、EKF 或公共 TF。底层停止使用 `systemctl stop feedbot-base`；直接前台运行而未采用上述依赖的上层，必须由操作员先停止。

`start.sh` 的 ExecStop 显式停止唯一上层容器，`run.sh` 将容器 StopSignal 定为 SIGINT，让 ROS launch 执行退出流程。不能依靠 systemd 向 Docker CLI 发送 SIGTERM 就认定容器已经退出。run35实车验证托管停止1.147秒完成，底层PID不变、数据与TF继续更新，随后原厂恢复通过；地图发布补证因取证脚本错误未完成，不能把退出通过写成整轮通过。见 `../../acceptance/runs/20261123_base_lifecycle_35/RESULT.md`。

底层 probe 与原建图 probe 共享采样及验证实现，底层只移除 `/map` 和 `map→odom` 要求；传感器频率、间隔、协方差、零命令、静态外参、`odom→base_link` 和系统时间要求不放宽。额外 `require_map` 是内部验收范围，不是新增用户模式或容错开关。

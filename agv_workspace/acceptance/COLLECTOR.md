# ROS1 只读采集器

`collect_ros_readonly.py` 在原厂 ROS1 图上只订阅已有话题，生成有界 JSON 观测报告。它不发布消息，不调用服务，不写参数，不打开串口或网络硬件，不执行机器人命令。

## 运行

在 Ubuntu 18.04 / ROS Melodic 已配置的 shell 中：

```sh
python2 agv_workspace/acceptance/collect_ros_readonly.py \
  --duration 20 \
  --output /tmp/feedbot-readonly-observation.json
```

`--duration` 范围是 1–120 秒。默认订阅 `/scan`、`/imu/data`、`/wheel_odom`、`/ascamera_hp60c/depth0/image_raw`、`/ascamera_hp60c/rgb0/image`、两个对应 `camera_info` 话题和 `/driver_info`。所有话题都有对应 `--*-topic` 覆盖选项；覆盖前应先用只读 ROS 图证据确认实际名称。

两种 ROS 采集工具的输出路径必须不存在；先序列化至同目录临时文件，再通过硬链接原子提交，拒绝覆盖已有证据。目标文件系统须支持硬链接（当前 Mac/Linux 本地文件系统适用）。

采集器不默认订阅 `/tf` 或 `/tf_static`。TF 流量、内存与语义需要单独有界采证，不应与本轮传感器内容检查混合。

## 报告边界

报告中每个话题状态只能是 `observation_ok`、`invalid`、`no_data` 或 `error`。`observation_ok` 仅表示已收到的样本通过结构和有限值检查，不表示精度、标定、安装、长期稳定性或硬件已验收。`all_hardware_accepted` 始终为 `false`。

频率和最大接收间隔使用本机单调时钟。Header 年龄假设 ROS header 为 Unix epoch 且采集主机时钟已同步，仅作观测；回退、重复、未来时戳和缺失时戳都单独计数，不会被伪装为通过。图像只保留尺寸、编码、step 和数据长度统计，不写入原始像素。`driver_info` 只保留预定义数值字段的最小/最大值。

## 离线测试

```sh
python3 -B agv_workspace/acceptance/test_collect_ros_readonly.py
python2 -B agv_workspace/acceptance/test_collect_ros_readonly.py
```

离线测试不需要 `rospy`，覆盖正常样本、雷达长度错误、NaN、时间倒退、无数据、缺标定和图像数据长度错误。本机若没有 Python 2，只能在目标 ROS Melodic 主机上补跑 Python 2 兼容性验证。

## 其他验收入口

- `run_offline.py --output runs/<新目录>`：统一运行离线协议、生命周期、恢复模型和采集工具测试；使用原资料的只读输入，构建输出位于临时目录。
- 可选 `--differential-python /path/to/venv/bin/python`：在结果目录中的副本执行原二进制差分，依赖 Unicorn 与 pyelftools。不传参数时差分标为 `not_run`，总结果不冒充全部通过。
- `capture_lidar_passive.py --duration 10 --output /tmp/lidar-new.bin`：仅 Linux AF_PACKET 被动抓取来自 `192.168.10.160` 的 UDP 2105→2105；需原始套接字权限，不发包、不占 UDP 控制口。零包只说明窗口内该过滤条件无数据，不代表硬件坏或测试通过。
- 全部工具测试：`python3 -B -m unittest discover -s agv_workspace/acceptance -p 'test_*.py' -v`。采集退出 0 仅代表报告生成；必须检查逐话题状态。
- `probe_sensor_content.py --duration 7 --output /tmp/content-new.json`：依赖同目录 `collect_ros_readonly.py`，ROS1 中只订阅；检查扫描内容指纹、单张 RGB/16UC1 内容和传感器至 `base_footprint` 的最新可用 TF。可用 `--base-frame` 更改目标。单张内容的时戳与最后接收时戳分开；不会证明物理新鲜度或 TF 标定。当前车机 7 秒观测及 Python 2 测试见 `runs/20261122_live_02/`；组合 SSH 断连失败与后来成功取回报告分别记录。

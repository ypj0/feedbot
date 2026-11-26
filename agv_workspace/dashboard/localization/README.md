# 独立定位只读网页

该入口只订阅地图、激光、TF、定位核验和底盘诊断，不创建运动发布者，不提供POST、运动、电机释放、保存或原厂地图加载接口。不启动旧manual_session或mapping网页控制Runtime。

车机Jazzy中运行 `FEEDBOT_TEST_TOKEN_FILE=/key/token python3 -B /web/car_bridge.py`，监听10.7.5.88:8098。RK运行同目录`rk_server.py`，使用已有`FEEDBOT_TEST_TOKEN_FILE`，浏览器经ADB转发访问127.0.0.1:8088。两端GET /api/state只读；HTTP断线清除页面实时位姿，静态地图允许保留。

独立base就绪后，用既有mapping/start.sh localization MAP_YAML启动map_server与AMCL，不同时运行SLAM。本次移动后未知初始位姿时，在确认AMCL active、其自身已收到地图、激光及odom/base TF可用后，主控可以单次调用 `/reinitialize_global_localization`（std_srvs/srv/Empty）。它重建全地图粒子，不发送运动；服务成功不代表位置已正确。后续需要新扫描、必要的手推运动与现场几何确认，不重试、不复用旧seed、不自动发送目标。

`robot_tf`为新鲜TF当前位姿；`amcl`来自核验器一次输出的原始后验、源时间戳、全部36项协方差和核验结果，不再独立订阅AMCL后拼接。地图或发布源变化由核验器清除旧配对，网页不沿用旧位姿；地图变化也清除旧扫描/TF显示及待配对扫描。静止时允许展示当前地图/发布源的最后配对后验，年龄使用`observation_age_s`，不被核验心跳刷新。`data_ready`仅表示显示数据到齐；`tracking_valid`仅在同帧核验新鲜且通过时为true，等待、未收敛、拒绝、过期时为false。`localization_accepted`与`motion_authorized`保持false，不构成运动授权或实车验收。扫描/当前TF/核验消息沿用0.5s接收新鲜度，不改变驱动200ms超时。

停止顺序：只读网页与定位消费者先停，再停base并执行既有原厂恢复验收。网页不接管串口、不负责底层恢复，也不携带旧位姿恢复逻辑。

扫描显示使用独立配对状态：选中一帧后，等该帧源时间戳的TF可用再转换，等待不阻塞TF订阅，也不被更新扫描覆盖。等待期间未选中的扫描计入`scan_pairing.skipped_display_scans`，只是网页抽样，不改变雷达发布或定位输入。接收与源时间0.5秒有效期保持不变；原成功显示不会刷新接收时间，真正过期仍清除并报错。禁止以最新TF替代该帧时刻TF。

本地及无设备Jazzy逻辑检查：`python3 -B -m unittest discover -s agv_workspace/dashboard/localization -p 'test_*.py' -v`。
浏览器检查：`python3 -B agv_workspace/dashboard/localization/browser_fixture.py '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome'`。
实车状态以`acceptance/runs/20261123_handpush_localization_47/`为准，代码自测不等同实车定位通过。

配对修正见`acceptance/runs/20261123_web_scan_pairing_49/STATUS.md`：网页同步实车静态通过，不代表定位已收敛。实际tf2隔离测试为`tf_pairing_fixture.py`，仅在无设备、独立网络/ROS域执行。

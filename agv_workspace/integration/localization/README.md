# 独立定位后验核验（第一阶段）

## 当前跟踪状态（run96，代码验证候选）

`/localization/convergence.tracking` 是同一生命周期节点持续计算的当前状态，与顶层
`phase/requests/localization_accepted/input_error` 所描述的静止获取/刷新流程分开。
历史 `succeeded` 不证明当前定位有效；正常轮速不再使跟踪状态失效，但仍禁止在运动中
进行静止收敛。跟踪直接接纳当前地图/AMCL发布端的新鲜已核验后验，不要求本进程发送过初值。

`tracking.localization_ready` 只表示当前定位输入与核验条件满足：scan、IMU、wheel、EKF
的接收/源时间和诊断/核验心跳沿用0.5秒窗口，后验源时间沿用2秒窗口。静止等待或协方差
未收敛为 `hold`；输入错误、几何拒绝、地图/发布端改变或底盘故障计数变化为 `invalid`。
失效后必须收到失效时刻之后的新后验；不会用同一条旧成功结果恢复。非零速度命令计数
不作为持续跟踪故障，但静止收敛仍检查它。新增IMU接收观测避免EKF持续预测掩盖IMU中断。

`tracking.stamp_ns` 是当前状态计算时刻，不替换 `observation_stamp_ns`；消费者也必须
检查状态消息新鲜度。`motion_authorized` 始终为false，本步没有导航任务、速度出口或目标
恢复执行者，不调用自动刷新/重定位，不修改几何、协方差、超时或协议。停止后不保留授权。
纯逻辑测试、`tracking_ros_fixture.py` 与run95记录回放用于代码验证，不代替候选版实车验收。

只订阅map、scan、amcl_pose和TF，发布`/localization/verification`。不发送initialpose、运动目标或服务请求，不调用全局搜索：`search()`始终收到AMCL后验和同消息协方差。网页只展示核验结果，不拥有核验生命周期。

现有后验不需要由本进程发送初值才可核验。首张地图迟到保留当前配对；实际地图内容变化递增代际、清空旧配对和核验结果，迟到的旧代际计算不能回写。扫描和后验必须源时间戳完全一致，使用同消息位姿与36项协方差中的x/y/yaw子矩阵；不与当前TF混合。旧版2秒同帧证据窗口保持不变。

`/localization/verification.posterior`携带该次已完成核验的原始AMCL位姿、源时间戳和完整36项协方差，与结果原子更新、共同清除；不是搜索得到的候选位姿。网页直接使用此配对，不再对异步原始AMCL做第二次拼接。`observation_age_s`表示后验年龄，核验消息心跳不会刷新它。

核验同时绑定`/amcl_pose`的唯一发布端GID。后验到达及既有200ms核验周期会读取ROS图；观察到发布端消失、多发布端或GID更换时，清空旧配对/结果并递增独立的`posterior_generation`，旧异步计算不能回写。重新出现的发布端必须提供源时间晚于这次变更观测的新扫描/后验配对；首次发现唯一发布端仍允许接纳其新鲜保留消息。地图代际不因此伪增。失效时刻受DDS发现及核验周期影响，不承诺在进程退出瞬间检测，也不覆盖保留同一发布端的AMCL内部生命周期变化或无里程计的搬车。

几何运算在独立进程的单工作线程中执行，不占用底盘/IMU进程。输入错误与异步计算结果分别管理，旧计算成功不能覆盖新的输入错误。核验过期或静止等不到新后验标为waiting_pair，而不是自动判丢定位；本阶段不自动请求静止更新。

`pair_verified`只表示这一帧符合旧核验判据；`localization_accepted`仍为false，因为连续跟踪、恢复状态机和运动授权尚未接入。不能把单帧通过或协方差下降称为导航可用。网页同时检查核验消息接收时间及地图摘要，旧地图结果不得用于新地图显示。

## 复用来源

以下文件与`/Users/ypj/feedbot/navigation_runtime/replacement/native_ros/`原文件逐字一致，未改算法：

| 文件 | SHA256 |
| --- | --- |
| global_localization.py | 54a6f21ce60af46a0ef8a73a9d960cc05139b8823f28687177e0238d0dca2a7f |
| localization_measurement.py | e71cc2aa62ef357f2e66a603e04a09a3b6d4e1e742075c07dc649063237b9589 |
| localization_posterior.py | f18f23256741de6b3f769fdcba800c5944a0fe8ea8fac1bbf61cc9516ea786bd |
| localization_observation.py | e045eaf00dd49f9c6b88445014a2f44d21806e3a345e8f4d12d7748f13de33b6 |
| test_global_localization.py | 0a1a04220047eda2b6f8aa4bbf4469ba4eee1bdf11aecae5fbedc1b396260d5c |

旧射线模型假设雷达平面外参x=0.27m、y=0、yaw/roll/pitch=0。适配层在TF输入边界核对这个已冻结配置，不静默将该模型用于不同外参。z不参与二维模型。依赖Jazzy现有numpy，不增添驱动或旧NativeState/TaskLease依赖。旧原厂恢复、自动全局搜索、重复服务请求和局部修正发布均未搬入。

## 运行和验证

在现有base和map-layer会话内，车机执行本目录`bash start.sh`。服务绑定这两个会话，`Restart=no`；停止只读核验不会停止底层或AMCL。DDS复用v7 UDP回环配置。

`python3 -B -m unittest discover -s agv_workspace/integration/localization -v`运行旧几何回归及新配对/地图代际/错误时序测试。`ros_fixture.py`必须在无设备、独立网络和ROS域的Jazzy容器执行，不得向生产域注入模拟map/scan/AMCL。

## 有界静止测量生命周期

`lifecycle_node.py`是独立动作拥有者，核验器`observer.py`保持只读。它不随网页刷新、消息过期或进程启动自动请求更新。运行`bash start_lifecycle.sh`后，在输入就绪且车体静止时显式调用一次：

```sh
ros2 service call /localization/converge std_srvs/srv/Trigger '{}'
ros2 topic echo /localization/convergence
```

首次初始化已有当前地图下几何兼容后验、但仍需收敛时，显式调用`ros2 service call /localization/acquire std_srvs/srv/Trigger '{}'`。该入口最多运行30秒、核验通过即结束；原`/localization/converge`仍保持10秒/10次刷新。两入口共用一个动作拥有者，运行中拒绝重复启动，不自动切换或重试。获取入口不执行搜索、不发送initialpose，不解决没有兼容后验的未知位置问题。状态`operation`为`acquire`或`refresh`，`budget_s`记录固定预算；启动前两者为null，不提供任意预算配置。

Trigger回复成功仅表示接受这次流程，必须看`/localization/convergence`最终`phase`和`reason`。`succeeded`只代表本次同帧测量核验通过，`localization_accepted`始终为false；终态及其时间戳是历史流程结果，不代替实时定位有效性。正式导航运动授权尚未实现。

刷新最多10秒/10次，首次获取最多30秒，不受刷新10次上限截断；两者相邻`/request_nomotion_update`请求均至少1.01秒。前一次服务回复且获得新的同帧后验核验后才进入下一次。发送前以`max(上次确认后验时间戳, 当前已知核验后验时间戳)`记录后验水位，后续核验必须严格晚于该水位，并处于新鲜的`verified_pair`或`pending_convergence`阶段；不能重复计入已知后验。已核验通过立即结束；请求/新核验1秒未完成、输入异常、运动、地图代际改变、故障或非零指令计数变化立即失败，不自动重试或重新开始。截止前已发送的Empty请求不能撤回，停止只取消本地等待，不伪称远端已取消。

这个流程证明的是阶段内有新的有效后验进展，不声称每条后验由某次Empty请求单独造成。AMCL的Empty服务只置位`force_update_`，没有消费扫描序号；独立进程接收raw scan的先后不能充当AMCL处理顺序。请求前尚未观察到、到达时仍新鲜且同图/同源/同帧核验通过的后验可以推动阶段进展。run65已实车复现旧raw-scan水位误拒绝等stamp新后验；修正未放宽新鲜度、几何/协方差或超时门限，不增加请求次数。

运行中的接续流程也绑定`posterior_generation`和发布端GID；变更时失败为`posterior_source_changed`，不把另一个AMCL实例的新消息当作旧请求的完成。核验器与接续节点需使用同版消息字段，旧版消息缺字段显式报错，不回退旧协议。终态仍是历史流程结果，不代替当前核验状态。

请求边界观测字段：`request_scan_stamp_ns`仅记录发送时本进程已收到的最新扫描源时间，不作为后验进展门限；`request_posterior_stamp_ns`是实际后验水位，`request_sent_monotonic_s`是本机单调发送时刻。`request_pending`、`request_acknowledged`保留状态机原值，失败后不表示仍在发请求。`observed_pair_stamp_ns`、`observed_scan_stamp_ns`和`observed_verification_phase`是发布状态时本进程实际读到的核验视图，输入失效时为null。观测字段已在run65用于定位错误分支；后验水位修正当前仅代码通过，未实车验收。

入口只接纳当前地图下几何兼容、等待协方差收敛的后验，或已有通过结果；`waiting_pair`的旧兼容结果只允许启动一次新扫描刷新，不作为当前位置可信或导航授权依据。完全没有兼容位置线索、几何拒绝、未知位置重定位不由这个流程解决。服务不发送initialpose、全局定位或运动指令，不修改参数/协方差/核验门限。

scan、wheel odom、filtered odom需接收和源时间新鲜（0.5秒）；诊断及核验接收新鲜。沿用静止判据，各里程计速度分量绝对值≤0.01且有限。底盘故障/非零命令计数与入口基线比较，历史运动计数不被误当本轮运动。服务绑定base、map-layer与verifier生命周期，Restart=no，停止新服务不停止已有父服务。当前部署需显式启动此服务，不重启底层或AMCL。

`test_convergence.py`验证状态转移与边界；`convergence_ros_fixture.py`仅在无设备、独立网络/ROS域的Jazzy容器运行，覆盖真实Trigger/Empty接口、多次更新成功与只有服务回复而无后验时失败。run52提供实车静止测量收敛证据，不等于这一生产实现已通过运动验收。不实现局部修正发布或无限自动恢复。

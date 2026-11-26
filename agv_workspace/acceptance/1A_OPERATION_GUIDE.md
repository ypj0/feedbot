# Phase 1A 实车验收操作指南

## 前置要求

**硬件**：
- 车机已启动，网络连接正常
- 底盘、IMU、雷达硬件都已连接
- 建议准备：5m 卷尺、90° 角度标记物、平整测试地面

**软件**：
- 本地开发机能 SSH 连接到车机
- 车机已安装最新的 agv_workspace (Slice 1-9)
- ROS 2 Jazzy 环境已配置

**人员**：
- 2 人操作（一人控制命令，一人观察车辆）
- 熟悉 ROS 2 基本操作

---

## 步骤 0：启动基础栈

### 车机端

```bash
# 进入车机
ssh feedbot@10.7.5.88

# 启动基础栈（会启动所有驱动 + MotionAuthority）
cd /work
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch feedbot_bringup base.launch.py
```

**预期输出**：
- 无致命错误
- MotionAuthority 节点正常启动消息
- `/motion/acquire_lease` service 可用

### 本地端（验证连接）

```bash
# 验证基础栈是否运行
ros2 node list | grep -E "motion_authority|imu|chassis|ekf"

# 验证服务可用
ros2 service list | grep motion

# 验证主题流
ros2 topic list | grep -E "scan|imu|wheel/odom|odometry/filtered"
```

**预期**：
- 至少 5 个节点活跃
- 3 个 service 显示：`/motion/acquire_lease`, `/motion/clear_fault`, `/motion/release_lease`

---

## 步骤 1A：Static Window（车辆静止，60 秒）

### 准备

```bash
# 本地开发机
cd agv_workspace/acceptance

# 创建结果目录
mkdir -p /tmp/acceptance_1b_static_$(date +%Y%m%d_%H%M%S)
RESULT_DIR="/tmp/acceptance_1b_static_$(date +%Y%m%d_%H%M%S)"
```

### 运行采集

```bash
# 本地开发机，连接到车机 ROS 2 网络
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

python3 1b_static_window.py $RESULT_DIR
```

**采集过程中**：
- 确保车辆**完全静止**
- 不要发送任何运动命令
- 让脚本运行 60 秒

**预期输出**：
```json
{
  "passed": true,
  "failures": [],
  "samples": {
    "scan": 900,      // 60s × 15 Hz
    "imu": 12000,     // 60s × 200 Hz
    "wheel": 3000,    // 60s × 50 Hz
    "filtered": 3000  // 60s × 50 Hz
  }
}
```

### 检查结果

```bash
cat $RESULT_DIR/1b_static_result.json | jq .

# 如果失败，查看具体失败项
cat $RESULT_DIR/1b_static_result.json | jq .failures
```

**通过条件**：
- `"passed": true`
- `failures` 列表为空

**常见失败**：
| 失败项 | 原因 | 修复 |
|--------|------|------|
| `non-monotonic scan timestamps` | NTP 时钟跳跃 | 禁用 NTP 或确保 steady_clock 注入 |
| `non-monotonic imu timestamps` | 同上 | 同上 |
| `rate outside range: scan` | 雷达 USB 有丢包 | 检查 USB 连接或雷达 IP 配置 |
| `diagnostic ERROR` | 某模块异常 | 查看 `diagnostics` 主题内容 |
| `insufficient samples` | 对应主题未收到数据 | 检查该模块是否正常运行 |

---

## 步骤 1B：Support Window（轮离地，手动遥控）

### 准备

```bash
# 物理准备：
# 1. 车辆放在支架上或升起，轮离地（不要碰到地面）
# 2. 轮子转动时不会伤人，准备安全区域
# 3. 准备停止开关（SIGTERM 或类似）

RESULT_DIR="/tmp/acceptance_1b_support_$(date +%Y%m%d_%H%M%S)"
mkdir -p $RESULT_DIR
```

### 运行

```bash
python3 1b_support_window.py $RESULT_DIR
```

**交互提示**（脚本会引导）：

1. **左转验证**
   ```
   提示：Command LEFT TURN for 3s, press ENTER
   
   操作：按 ENTER 后，脚本发送左转命令（角速度 +0.5 rad/s）
        观察左右轮是否向左旋转（逆时针看）
        等待 3s，脚本自动停止
   
   验证：/wheel/odom.y 应该增加（向左偏移）
   ```

2. **租约释放验证**
   ```
   提示：Release lease, then command without lease, press ENTER
   
   操作：脚本释放租约，尝试发送前进命令
   
   验证：车应该保持静止（不响应命令）
   ```

### 检查结果

```bash
cat $RESULT_DIR/1b_support_result.json | jq .
```

**通过条件**：
```json
{
  "passed": true,
  "errors": []
}
```

**常见问题**：
| 问题 | 检查 |
|------|------|
| 左转时轮反向旋转 | MCU PWM 极性配置，可能需要硬件反接或软件反号 |
| 释放租约后仍有运动 | MotionAuthority 节点是否正确接收 ReleaseLease 请求 |
| 没有收集到里程计数据 | /wheel/odom 是否有发布，检查差分驱动控制器 |

---

## 步骤 1B-Ground Window（地面上，已知距离）

### 准备

```bash
# 物理准备：
# 1. 在平整硬地面（如停车场）标记**直线起点线**（粉笔或胶带）
# 2. 用卷尺测量 5m，标记**终点线**
# 3. 标记 **90° 转角**（如用两条 5m 线段形成 L 形）

# 位置建议：长直走廊或停车场大空地，避免障碍物

RESULT_DIR="/tmp/acceptance_1b_ground_$(date +%Y%m%d_%H%M%S)"
mkdir -p $RESULT_DIR
```

### 运行

```bash
python3 1b_ground_window.py $RESULT_DIR
```

**交互流程**：

1. **5m 直线**
   ```
   提示：Mark start line, press ENTER to begin
   
   操作：
   - 将车摆在起点线上
   - 确保车头朝向 5m 终点线方向
   - 按 ENTER，脚本发送 0.1 m/s 前进命令（约 50 秒）
   - 车应稳定前进 5m
   - 脚本停止时，立即标记车停止位置（可用粉笔）
   
   验证：
   - 实际停止位置 vs 预期终点线
   - 脚本计算里程计偏差百分比
   ```

2. **90° 转角**
   ```
   提示：Command 90° at 0.2 rad/s
   
   操作：
   - 将车摆在直线终点
   - 确保车头朝向标记好的转角方向
   - 按 ENTER，脚本发送 0.2 rad/s 转角命令（约 7.9 秒）
   - 车应在原地旋转 ~90°
   - 标记最终车头朝向
   
   验证：
   - 实际车头朝向 vs 标记的 90° 位置
   - 脚本计算角度偏差百分比
   ```

### 检查结果

```bash
cat $RESULT_DIR/1b_ground_result.json | jq .
```

**通过条件**：
```json
{
  "passed": true,
  "distance_error_pct": 1.5,  // < 2%
  "angle_error_pct": 1.2,     // < 2%
  "errors": []
}
```

**失败处理**：
- 如果误差 2-5%：可接受，记录证据，考虑进行 1C 多次标定
- 如果误差 > 5%：
  - 检查地面是否平整（斜坡会影响直线）
  - 检查轮径/轮距配置是否正确
  - 检查 MCU 是否有反馈延迟

---

## 步骤 1C：Calibration（如需，多次统计）

### 仅在 1B-Ground 误差 2-5% 时建议运行

```bash
RESULT_DIR="/tmp/acceptance_1c_calibration_$(date +%Y%m%d_%H%M%S)"
mkdir -p $RESULT_DIR

python3 1c_calibration.py $RESULT_DIR
```

**流程**：
- 运行 3 次 5m 直线试验
- 每次标记起点和终点
- 脚本计算平均误差

**通过条件**：平均误差 < 1%

---

## 验收结果保存

### 保存本地副本

```bash
# 从车机复制结果（如果脚本是在车机上运行）
scp -r feedbot@10.7.5.88:/results/1b_* ./acceptance/runs/$(date +%Y%m%d)/
scp -r feedbot@10.7.5.88:/results/1c_* ./acceptance/runs/$(date +%Y%m%d)/

# 或本地运行的话，复制本地结果
cp -r /tmp/acceptance_1b_* ./acceptance/runs/$(date +%Y%m%d)/
cp -r /tmp/acceptance_1c_* ./acceptance/runs/$(date +%Y%m%d)/
```

### 整理成验收报告

```bash
cat > ./acceptance/runs/$(date +%Y%m%d)/RESULTS.md << 'EOF'
# Phase 1A 验收结果 - $(date)

## 1B-Static Window
$(cat /tmp/acceptance_1b_static_*/1b_static_result.json | jq .)

## 1B-Support Window  
$(cat /tmp/acceptance_1b_support_*/1b_support_result.json | jq .)

## 1B-Ground Window
$(cat /tmp/acceptance_1b_ground_*/1b_ground_result.json | jq .)

## 1C-Calibration (if applicable)
$(cat /tmp/acceptance_1c_*/1c_calibration_result.json | jq .)

## 总体状态
- [ ] 1B-Static: PASS
- [ ] 1B-Support: PASS  
- [ ] 1B-Ground: PASS (误差 < 2%)
- [ ] 1C-Calibration: PASS (if run, 误差 < 1%)

**验收时间**：$(date)
**操作员**：___________
**签字**：___________
EOF
```

---

## 故障排查

### 如果基础栈无法启动

```bash
# 检查 MotionAuthority 节点是否有错误
ros2 launch feedbot_motion_authority_node motion_authority.launch.py

# 检查契约是否加载成功
# （应该有类似 "Contract loaded: 50.0 Hz, lease=300 ms, ..." 的日志）

# 检查接口包是否编译
ros2 interface list | grep feedbot_motion_interfaces
```

### 如果某个流（scan/imu/wheel）缺失

```bash
# 检查该模块节点是否运行
ros2 node list

# 检查该主题是否发布
ros2 topic echo /topic_name --no-arr  # 1 条消息后 Ctrl+C

# 检查诊断
ros2 topic echo /diagnostics
```

### 如果里程计误差很大（> 10%)

```bash
# 1. 检查轮径/轮距配置
grep -r "wheel_radius\|wheel_separation" agv_workspace/drivers/chassis_ros2/config/

# 2. 检查地面（斜坡会导致偏差）
# 3. 检查轮子是否滑动（低速/高速对摩擦力的影响）
# 4. 重新校准（进行 1C）
```

---

## 验收通过后

1. **提交证据**：将验收结果复制到 git 仓库
   ```bash
   git add acceptance/runs/$(date +%Y%m%d)/
   git commit -m "test: Phase 1A acceptance results - [date]"
   ```

2. **启动 Phase 2**：准备定位模块（AMCL + 初值方案）

3. **后续测试**：
   - 整合高级模块（导航、回充等）
   - 进行长时间耐久性测试

---

## 技术支持

如有问题，检查：
1. MotionAuthority 节点日志
2. 契约检查器输出：`python3 contract/check_contract.py agv_workspace`
3. ROS 2 工具诊断
4. 本地/远程 Git 记录了所有改动

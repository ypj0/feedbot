# Phase 1A 实车验收清单

## 前置条件
- [ ] Vehicle powered on, base stack running: `ros2 launch feedbot_bringup base.launch.py`
- [ ] Motion Authority node active: `/motion/acquire_lease` service responding
- [ ] No other motion clients running (dashboard/navigation disabled)
- [ ] Lidar spinning (/scan ≈15 Hz)
- [ ] IMU responding (~200 Hz)
- [ ] MCU connected and responding

## 1B-Static Window（车辆静止，30-60秒）

### 检查项
- [ ] `/scan` 频率 14-16 Hz，时间戳严格单调递增
- [ ] `/imu` via `/joint_states` ≈200 Hz，时间戳严格单调递增  
- [ ] `/wheel/odom` ≈50 Hz，时间戳严格单调递增
- [ ] `/odometry/filtered` ≈50 Hz，时间戳严格单调递增
- [ ] TF `odom→base_link` 单一发布者（ekf_filter_node）
- [ ] 诊断无 ERROR 级别报告（启动时期望的 EKF 预热消息除外）
- [ ] 无 MCU 命令非零计数
- [ ] /scan 几何一致（1075 个光束，角度增量正确）

### 运行
```bash
cd /work/agv_workspace/acceptance
python3 1b_static_window.py /results/1b_static
```

### 检查结果
```bash
cat /results/1b_static/1b_static_result.json
```

---

## 1B-Support Window（轮离地，手动遥控）

### 检查项
- [ ] 左转（角速度 > 0）时，/wheel/odom.y 增加（向左转）
- [ ] 右转（角速度 < 0）时，/wheel/odom.y 减少（向右转）
- [ ] 直线前进时，/wheel/odom.x 增加
- [ ] 无租约时，发送运动命令无效（/wheel/odom 速度保持 0）
- [ ] 释放租约后，车停止（即使继续发送命令）

### 运行
```bash
python3 1b_support_window.py /results/1b_support
# 按提示手动验证各项
```

### 检查结果
```bash
cat /results/1b_support/1b_support_result.json
```

---

## 1B-Ground Window（地面上，已知距离和转角）

### 准备
- 在平整地面标记起点线
- 准备 5m 直线（用标记或测量轮标出）
- 准备 90° 转角标记

### 检查项
- [ ] 直线 5m 命令：记录里程计误差 < 2%
- [ ] 90° 转角命令：记录里程计误差 < 2%
- [ ] 如果误差 < 2%：通过地面窗口

### 运行
```bash
python3 1b_ground_window.py /results/1b_ground
# 按提示标记距离和角度
```

### 结果处理
如果通过（误差 < 2%）：
```bash
# 保存结果
cp /results/1b_ground/1b_ground_result.json acceptance/runs/$(date +%Y%m%d)_1b_ground/

# 如需，更新校准参数（见 1C）
```

如果失败（误差 ≥ 2%）：
- 检查轮径/轮距配置是否正确
- 检查 /wheel/odom 是否有异常
- 检查地面是否平整
- 重试或进行 1C 校准

---

## 1C-Calibration（如需，多次试验统计）

### 运行
```bash
python3 1c_calibration.py /results/1c_calibration
# 进行 3 次 5m 直线试验，每次标记起点和终点
```

### 检查结果
```bash
cat /results/1c_calibration/1c_calibration_result.json
# 如果平均误差 < 1%，校准完成
# 否则，检查以下参数是否需要更新：
#   - agv_workspace/drivers/chassis_ros2/config/controllers.yaml: wheel_radius
#   - agv_workspace/drivers/chassis_ros2/config/chassis.urdf: wheel z 坐标
#   - agv_workspace/drivers/chassis_ros2/include/feedbot_chassis_ros2/session.hpp: Wheels::radius
```

---

## 验收通过条件

✓ 1B-Static：所有流单调、频率正常、诊断无异常  
✓ 1B-Support：方向正确、租约控制有效  
✓ 1B-Ground：距离/角度误差 < 2%  
✓ 1C-Calibration（可选）：平均误差 < 1%  

---

## 失败/阻挡处理

| 现象 | 检查项 | 修复 |
|------|-------|------|
| 时间戳非单调 | NTP 是否调整过时钟？MotionAuthority 是否运行？ | 重启，确保 steady_clock 注入 |
| 频率异常 | 网络丢包？CPU 过载？ | 检查网络/CPU 负载 |
| 诊断 ERROR | 哪个模块报错？ | 查看诊断消息内容，重启对应节点 |
| 轮转向错误 | PWM 极性反了？ | 检查 MCU 配置或硬件连接 |
| 租约无效 | AcquireLease 是否返回 granted=true？ | 检查 MotionAuthority 节点状态 |
| 里程计误差大 | 轮是否滑动？地面是否平整？ | 检查轮径/轮距配置，重新校准 |

---

## 保存证据

所有结果已自动保存至 `/results/1b_*/` 和 `/results/1c_*/`，包含：
- JSON 结果文件（通过/失败、数值误差）
- 时间戳、频率统计

运行后复制到本地档案：
```bash
# 在开发机上
scp -r vehicle:/results/1b_* acceptance/runs/$(date +%Y%m%d)_1b/
scp -r vehicle:/results/1c_* acceptance/runs/$(date +%Y%m%d)_1c/
```

---

## 下一步

如果 1A 验收通过：
- **Slice 1B**（已完成）：三窗口验收脚本
- **Slice 1C**（已完成）：多次校准验证
- **Phase 2**：定位模块（AMCL + 初值方案）
- **Phase 3**：回充模块
- **Phase 4**：建图模块
- **Phase 5**：导航规划 + 前向过滤

每个后续模块都复用 Phase 1A 的运动控制和契约基础。

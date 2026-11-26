#!/bin/bash
# 实车验证一键启动脚本（本地开发机运行）
# 用法：bash start_acceptance_local.sh <车机IP>

set -e

VEHICLE_IP="${1:-10.7.5.88}"
LOCAL_PORT=8765
REMOTE_PORT=8765

echo "=========================================="
echo "Phase 1A 实车验收 - 一键启动"
echo "=========================================="
echo ""
echo "🚗 车机 IP: $VEHICLE_IP"
echo "🌐 本地端口: localhost:$LOCAL_PORT"
echo ""

# 检查 SSH 连接
echo "🔍 检查车机连接..."
if ! ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new feedbot@$VEHICLE_IP "echo '✓ 车机已连接'" 2>/dev/null; then
    echo "❌ 无法连接到车机 $VEHICLE_IP"
    echo "请检查："
    echo "  1. 车机是否已启动（IP 是否正确）"
    echo "  2. 网络连接是否正常"
    echo "  3. 是否设置了 SSH key（无需输入密码）"
    exit 1
fi

echo ""
echo "🚀 启动车机服务（后台）..."

# 启动基础栈（后台）
echo "   启动 ROS 基础栈..."
ssh feedbot@$VEHICLE_IP "nohup bash -c 'cd /work && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch feedbot_bringup base.launch.py' > /tmp/base_stack.log 2>&1 &" 2>/dev/null
sleep 3

# 启动网页服务（后台）
echo "   启动网页服务..."
ssh feedbot@$VEHICLE_IP "nohup bash -c 'cd /work/agv_workspace/acceptance && python3 acceptance_web_backend.py' > /tmp/web_service.log 2>&1 &" 2>/dev/null
sleep 2

echo ""
echo "📡 建立 SSH 隧道..."
echo ""
echo "⏳ 正在连接... (请输入车机密码或直接回车如果配置了 SSH key)"
echo ""

# 建立 SSH 隧道（前台，保持连接）
ssh -L $LOCAL_PORT:$VEHICLE_IP:$REMOTE_PORT feedbot@$VEHICLE_IP

# 如果隧道断开
echo ""
echo "⚠️ SSH 隧道已断开"

#!/bin/bash
# 实车验收快速启动指南
# 在三个不同的终端中运行下面的命令

VEHICLE_IP="10.7.5.88"

echo "=========================================="
echo "Phase 1A 实车验收 - 快速启动"
echo "=========================================="
echo ""
echo "请在三个独立的终端中按顺序运行以下命令："
echo ""
echo "【终端 1 - 基础栈】"
echo "ssh feedbot@$VEHICLE_IP"
echo "cd /work && source /opt/ros/jazzy/setup.bash && source install/setup.bash"
echo "ros2 launch feedbot_bringup base.launch.py"
echo ""
echo "【终端 2 - 网页服务】"
echo "ssh feedbot@$VEHICLE_IP"
echo "cd /work/agv_workspace/acceptance"
echo "python3 acceptance_web_backend.py"
echo ""
echo "【终端 3 - SSH 隧道】（本地开发机）"
echo "ssh -L 8765:$VEHICLE_IP:8765 feedbot@$VEHICLE_IP"
echo ""
echo "然后在浏览器中打开："
echo "👉 http://localhost:8765"
echo ""
echo "=========================================="

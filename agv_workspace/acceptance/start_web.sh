#!/bin/bash
# 实车验证网页启动脚本

set -e

cd "$(dirname "$0")"

echo "=========================================="
echo "Phase 1A 实车验证网页"
echo "=========================================="

# 检查依赖
echo ""
echo "检查依赖..."

if ! python3 -c "import aiohttp" 2>/dev/null; then
    echo "   安装 aiohttp..."
    pip install aiohttp -q
fi

if ! python3 -c "import websockets" 2>/dev/null; then
    echo "   安装 websockets..."
    pip install websockets -q
fi

echo "   ✓ 依赖就绪"

# 启动后端
echo ""
echo "🚀 启动网页服务..."
echo ""

python3 acceptance_web_backend.py

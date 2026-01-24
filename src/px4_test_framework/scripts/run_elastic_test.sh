#!/bin/bash

# Elastic Tracker 测试脚本（集成状态机框架）
# 使用方法：./run_elastic_test.sh

echo "========================================="
echo "  Elastic Tracker + PX4 测试脚本"
echo "  (集成状态机框架)"
echo "========================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查 PX4 是否正在运行
echo -e "${YELLOW}[1/5] 检查 PX4 SITL 是否运行...${NC}"
if ! pgrep -f "px4" > /dev/null; then
    echo -e "${RED}错误: PX4 SITL 未运行!${NC}"
    echo "请在另一个终端运行："
    echo "  cd ~/PX4-Autopilot"
    echo "  make px4_sitl gazebo"
    exit 1
fi
echo -e "${GREEN}✓ PX4 SITL 正在运行${NC}"

# 检查 MAVROS 是否正在运行
echo -e "${YELLOW}[2/5] 检查 MAVROS 是否运行...${NC}"
if ! rostopic list | grep -q "/mavros"; then
    echo -e "${RED}错误: MAVROS 未运行!${NC}"
    echo "请在另一个终端运行："
    echo "  roslaunch px4_test_framework px4_simulation.launch"
    exit 1
fi
echo -e "${GREEN}✓ MAVROS 正在运行${NC}"

# 等待一下
sleep 2

# 启动 Elastic Tracker（包含状态机）
echo -e "${YELLOW}[3/5] 启动 Elastic Tracker + 状态机...${NC}"
echo -e "${YELLOW}状态机将自动执行：ARMING → TAKEOFF → HOVER → TRAJ${NC}"
echo -e "${YELLOW}进入 TRAJ 状态时，Elastic Tracker 将自动开始跟踪${NC}"

roslaunch px4_test_framework elastic_tracking.launch &
ELASTIC_PID=$!

sleep 3

echo -e "${GREEN}=========================================${NC}"
echo -e "${GREEN}  Elastic Tracker 测试已启动!${NC}"
echo -e "${GREEN}=========================================${NC}"
echo ""
echo "工作流程："
echo "  1. 状态机自动执行：ARMING → TAKEOFF → HOVER"
echo "  2. 等待进入 TRAJ 状态（可通过发送命令触发）"
echo "  3. 进入 TRAJ 状态后，Elastic Tracker 自动开始跟踪"
echo ""
echo "手动触发 TRAJ 状态（可选）："
echo "  rostopic pub /state/command_drone_0 std_msgs/Int32 \"data: 5\" --once"
echo ""
echo "按 Ctrl+C 停止测试"

# 等待用户中断
wait $ELASTIC_PID


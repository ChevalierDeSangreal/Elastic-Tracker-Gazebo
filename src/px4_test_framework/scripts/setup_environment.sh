#!/bin/bash

# 环境设置脚本
# 使用方法：source setup_environment.sh

echo "========================================="
echo "  PX4 测试框架 - 环境设置"
echo "========================================="

# 设置 ROS 环境
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$(dirname $(dirname $(dirname $SCRIPT_DIR)))"

if [ -f "$WORKSPACE_DIR/devel/setup.bash" ]; then
    source "$WORKSPACE_DIR/devel/setup.bash"
    echo "✓ ROS 工作空间已加载: $WORKSPACE_DIR"
else
    echo "✗ 错误: 未找到 devel/setup.bash"
    echo "  请先编译项目: cd $WORKSPACE_DIR && catkin_make"
    return 1
fi

# 设置使用系统时间（不使用仿真时间）
export USE_SIM_TIME=false
rosparam set use_sim_time false 2>/dev/null || true

# 设置 Gazebo 模型路径（如果需要）
if [ -d "$HOME/PX4-Autopilot/Tools/sitl_gazebo/models" ]; then
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/PX4-Autopilot/Tools/sitl_gazebo/models
fi

# 设置测试结果保存目录
export TRACKING_RESULTS_DIR="/tmp/tracking_results"
mkdir -p $TRACKING_RESULTS_DIR
echo "✓ 测试结果目录: $TRACKING_RESULTS_DIR"

# 打印有用的命令
echo ""
echo "========================================="
echo "  可用命令"
echo "========================================="
echo "启动 PX4 SITL:"
echo "  cd ~/PX4-Autopilot && make px4_sitl gazebo"
echo ""
echo "启动测试环境:"
echo "  roslaunch px4_test_framework px4_simulation.launch"
echo ""
echo "测试 Elastic Tracker:"
echo "  roslaunch px4_test_framework elastic_tracking.launch"
echo ""
echo "测试神经网络:"
echo "  roslaunch px4_test_framework neural_tracking.launch"
echo ""
echo "性能评估:"
echo "  rosrun px4_test_framework performance_evaluator.py _test_name:=<name>"
echo "========================================="


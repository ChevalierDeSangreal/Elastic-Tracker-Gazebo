# Elastic-Tracker

## 0. Overview
**Elastic-Tracker** is a flexible trajectory planning framework that can deal with challenging tracking tasks with guaranteed safety and visibility.

**Authors**: Jialin Ji, Neng Pan and [Fei Gao](https://ustfei.com/) from the [ZJU Fast Lab](http://zju-fast.com/). 

**Paper**: [Elastic Tracker: A Spatio-temporal Trajectory Planner Flexible Aerial Tracking](https://arxiv.org/abs/2109.07111), Jialin Ji, Neng Pan, Chao Xu, Fei Gao, Accepted in IEEE International Conference on Robotics and Automation (__ICRA 2022__).

**Video Links**: [youtube](https://www.youtube.com/watch?v=G5taHOpAZj8) or [bilibili](https://www.bilibili.com/video/BV1o44y1b7wC)
<a href="https://www.youtube.com/watch?v=G5taHOpAZj8" target="blank">
  <p align="center">
    <img src="figs/cover.png" width="500"/>
  </p>
</a>

## 1. Simulation of Aerial Tracking 

[NOTE] remember to change the CUDA option of **src/uav_simulator/local_sensing/CMakeLists.txt**

>Preparation and visualization:
```
git clone https://github.com/ZJU-FAST-Lab/Elastic-Tracker.git
cd Elastic-Tracker
catkin_make
source devel/setup.zsh
chmod +x sh_utils/pub_triger.sh
roslaunch mapping rviz_sim.launch
```

>A small drone with the global map as the chasing target:
```
roslaunch planning fake_target.launch
```

>Start the elastic tracker:
```
roslaunch planning simulation1.launch
```

>Triger the drone to track the target:
```
./sh_utils/pub_triger.sh
```
<p align="center">
    <img src="figs/sim1.gif" width="500"/>
</p>

Comparision of the planners *with* ![blue](https://via.placeholder.com/10/1F24E6/000000?text=+) and *without* ![blue](https://via.placeholder.com/10/E6691E/000000?text=+) **visibility guarantee**:
```
roslaunch planning simulation2.launch
```
<p align="center">
    <img src="figs/sim2.gif" width="500"/>
</p>

## 2. Simulation of Aerial Landing

> First start the stage of tracking:
```
roslaunch planning fake_car_target.launch
roslaunch planning simulation_landing.launch
./sh_utils/pub_triger.sh
```
> Triger the drone to land on the moving vehicle:
```
./sh_utils/land_triger.sh
```
<p align="center">
    <img src="figs/sim_landing.gif" width="500"/>
</p>

## 3. Tracking Visualization (追踪可视化)

为了方便分析和对比跟踪效果，我们提供了追踪结果可视化工具，**已集成到主 launch 文件中**。

### 使用方法

**一键启动（推荐）**：
```bash
# 启动 Elastic Tracker，自动包含可视化和RViz
cd Elastic-Tracker
source devel/setup.bash
roslaunch px4_test_framework elastic_tracking.launch use_rviz:=true
```

可视化节点会自动启动，RViz 会显示完整的追踪可视化界面。

**独立启动**（如果不想自动启动）：
```bash
# 单独启动可视化节点
roslaunch tracking_visualizer tracking_visualizer.launch

# 单独启动 RViz
rviz -d $(rospack find tracking_visualizer)/config/tracking_visualization.rviz
```

### 功能特性

- **实时轨迹显示**：无人机轨迹（蓝色）和目标轨迹（红色）
- **误差可视化**：黄色连线显示当前跟踪误差，文本显示误差数值
- **数据记录**：自动保存追踪数据到 `/tmp/elastic_tracker_tracking_data.csv`
- **统计分析**：实时计算平均误差、最大误差、RMS 误差

### 可视化话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/tracking_viz/drone_path` | `nav_msgs/Path` | 无人机历史轨迹 |
| `/tracking_viz/target_path` | `nav_msgs/Path` | 目标历史轨迹 |
| `/tracking_viz/error_markers` | `visualization_msgs/MarkerArray` | 误差可视化标记 |
| `/tracking_viz/distance_error` | `std_msgs/Float64` | 当前距离误差 [m] |
| `/tracking_viz/velocity_error` | `std_msgs/Float64` | 当前速度误差 [m/s] |

## 4. Acknowledgement
We use [**MINCO**](https://github.com/ZJU-FAST-Lab/GCOPTER) as our trajectory representation.

We use [**DecompROS**](https://github.com/sikang/DecompROS) for safe flight corridor generation and visualization.
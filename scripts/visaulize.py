import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# 读取数据
data = pd.read_csv('/tmp/elastic_tracker_tracking_data.csv')

# 打印数据基本信息
print(f"数据总行数: {len(data)}")
print(f"时间戳范围: {data['timestamp'].min():.2f} - {data['timestamp'].max():.2f}")
print(f"时间跨度: {data['timestamp'].max() - data['timestamp'].min():.2f} 秒")

# 检查是否有NaN值
print(f"缺失值检查:")
print(f"  timestamp: {data['timestamp'].isna().sum()}")
print(f"  drone_x: {data['drone_x'].isna().sum()}")
print(f"  target_x: {data['target_x'].isna().sum()}")
print(f"  setpoint_x: {data['setpoint_x'].isna().sum()}")

# 删除包含NaN的行
data = data.dropna()
print(f"删除NaN后数据行数: {len(data)}")

# 确保数据按时间排序
data_sorted = data.sort_values('timestamp').reset_index(drop=True)
time_sorted = data_sorted['timestamp'] - data_sorted['timestamp'].iloc[0]

# 检查时间戳是否有效（如果所有时间戳相同，使用数据索引作为时间轴）
if time_sorted.max() <= 0.0:
    print("警告: 所有时间戳相同，使用数据索引作为时间轴")
    # 假设采样频率为10Hz（根据visualization_rate默认值）
    # 或者根据数据量估算：如果有N个数据点，假设总时长，计算采样频率
    estimated_duration = len(data_sorted) / 10.0  # 假设10Hz采样
    time_sorted = np.arange(len(data_sorted)) / 10.0  # 使用索引/采样频率作为时间
    print(f"使用估算时间轴: 0.00 - {time_sorted.max():.2f} 秒 (假设10Hz采样)")
else:
    print(f"时间范围: {time_sorted.min():.2f} - {time_sorted.max():.2f} 秒")

# 重新计算排序后的数据
dx_sorted = data_sorted['target_x'] - data_sorted['drone_x']
dy_sorted = data_sorted['target_y'] - data_sorted['drone_y']
dz_sorted = data_sorted['target_z'] - data_sorted['drone_z']
distance_sorted = np.sqrt(dx_sorted**2 + dy_sorted**2 + dz_sorted**2)

drone_velocity_magnitude_sorted = np.sqrt(data_sorted['drone_vx']**2 + data_sorted['drone_vy']**2 + data_sorted['drone_vz']**2)
target_velocity_magnitude_sorted = np.sqrt(data_sorted['target_vx']**2 + data_sorted['target_vy']**2 + data_sorted['target_vz']**2)
velocity_diff_sorted = drone_velocity_magnitude_sorted - target_velocity_magnitude_sorted

# 计算无人机机体x轴与到目标方向的夹角
# 使用完整的roll、pitch、yaw构建旋转矩阵
drone_roll_sorted = data_sorted['drone_roll']    # ENU坐标系中的roll角
drone_pitch_sorted = data_sorted['drone_pitch']  # ENU坐标系中的pitch角
drone_yaw_sorted = data_sorted['drone_yaw']      # ENU坐标系中的yaw角

# 从无人机到目标的方向向量（归一化）
direction_norm = np.sqrt(dx_sorted**2 + dy_sorted**2 + dz_sorted**2)
direction_x_normalized = dx_sorted / (direction_norm + 1e-8)
direction_y_normalized = dy_sorted / (direction_norm + 1e-8)
direction_z_normalized = dz_sorted / (direction_norm + 1e-8)

# 使用ZYX欧拉角顺序（yaw-pitch-roll）构建旋转矩阵，将世界坐标转换到机体坐标
cos_roll = np.cos(drone_roll_sorted)
sin_roll = np.sin(drone_roll_sorted)
cos_pitch = np.cos(drone_pitch_sorted)
sin_pitch = np.sin(drone_pitch_sorted)
cos_yaw = np.cos(drone_yaw_sorted)
sin_yaw = np.sin(drone_yaw_sorted)

# 将目标方向向量转换到机体坐标系
direction_body_x = (cos_yaw * cos_pitch * dx_sorted + 
                   sin_yaw * cos_pitch * dy_sorted - 
                   sin_pitch * dz_sorted)
direction_body_y = ((cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll) * dx_sorted +
                   (sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll) * dy_sorted +
                   cos_pitch * sin_roll * dz_sorted)
direction_body_z = ((cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll) * dx_sorted +
                   (sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll) * dy_sorted +
                   cos_pitch * cos_roll * dz_sorted)

# 归一化机体坐标系中的方向向量
direction_body_norm = np.sqrt(direction_body_x**2 + direction_body_y**2 + direction_body_z**2)
direction_body_x_normalized = direction_body_x / (direction_body_norm + 1e-8)

# 计算机体x轴[1,0,0]与目标方向的夹角
# cos(angle) = dot([1,0,0], direction_body_normalized) = direction_body_x_normalized
cos_angle = np.clip(direction_body_x_normalized, -1.0, 1.0)
angle_rad_sorted = np.arccos(cos_angle)
angle_deg_sorted = np.degrees(angle_rad_sorted)

# 绘制四个子图 (2x2布局)
fig = plt.figure(figsize=(16, 12))
gs = fig.add_gridspec(2, 2, hspace=0.3, wspace=0.3)
axes = [fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1]), 
        fig.add_subplot(gs[1, 0]), fig.add_subplot(gs[1, 1])]

# 子图1: 到目标物体距离随时间变化
axes[0].plot(time_sorted, distance_sorted, linewidth=2, color='blue', label='Distance')
axes[0].axhline(y=distance_sorted.mean(), color='r', linestyle='--', 
                label=f'Mean: {distance_sorted.mean():.4f} m')
axes[0].set_xlabel('Time (s)')
axes[0].set_ylabel('Distance (m)')
axes[0].set_title('Distance to Target vs Time')
axes[0].legend()
axes[0].grid(True)
if time_sorted.max() > time_sorted.min():
    axes[0].set_xlim([time_sorted.min(), time_sorted.max()])

# 子图2: 与目标物体速度值差异随时间变化
axes[1].plot(time_sorted, velocity_diff_sorted, linewidth=2, color='orange', label='Velocity Difference')
axes[1].axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel('Velocity Difference (m/s)')
axes[1].set_title('Velocity Magnitude Difference vs Time')
axes[1].legend()
axes[1].grid(True)
if time_sorted.max() > time_sorted.min():
    axes[1].set_xlim([time_sorted.min(), time_sorted.max()])

# 子图3: 无人机机体x轴与到目标方向的夹角随时间变化
axes[2].plot(time_sorted, angle_deg_sorted, linewidth=2, color='green', label='Angle')
axes[2].axhline(y=angle_deg_sorted.mean(), color='r', linestyle='--', 
                label=f'Mean: {angle_deg_sorted.mean():.4f} deg')
axes[2].set_xlabel('Time (s)')
axes[2].set_ylabel('Angle (deg)')
axes[2].set_title('Angle: Body X-axis to Target Direction vs Time')
axes[2].legend()
axes[2].grid(True)
if time_sorted.max() > time_sorted.min():
    axes[2].set_xlim([time_sorted.min(), time_sorted.max()])

# 子图4: 俯视图 - 目标、无人机和规划点轨迹
axes[3].plot(data_sorted['target_x'], data_sorted['target_y'], 
             linewidth=2, color='red', label='Target Trajectory', alpha=0.8)
axes[3].plot(data_sorted['drone_x'], data_sorted['drone_y'], 
             linewidth=2, color='blue', label='Drone Trajectory', alpha=0.8)
axes[3].plot(data_sorted['setpoint_x'], data_sorted['setpoint_y'], 
             linewidth=2, color='green', label='Setpoint Trajectory', alpha=0.8, linestyle='--')

# 标记起点和终点
axes[3].scatter(data_sorted['target_x'].iloc[0], data_sorted['target_y'].iloc[0], 
                s=100, c='red', marker='o', edgecolors='black', linewidths=2, 
                label='Target Start', zorder=5)
axes[3].scatter(data_sorted['drone_x'].iloc[0], data_sorted['drone_y'].iloc[0], 
                s=100, c='blue', marker='o', edgecolors='black', linewidths=2, 
                label='Drone Start', zorder=5)
axes[3].scatter(data_sorted['target_x'].iloc[-1], data_sorted['target_y'].iloc[-1], 
                s=100, c='red', marker='s', edgecolors='black', linewidths=2, 
                label='Target End', zorder=5)
axes[3].scatter(data_sorted['drone_x'].iloc[-1], data_sorted['drone_y'].iloc[-1], 
                s=100, c='blue', marker='s', edgecolors='black', linewidths=2, 
                label='Drone End', zorder=5)

axes[3].set_xlabel('X Position (m)')
axes[3].set_ylabel('Y Position (m)')
axes[3].set_title('Top View: Target, Drone and Setpoint Trajectories')
axes[3].legend(loc='best', fontsize=9)
axes[3].grid(True, alpha=0.3)
axes[3].axis('equal')  # 保持坐标轴比例一致

plt.tight_layout()
# 获取脚本所在目录
script_dir = os.path.dirname(os.path.abspath(__file__))
output_path = os.path.join(script_dir, 'tracking_performance.png')
plt.savefig(output_path, dpi=300, bbox_inches='tight')
plt.close()
print(f"图片已保存到: {output_path}")

# 打印统计信息
print("\n=== Elastic-Tracker 跟踪统计信息 ===")
print(f"\n【距离统计】")
print(f"  平均值: {distance_sorted.mean():.4f} m")
print(f"  方差:   {distance_sorted.var():.4f} m²")
print(f"  标准差: {distance_sorted.std():.4f} m")
print(f"  最大值: {distance_sorted.max():.4f} m")
print(f"  最小值: {distance_sorted.min():.4f} m")

print(f"\n【夹角统计】")
print(f"  平均值: {angle_deg_sorted.mean():.4f}°")
print(f"  方差:   {angle_deg_sorted.var():.4f}°²")
print(f"  标准差: {angle_deg_sorted.std():.4f}°")
print(f"  最大值: {angle_deg_sorted.max():.4f}°")
print(f"  最小值: {angle_deg_sorted.min():.4f}°")

print(f"\n【速度差异统计】")
print(f"  平均值: {velocity_diff_sorted.mean():.4f} m/s")
print(f"  标准差: {velocity_diff_sorted.std():.4f} m/s")
print(f"  最大值: {velocity_diff_sorted.max():.4f} m/s")
print(f"  最小值: {velocity_diff_sorted.min():.4f} m/s")


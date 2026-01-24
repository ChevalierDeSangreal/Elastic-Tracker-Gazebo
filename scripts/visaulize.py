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
# 在ENU坐标系中，机体x轴方向通过yaw角确定
drone_yaw_sorted = data_sorted['drone_yaw']  # ENU坐标系中的yaw角

# 机体x轴在世界坐标系中的方向（ENU: x=East, y=North）
body_x_world_x = np.cos(drone_yaw_sorted)  # x分量
body_x_world_y = np.sin(drone_yaw_sorted)  # y分量

# 从无人机到目标的方向向量（归一化）
direction_norm = np.sqrt(dx_sorted**2 + dy_sorted**2 + dz_sorted**2)
direction_x_normalized = dx_sorted / (direction_norm + 1e-8)
direction_y_normalized = dy_sorted / (direction_norm + 1e-8)

# 计算机体x轴与目标方向的夹角（使用点积）
# 只考虑水平面（xy平面）的投影
cos_angle = body_x_world_x * direction_x_normalized + body_x_world_y * direction_y_normalized
cos_angle = np.clip(cos_angle, -1.0, 1.0)
angle_rad_sorted = np.arccos(cos_angle)
angle_deg_sorted = np.degrees(angle_rad_sorted)

# 绘制三个子图
fig, axes = plt.subplots(3, 1, figsize=(12, 12))

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


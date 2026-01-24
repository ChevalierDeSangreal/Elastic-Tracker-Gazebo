#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
性能评估工具
记录无人机和目标的轨迹，计算跟踪性能指标
"""

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
from datetime import datetime

class PerformanceEvaluator:
    def __init__(self):
        rospy.init_node('performance_evaluator', anonymous=True)
        
        # 数据存储
        self.drone_positions = []
        self.drone_velocities = []
        self.drone_timestamps = []
        
        self.target_positions = []
        self.target_timestamps = []
        
        # 参数
        self.save_dir = rospy.get_param('~save_dir', '/tmp/tracking_results')
        self.test_name = rospy.get_param('~test_name', 'test')
        
        # 创建保存目录
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        
        # 订阅话题
        self.drone_odom_sub = rospy.Subscriber(
            '/drone0/mavros/local_position/odom',
            Odometry,
            self.drone_odom_callback
        )
        
        self.target_pos_sub = rospy.Subscriber(
            '/target/position',
            PointStamped,
            self.target_position_callback
        )
        
        rospy.loginfo("性能评估器已启动")
        rospy.loginfo("保存目录: %s", self.save_dir)
        rospy.loginfo("测试名称: %s", self.test_name)
        
    def drone_odom_callback(self, msg):
        """无人机里程计回调"""
        t = msg.header.stamp.to_sec()
        pos = [msg.pose.pose.position.x, 
               msg.pose.pose.position.y, 
               msg.pose.pose.position.z]
        vel = [msg.twist.twist.linear.x,
               msg.twist.twist.linear.y,
               msg.twist.twist.linear.z]
        
        self.drone_timestamps.append(t)
        self.drone_positions.append(pos)
        self.drone_velocities.append(vel)
        
    def target_position_callback(self, msg):
        """目标位置回调"""
        t = msg.header.stamp.to_sec()
        pos = [msg.point.x, msg.point.y, msg.point.z]
        
        self.target_timestamps.append(t)
        self.target_positions.append(pos)
        
    def calculate_metrics(self):
        """计算性能指标"""
        if len(self.drone_positions) == 0 or len(self.target_positions) == 0:
            rospy.logwarn("没有足够的数据进行评估")
            return None
        
        drone_pos = np.array(self.drone_positions)
        target_pos = np.array(self.target_positions)
        drone_vel = np.array(self.drone_velocities)
        
        # 插值使时间对齐
        drone_t = np.array(self.drone_timestamps)
        target_t = np.array(self.target_timestamps)
        
        # 使用无人机的时间戳作为基准
        target_pos_interp = np.zeros_like(drone_pos)
        for i in range(3):
            target_pos_interp[:, i] = np.interp(drone_t, target_t, target_pos[:, i])
        
        # 计算跟踪误差
        errors = np.linalg.norm(drone_pos - target_pos_interp, axis=1)
        
        # 性能指标
        metrics = {
            'mean_error': np.mean(errors),
            'std_error': np.std(errors),
            'max_error': np.max(errors),
            'rmse': np.sqrt(np.mean(errors**2)),
            'mean_velocity': np.mean(np.linalg.norm(drone_vel, axis=1)),
            'max_velocity': np.max(np.linalg.norm(drone_vel, axis=1)),
            'duration': drone_t[-1] - drone_t[0],
            'num_samples': len(drone_pos)
        }
        
        return metrics, drone_pos, target_pos_interp, drone_t, errors
    
    def save_results(self):
        """保存结果"""
        rospy.loginfo("正在保存结果...")
        
        result = self.calculate_metrics()
        if result is None:
            return
        
        metrics, drone_pos, target_pos, drone_t, errors = result
        
        # 打印指标
        rospy.loginfo("========== 性能指标 ==========")
        rospy.loginfo("平均误差: %.4f m", metrics['mean_error'])
        rospy.loginfo("误差标准差: %.4f m", metrics['std_error'])
        rospy.loginfo("最大误差: %.4f m", metrics['max_error'])
        rospy.loginfo("RMSE: %.4f m", metrics['rmse'])
        rospy.loginfo("平均速度: %.4f m/s", metrics['mean_velocity'])
        rospy.loginfo("最大速度: %.4f m/s", metrics['max_velocity'])
        rospy.loginfo("测试时长: %.2f s", metrics['duration'])
        rospy.loginfo("采样点数: %d", metrics['num_samples'])
        rospy.loginfo("=============================")
        
        # 保存到文件
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename_prefix = os.path.join(self.save_dir, f"{self.test_name}_{timestamp}")
        
        # 保存指标
        with open(f"{filename_prefix}_metrics.txt", 'w') as f:
            f.write("========== 性能指标 ==========\n")
            for key, value in metrics.items():
                f.write(f"{key}: {value}\n")
        
        # 保存轨迹数据
        np.savez(f"{filename_prefix}_trajectory.npz",
                 drone_pos=drone_pos,
                 target_pos=target_pos,
                 drone_t=drone_t,
                 errors=errors)
        
        # 绘制轨迹
        self.plot_trajectory(drone_pos, target_pos, drone_t, errors, filename_prefix)
        
        rospy.loginfo("结果已保存到: %s", filename_prefix)
    
    def plot_trajectory(self, drone_pos, target_pos, drone_t, errors, filename_prefix):
        """绘制轨迹和误差"""
        # 3D轨迹
        fig = plt.figure(figsize=(15, 5))
        
        # 子图1: 3D轨迹
        ax1 = fig.add_subplot(131, projection='3d')
        ax1.plot(drone_pos[:, 0], drone_pos[:, 1], drone_pos[:, 2], 
                 'b-', label='Drone', linewidth=2)
        ax1.plot(target_pos[:, 0], target_pos[:, 1], target_pos[:, 2], 
                 'r--', label='Target', linewidth=2)
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_zlabel('Z [m]')
        ax1.set_title('3D Trajectory')
        ax1.legend()
        ax1.grid(True)
        
        # 子图2: XY平面轨迹
        ax2 = fig.add_subplot(132)
        ax2.plot(drone_pos[:, 0], drone_pos[:, 1], 'b-', label='Drone', linewidth=2)
        ax2.plot(target_pos[:, 0], target_pos[:, 1], 'r--', label='Target', linewidth=2)
        ax2.set_xlabel('X [m]')
        ax2.set_ylabel('Y [m]')
        ax2.set_title('XY Plane Trajectory')
        ax2.legend()
        ax2.grid(True)
        ax2.axis('equal')
        
        # 子图3: 跟踪误差
        ax3 = fig.add_subplot(133)
        time_rel = drone_t - drone_t[0]
        ax3.plot(time_rel, errors, 'g-', linewidth=2)
        ax3.set_xlabel('Time [s]')
        ax3.set_ylabel('Tracking Error [m]')
        ax3.set_title('Tracking Error vs Time')
        ax3.grid(True)
        
        plt.tight_layout()
        plt.savefig(f"{filename_prefix}_plot.png", dpi=150)
        rospy.loginfo("图表已保存: %s_plot.png", filename_prefix)
        
    def run(self):
        """运行评估器"""
        rospy.spin()
        
        # 关闭时保存结果
        self.save_results()

if __name__ == '__main__':
    try:
        evaluator = PerformanceEvaluator()
        evaluator.run()
    except rospy.ROSInterruptException:
        pass


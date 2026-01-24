#ifndef SO3_TO_PX4_NODE_HPP
#define SO3_TO_PX4_NODE_HPP

#include <ros/ros.h>
#include <quadrotor_msgs/SO3Command.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/**
 * @brief SO3命令到PX4转换节点
 * 
 * 将Elastic-Tracker的SO3控制器输出转换为PX4 Attitude Target控制指令
 * SO3Command包含：force (推力向量), orientation (期望姿态四元数), kR, kOm (姿态控制增益)
 * PX4需要：thrust (归一化推力), orientation (姿态四元数), body_rate (角速度)
 */
class SO3ToPX4Node {
public:
  SO3ToPX4Node(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  ~SO3ToPX4Node() = default;

private:
  void so3_cmd_callback(const quadrotor_msgs::SO3Command::ConstPtr& msg);
  void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
  
  // 计算归一化推力
  double calculate_normalized_thrust(const Eigen::Vector3d& force);
  
  // ROS节点句柄
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  
  // 订阅器和发布器
  ros::Subscriber so3_cmd_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher attitude_target_pub_;
  
  // 参数
  double mass_;              // 无人机质量 [kg]
  double g_;                 // 重力加速度 [m/s^2]
  double max_thrust_;        // 最大推力 [N]
  bool use_body_rate_;       // 是否使用角速率控制模式
  
  // 状态变量
  Eigen::Vector3d current_angular_velocity_;  // 当前角速度
  bool odom_received_;
  
  // 时间戳
  ros::Time last_so3_cmd_time_;
};

#endif // SO3_TO_PX4_NODE_HPP


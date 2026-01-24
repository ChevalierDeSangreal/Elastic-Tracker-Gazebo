#include "px4_test_framework/so3_to_px4_node.hpp"
#include <cmath>

SO3ToPX4Node::SO3ToPX4Node(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
  : nh_(nh)
  , nh_private_(nh_private)
  , odom_received_(false)
  , current_angular_velocity_(Eigen::Vector3d::Zero())
{
  // 读取参数
  nh_private_.param("mass", mass_, 1.5);  // 默认无人机质量 1.5kg
  nh_private_.param("g", g_, 9.81);       // 重力加速度
  nh_private_.param("max_thrust", max_thrust_, 25.0);  // 最大推力 25N
  nh_private_.param("use_body_rate", use_body_rate_, false);  // 默认不使用角速率模式
  
  ROS_INFO("=== SO3 to PX4 Converter Node ===");
  ROS_INFO("Mass: %.2f kg", mass_);
  ROS_INFO("Gravity: %.2f m/s^2", g_);
  ROS_INFO("Max thrust: %.2f N", max_thrust_);
  ROS_INFO("Use body rate control: %s", use_body_rate_ ? "YES" : "NO");
  
  // 创建订阅器
  // 使用相对话题名，在 drone0 命名空间下运行时自动变为 /drone0/so3_cmd
  so3_cmd_sub_ = nh_.subscribe<quadrotor_msgs::SO3Command>(
    "so3_cmd", 10, &SO3ToPX4Node::so3_cmd_callback, this);
  
  // 订阅 MAVROS 话题（相对路径，自动添加命名空间前缀）
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
    "mavros/local_position/odom", 10, &SO3ToPX4Node::odom_callback, this);
  
  // 创建发布器（相对路径，自动添加命名空间前缀）
  attitude_target_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(
    "mavros/setpoint_raw/attitude", 10);
  
  ROS_INFO("Subscribed to: ~so3_cmd");
  ROS_INFO("Subscribed to: ~mavros/local_position/odom");
  ROS_INFO("Publishing to: ~mavros/setpoint_raw/attitude");
  ROS_INFO("SO3 to PX4 converter node initialized successfully!");
}

void SO3ToPX4Node::odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  // 提取当前角速度（body frame）
  current_angular_velocity_ << 
    msg->twist.twist.angular.x,
    msg->twist.twist.angular.y,
    msg->twist.twist.angular.z;
  
  odom_received_ = true;
}

void SO3ToPX4Node::so3_cmd_callback(const quadrotor_msgs::SO3Command::ConstPtr& msg) {
  // 提取SO3命令中的力和姿态
  Eigen::Vector3d force(msg->force.x, msg->force.y, msg->force.z);
  Eigen::Quaterniond orientation(
    msg->orientation.w,
    msg->orientation.x,
    msg->orientation.y,
    msg->orientation.z
  );
  
  // 创建PX4姿态目标消息
  mavros_msgs::AttitudeTarget attitude_target;
  attitude_target.header.stamp = ros::Time::now();
  attitude_target.header.frame_id = "base_link";
  
  // 设置姿态四元数
  attitude_target.orientation.w = orientation.w();
  attitude_target.orientation.x = orientation.x();
  attitude_target.orientation.y = orientation.y();
  attitude_target.orientation.z = orientation.z();
  
  // 计算归一化推力 (0.0 - 1.0)
  attitude_target.thrust = calculate_normalized_thrust(force);
  
  // 类型掩码设置
  if (use_body_rate_) {
    // 使用姿态 + 角速率控制模式
    // IGNORE_ROLL_RATE | IGNORE_PITCH_RATE | IGNORE_YAW_RATE = 0
    // 表示使用角速率控制
    attitude_target.type_mask = 0;  // 使用姿态和角速率
    
    // 设置角速率（如果有的话，从SO3控制器的kOm参数中推导）
    // 这里简化处理，使用当前角速度
    if (odom_received_) {
      attitude_target.body_rate.x = current_angular_velocity_.x();
      attitude_target.body_rate.y = current_angular_velocity_.y();
      attitude_target.body_rate.z = current_angular_velocity_.z();
    }
  } else {
    // 只使用姿态控制模式
    // IGNORE_ROLL_RATE | IGNORE_PITCH_RATE | IGNORE_YAW_RATE = 7
    attitude_target.type_mask = 
      mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
      mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
      mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    
    // 角速率设为0
    attitude_target.body_rate.x = 0.0;
    attitude_target.body_rate.y = 0.0;
    attitude_target.body_rate.z = 0.0;
  }
  
  // 发布姿态目标
  attitude_target_pub_.publish(attitude_target);
  
  // 调试信息（限流）
  ROS_INFO_THROTTLE(1.0, "SO3->PX4: thrust=%.3f, q=[%.2f,%.2f,%.2f,%.2f]", 
                    attitude_target.thrust,
                    orientation.w(), orientation.x(), orientation.y(), orientation.z());
  
  last_so3_cmd_time_ = ros::Time::now();
}

double SO3ToPX4Node::calculate_normalized_thrust(const Eigen::Vector3d& force) {
  // 计算推力大小
  double force_magnitude = force.norm();
  
  // 归一化到 [0, 1]
  // PX4期望的推力范围是 0.0 - 1.0
  // 其中 0.0 表示无推力，1.0 表示最大推力
  double normalized_thrust = force_magnitude / max_thrust_;
  
  // 限制在合理范围内
  normalized_thrust = std::max(0.0, std::min(1.0, normalized_thrust));
  
  return normalized_thrust;
}


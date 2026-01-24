#include "px4_test_framework/state_listener_node.hpp"

// FSM states (from RealFlight_ros)
enum class FsmState {
  INIT = 0,
  ARMING = 1,
  TAKEOFF = 2,
  GOTO = 3,
  HOVER = 4,
  TRAJ = 5,
  END_TRAJ = 6,
  LAND = 7,
  DONE = 8
};

StateListenerNode::StateListenerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
  : nh_(nh)
  , nh_private_(nh_private)
  , last_state_(-1)
  , trigger_sent_(false)
  , in_traj_state_(false)
  , traj_completed_(false)
{
  // 读取参数
  nh_private_.param("drone_id", drone_id_, 0);
  nh_private_.param<std::string>("trigger_topic", trigger_topic_, "/move_base_simple/goal");
  nh_private_.param("traj_duration", traj_duration_, 30.0);  // 默认30秒
  nh_private_.param("end_traj_wait_time", end_traj_wait_time_, 5.0);  // END_TRAJ状态等待时间，用于日志显示
  
  ROS_INFO("=== State Listener Node ===");
  ROS_INFO("Drone ID: %d", drone_id_);
  ROS_INFO("Trigger topic: %s", trigger_topic_.c_str());
  ROS_INFO("TRAJ duration: %.1f seconds", traj_duration_);
  
  // 创建订阅器 - 订阅状态机状态
  std::string state_topic = "/state/state_drone_" + std::to_string(drone_id_);
  state_sub_ = nh_.subscribe<std_msgs::Int32>(
    state_topic, 10, &StateListenerNode::state_callback, this);
  
  // 创建发布器 - 发布触发信号给 Elastic Tracker
  trigger_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(trigger_topic_, 1);
  
  // 创建状态命令发布器 - 用于发送END_TRAJ命令
  // 注意：状态机订阅的话题是 /state/command_drone_X，不是 /state/state_cmd_drone_X
  std::string state_cmd_topic = "/state/command_drone_" + std::to_string(drone_id_);
  state_cmd_pub_ = nh_.advertise<std_msgs::Int32>(state_cmd_topic, 1);
  
  // 创建定时器 - 每秒检查一次TRAJ时间
  traj_timer_ = nh_.createTimer(ros::Duration(1.0), 
                                &StateListenerNode::traj_timer_callback, this);
  traj_timer_.stop();  // 初始停止，进入TRAJ状态时启动
  
  ROS_INFO("Subscribed to: %s", state_topic.c_str());
  ROS_INFO("Publishing to: %s", trigger_topic_.c_str());
  ROS_INFO("State command topic: %s", state_cmd_topic.c_str());
  ROS_INFO("State listener node initialized successfully!");
  ROS_INFO("Waiting for TRAJ state to trigger Elastic Tracker...");
}

void StateListenerNode::state_callback(const std_msgs::Int32::ConstPtr& msg) {
  int current_state = msg->data;
  
  // 检测状态变化
  if (current_state != last_state_) {
    ROS_INFO("State changed: %d -> %d", last_state_, current_state);
    last_state_ = current_state;
  }
  
  // 如果已经完成TRAJ任务（进入END_TRAJ或之后的状态），不再允许重新进入TRAJ
  if (traj_completed_ && current_state == static_cast<int>(FsmState::TRAJ)) {
    ROS_WARN_THROTTLE(5.0, "TRAJ task already completed (current_state=%d) - Ignoring re-entry to TRAJ state. Expected: END_TRAJ(6), LAND(7), or DONE(8)", 
                      current_state);
    return;
  }
  
  // 标记任务完成：一旦进入END_TRAJ、LAND或DONE状态，标记为完成
  if (!traj_completed_ && 
      (current_state == static_cast<int>(FsmState::END_TRAJ) ||
       current_state == static_cast<int>(FsmState::LAND) ||
       current_state == static_cast<int>(FsmState::DONE))) {
    traj_completed_ = true;
    ROS_INFO("TRAJ task completed - Will not re-enter TRAJ state");
  }
  
  // 当进入 TRAJ 状态时，发送触发信号给 Elastic Tracker
  if (current_state == static_cast<int>(FsmState::TRAJ) && !in_traj_state_ && !traj_completed_) {
    in_traj_state_ = true;
    traj_start_time_ = ros::Time::now();
    traj_timer_.start();  // 启动定时器
    
    ROS_INFO("✅ Entered TRAJ state - Sending trigger to Elastic Tracker");
    
    if (!trigger_sent_) {
      geometry_msgs::PoseStamped trigger_msg;
      trigger_msg.header.stamp = ros::Time::now();
      trigger_msg.header.frame_id = "world";
      trigger_msg.pose.position.x = 0.0;
      trigger_msg.pose.position.y = 0.0;
      trigger_msg.pose.position.z = 1.5;
      trigger_msg.pose.orientation.w = 1.0;
      trigger_msg.pose.orientation.x = 0.0;
      trigger_msg.pose.orientation.y = 0.0;
      trigger_msg.pose.orientation.z = 0.0;
      
      trigger_pub_.publish(trigger_msg);
      trigger_sent_ = true;
      
      ROS_INFO("Trigger signal sent to Elastic Tracker");
    }
    
    ROS_INFO("TRAJ timer started - Will auto-transition to END_TRAJ after %.1f seconds", traj_duration_);
  }
  
  // 当离开 TRAJ 状态时，重置标志并停止定时器
  if (current_state != static_cast<int>(FsmState::TRAJ) && in_traj_state_) {
    in_traj_state_ = false;
    traj_timer_.stop();  // 停止定时器
    
    if (current_state == static_cast<int>(FsmState::END_TRAJ)) {
      ROS_INFO("Left TRAJ state -> END_TRAJ - Timer stopped, waiting for auto-land");
    } else {
      ROS_INFO("Left TRAJ state -> %d - Timer stopped", current_state);
    }
  }
}

void StateListenerNode::traj_timer_callback(const ros::TimerEvent& event) {
  if (!in_traj_state_ || traj_completed_) {
    // 不在TRAJ状态或任务已完成，停止定时器
    traj_timer_.stop();
    return;
  }
  
  // 计算已进入TRAJ状态的时间
  double elapsed = (ros::Time::now() - traj_start_time_).toSec();
  
  // 检查是否到达设定的持续时间
  if (elapsed >= traj_duration_) {
    ROS_INFO("✅ TRAJ duration (%.1f s) reached - Sending END_TRAJ command", traj_duration_);
    
    // 标记任务即将完成（防止重复发送）
    traj_completed_ = true;
    
    // 发送END_TRAJ命令
    std_msgs::Int32 state_cmd;
    state_cmd.data = static_cast<int>(FsmState::END_TRAJ);
    state_cmd_pub_.publish(state_cmd);
    
    // 停止定时器
    traj_timer_.stop();
    in_traj_state_ = false;
    
    ROS_INFO("END_TRAJ command sent to topic: /state/command_drone_%d", drone_id_);
    ROS_INFO("State machine should transition: END_TRAJ -> LAND (after %.1fs wait) -> DONE", end_traj_wait_time_);
    ROS_INFO("TRAJ task completed - Will not re-enter TRAJ state");
  } else {
    // 打印剩余时间（每5秒打印一次）
    double remaining = traj_duration_ - elapsed;
    if (static_cast<int>(elapsed) % 5 == 0) {
      ROS_INFO_THROTTLE(5.0, "TRAJ: %.1f / %.1f seconds (%.1f remaining)", 
                        elapsed, traj_duration_, remaining);
    }
  }
}


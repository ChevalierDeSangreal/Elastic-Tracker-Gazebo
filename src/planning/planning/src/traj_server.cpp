#include <nav_msgs/Odometry.h>
#include <mavros_msgs/PositionTarget.h>
#include <quadrotor_msgs/PolyTraj.h>
//#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>

#include <traj_opt/poly_traj_utils.hpp>

ros::Publisher pos_cmd_pub_;
ros::Time heartbeat_time_;
bool receive_traj_ = false;
bool flight_start_ = false;
quadrotor_msgs::PolyTraj trajMsg_, trajMsg_last_;
Eigen::Vector3d last_p_;
double last_yaw_ = 0;
int current_fsm_state_ = -1;  // 当前状态机状态 (-1=未初始化, 5=TRAJ)
int drone_id_ = 0;  // 无人机ID，用于订阅状态话题

/* 发布指令到MAVROS */
void publish_cmd(int traj_id,
                 const Eigen::Vector3d &p,
                 const Eigen::Vector3d &v,
                 const Eigen::Vector3d &a,
                 double y, double yd) {
  //quadrotor_msgs::PositionCommand cmd;
  mavros_msgs::PositionTarget cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  //cmd.header.frame_id = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  //cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  //cmd.trajectory_id = traj_id;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration_or_force.x = a(0);
  cmd.acceleration_or_force.y = a(1);
  cmd.acceleration_or_force.z = a(2);
  cmd.yaw = y;
  cmd.yaw_rate = yd;
  pos_cmd_pub_.publish(cmd);
  last_p_ = p;
  last_yaw_ = y;
}

/* 处理轨迹为可执行的指令 */
bool exe_traj(const quadrotor_msgs::PolyTraj &trajMsg) {
  double t = (ros::Time::now() - trajMsg.start_time).toSec();
  if (t > 0) {
    // 接收到悬停指令
    if (trajMsg.hover) {
      if (trajMsg.hover_p.size() != 3) {
        ROS_ERROR("[traj_server] hover_p is not 3d!");
      }
      Eigen::Vector3d p, v0;
      p.x() = trajMsg.hover_p[0];
      p.y() = trajMsg.hover_p[1];
      p.z() = trajMsg.hover_p[2];
      v0.setZero();
      publish_cmd(trajMsg.traj_id, p, v0, v0, trajMsg.yaw, 0); 
      return true; // 在此返回
    }

    if (trajMsg.order != 5) {
      ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
      return false; // 在此返回
    }

    // 核验轨迹参数
    if (trajMsg.duration.size() * (trajMsg.order + 1) != trajMsg.coef_x.size()) {
      ROS_ERROR("[traj_server] WRONG trajectory parameters!");
      return false; // 在此返回
    }

    // 处理轨迹并发布
    int piece_nums = trajMsg.duration.size();
    std::vector<double> dura(piece_nums);
    std::vector<CoefficientMat> cMats(piece_nums);
    for (int i = 0; i < piece_nums; ++i) {
      int i6 = i * 6;
      cMats[i].row(0) << trajMsg.coef_x[i6 + 0], trajMsg.coef_x[i6 + 1], trajMsg.coef_x[i6 + 2],
          trajMsg.coef_x[i6 + 3], trajMsg.coef_x[i6 + 4], trajMsg.coef_x[i6 + 5];
      cMats[i].row(1) << trajMsg.coef_y[i6 + 0], trajMsg.coef_y[i6 + 1], trajMsg.coef_y[i6 + 2],
          trajMsg.coef_y[i6 + 3], trajMsg.coef_y[i6 + 4], trajMsg.coef_y[i6 + 5];
      cMats[i].row(2) << trajMsg.coef_z[i6 + 0], trajMsg.coef_z[i6 + 1], trajMsg.coef_z[i6 + 2],
          trajMsg.coef_z[i6 + 3], trajMsg.coef_z[i6 + 4], trajMsg.coef_z[i6 + 5];

      dura[i] = trajMsg.duration[i];
    }
    Trajectory traj(dura, cMats);
    if (t > traj.getTotalDuration()) {
      ROS_ERROR("[traj_server] trajectory too short left!");
      return false;
    }
    Eigen::Vector3d p, v, a;
    p = traj.getPos(t);
    v = traj.getVel(t);
    a = traj.getAcc(t);

    // 处理偏航角指令，确保在[-PI,PI]范围内，并进行限幅
    double yaw = trajMsg.yaw;
    double d_yaw = yaw - last_yaw_;
    d_yaw = d_yaw >= M_PI ? d_yaw - 2 * M_PI : d_yaw;
    d_yaw = d_yaw <= -M_PI ? d_yaw + 2 * M_PI : d_yaw;
    double d_yaw_abs = fabs(d_yaw);
    if (d_yaw_abs >= 0.2) {
      yaw = last_yaw_ + d_yaw / d_yaw_abs * 0.2;
    }
    publish_cmd(trajMsg.traj_id, p, v, a, yaw, 0);  // TODO yaw
    return true;
  }
  return false;
}

/* 接收心跳包回调 */
void heartbeatCallback(const std_msgs::EmptyConstPtr &msg) {
  heartbeat_time_ = ros::Time::now();
}

/* 状态机状态回调 */
void stateCallback(const std_msgs::Int32::ConstPtr& msg) {
  int new_state = msg->data;
  if (new_state != current_fsm_state_) {
    ROS_INFO("[traj_server] FSM state changed: %d -> %d", current_fsm_state_, new_state);
    current_fsm_state_ = new_state;
    
    // 如果离开TRAJ状态，停止处理轨迹
    if (current_fsm_state_ != 5) {
      ROS_WARN("[traj_server] Left TRAJ state - Stopping trajectory execution, FSM takes over");
    }
  }
}

/* 接收轨迹回调 */
void polyTrajCallback(const quadrotor_msgs::PolyTrajConstPtr &msgPtr) {
  trajMsg_ = *msgPtr;
  if (!receive_traj_) {
    trajMsg_last_ = trajMsg_;
    receive_traj_ = true;
  }
}

/* 接收发布指令回调 */
void cmdCallback(const ros::TimerEvent &e) {
  // 只在TRAJ状态(5)下运行
  // 参考RealFlight_ros: 控制节点只在TRAJ状态工作，其他状态由FSM完全接管
  if (current_fsm_state_ != 5) {
    // 不在TRAJ状态，直接返回，让状态机接管控制
    return;
  }
  
  if (!receive_traj_) {
    return;
  }
  ros::Time time_now = ros::Time::now();
  if ((time_now - heartbeat_time_).toSec() > 0.5) {
    ROS_ERROR_ONCE("[traj_server] Lost heartbeat from the planner, is he dead?");
    publish_cmd(trajMsg_.traj_id, last_p_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), last_yaw_, 0);  // TODO yaw
    return;
  }
  if (exe_traj(trajMsg_)) {
    trajMsg_last_ = trajMsg_;
    return;
  } else if (exe_traj(trajMsg_last_)) {
    return;
  }
}

/* 主函数 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle nh("~");

  // 读取无人机ID参数
  nh.param("drone_id", drone_id_, 0);

  ros::Subscriber poly_traj_sub = nh.subscribe("trajectory", 10, polyTrajCallback);
  ros::Subscriber heartbeat_sub = nh.subscribe("heartbeat", 10, heartbeatCallback);
  
  // 订阅状态机状态（使用全局话题名称，不受命名空间影响）
  std::string state_topic = "/state/state_drone_" + std::to_string(drone_id_);
  // 注意：使用全局NodeHandle订阅全局话题，避免命名空间影响
  ros::NodeHandle nh_global;
  ros::Subscriber state_sub = nh_global.subscribe<std_msgs::Int32>(state_topic, 10, stateCallback);
  ROS_INFO("[traj_server] Subscribed to FSM state: %s", state_topic.c_str());

  //pos_cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 50);
  pos_cmd_pub_ = nh.advertise<mavros_msgs::PositionTarget>("position_cmd", 50);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}
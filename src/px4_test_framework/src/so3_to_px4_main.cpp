#include <ros/ros.h>
#include "px4_test_framework/so3_to_px4_node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "so3_to_px4_node");
  
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  SO3ToPX4Node node(nh, nh_private);
  
  ROS_INFO("SO3 to PX4 converter node is running...");
  
  ros::spin();
  
  return 0;
}


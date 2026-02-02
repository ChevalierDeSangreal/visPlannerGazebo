#include <ros/ros.h>
#include "px4_test_framework/pos_cmd_to_mavros_node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "pos_cmd_to_mavros_node");
  
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  PosCmdToMavrosNode node(nh, nh_private);
  
  ROS_INFO("Position command to MAVROS converter node is running...");
  
  ros::spin();
  
  return 0;
}


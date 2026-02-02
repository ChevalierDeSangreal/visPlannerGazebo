#include <ros/ros.h>
#include "px4_test_framework/state_listener_node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "state_listener_node");
  
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  StateListenerNode node(nh, nh_private);
  
  ROS_INFO("State listener node is running...");
  
  ros::spin();
  
  return 0;
}


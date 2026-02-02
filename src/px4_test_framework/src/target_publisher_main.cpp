#include <ros/ros.h>
#include "px4_test_framework/target_publisher_node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "target_publisher_node");
  
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  TargetPublisherNode node(nh, nh_private);
  
  ROS_INFO("Target publisher node is running...");
  
  ros::spin();
  
  return 0;
}


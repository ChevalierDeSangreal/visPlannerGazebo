#include "px4_test_framework/target_odom_to_bspline_node.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "target_odom_to_bspline_node");
  
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  TargetOdomToBsplineNode node(nh, nh_private);
  
  ros::spin();
  
  return 0;
}





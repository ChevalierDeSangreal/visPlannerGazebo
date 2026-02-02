// main.cpp - ROS1 version
#include "offboard_state_machine/offboard_fsm_node.hpp"
#include <ros/ros.h>
#include <memory>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "offboard_fsm_node");
  
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  // Read the drone_id parameter
  int drone_id;
  nh_private.param("drone_id", drone_id, 0);
  
  ROS_INFO("Starting Offboard FSM process for drone %d", drone_id);
  
  // Create FSM node for this drone
  OffboardFSM fsm(nh, nh_private, drone_id);
  
  ros::spin();
  
  return 0;
}


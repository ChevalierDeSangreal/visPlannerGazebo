#ifndef POS_CMD_TO_MAVROS_NODE_HPP
#define POS_CMD_TO_MAVROS_NODE_HPP

#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Int32.h>

/**
 * @brief PositionCommand 到 MAVROS PositionTarget 转换节点
 * 
 * 将 visPlanner 的 quadrotor_msgs/PositionCommand 转换为
 * mavros_msgs/PositionTarget，以便通过 MAVROS 控制 PX4
 */
class PosCmdToMavrosNode {
public:
  PosCmdToMavrosNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  ~PosCmdToMavrosNode() = default;

private:
  void pos_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
  void state_callback(const std_msgs::Int32::ConstPtr& msg);
  
  // ROS节点句柄
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  
  // 订阅器和发布器
  ros::Subscriber pos_cmd_sub_;
  ros::Subscriber state_sub_;
  ros::Publisher mavros_pub_;
  
  // 参数
  int drone_id_;
  bool in_traj_state_;  // 是否在TRAJ状态
};

#endif // POS_CMD_TO_MAVROS_NODE_HPP


#include "px4_test_framework/pos_cmd_to_mavros_node.hpp"

PosCmdToMavrosNode::PosCmdToMavrosNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
  : nh_(nh)
  , nh_private_(nh_private)
  , drone_id_(0)
  , in_traj_state_(false)
{
  // 读取参数
  nh_private_.param("drone_id", drone_id_, 0);
  
  // 订阅 visPlanner 的 PositionCommand
  std::string pos_cmd_topic;
  nh_private_.param<std::string>("pos_cmd_topic", pos_cmd_topic, "/position_cmd");
  pos_cmd_sub_ = nh_.subscribe<quadrotor_msgs::PositionCommand>(
    pos_cmd_topic, 10, &PosCmdToMavrosNode::pos_cmd_callback, this);
  
  // 订阅状态机状态
  std::string state_topic = "/state/state_drone_" + std::to_string(drone_id_);
  state_sub_ = nh_.subscribe<std_msgs::Int32>(
    state_topic, 10, &PosCmdToMavrosNode::state_callback, this);
  
  // 发布到 MAVROS
  std::string mavros_topic;
  nh_private_.param<std::string>("mavros_topic", mavros_topic, "/mavros/setpoint_raw/local");
  mavros_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(mavros_topic, 10);
  
  ROS_INFO("=== Position Command to MAVROS Converter ===");
  ROS_INFO("Drone ID: %d", drone_id_);
  ROS_INFO("Subscribed to: %s", pos_cmd_topic.c_str());
  ROS_INFO("Publishing to: %s", mavros_topic.c_str());
  ROS_INFO("Subscribed to state: %s", state_topic.c_str());
}

void PosCmdToMavrosNode::state_callback(const std_msgs::Int32::ConstPtr& msg) {
  in_traj_state_ = (msg->data == 5);  // TRAJ state = 5
}

void PosCmdToMavrosNode::pos_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
  // 只在 TRAJ 状态下转换并发布命令
  if (!in_traj_state_) {
    return;
  }
  
  mavros_msgs::PositionTarget mavros_cmd;
  mavros_cmd.header.stamp = ros::Time::now();
  mavros_cmd.header.frame_id = "map";
  
  // PX4 使用 MAV_FRAME_LOCAL_NED (1)
  // MAVROS 会自动将 ENU 坐标转换为 NED
  mavros_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  
  // 设置位置
  mavros_cmd.position.x = msg->position.x;
  mavros_cmd.position.y = msg->position.y;
  mavros_cmd.position.z = msg->position.z;
  
  // 设置速度
  mavros_cmd.velocity.x = msg->velocity.x;
  mavros_cmd.velocity.y = msg->velocity.y;
  mavros_cmd.velocity.z = msg->velocity.z;
  
  // 设置加速度
  mavros_cmd.acceleration_or_force.x = msg->acceleration.x;
  mavros_cmd.acceleration_or_force.y = msg->acceleration.y;
  mavros_cmd.acceleration_or_force.z = msg->acceleration.z;
  
  // 设置偏航角
  mavros_cmd.yaw = msg->yaw;
  mavros_cmd.yaw_rate = msg->yaw_dot;
  
  // 设置 type_mask - 使用位置+速度+加速度+偏航角，忽略偏航角速度
  // type_mask: bit为1表示忽略该字段
  // 我们使用位置(0,1,2)、速度(3,4,5)、加速度(6,7,8)、偏航角(10)，忽略偏航角速度(11)
  mavros_cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  
  // 发布命令
  mavros_pub_.publish(mavros_cmd);
  
  // 输出调试信息（每100次输出一次，避免刷屏）
  static int count = 0;
  if (++count % 100 == 0) {
    ROS_INFO("[pos_cmd_to_mavros] Publishing: pos=[%.2f,%.2f,%.2f] vel=[%.2f,%.2f,%.2f] yaw=%.2f",
             mavros_cmd.position.x, mavros_cmd.position.y, mavros_cmd.position.z,
             mavros_cmd.velocity.x, mavros_cmd.velocity.y, mavros_cmd.velocity.z,
             mavros_cmd.yaw);
  }
}


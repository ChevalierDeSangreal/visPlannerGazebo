#ifndef STATE_LISTENER_NODE_HPP
#define STATE_LISTENER_NODE_HPP

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <traj_utils/Bspline.h>

/**
 * @brief 状态监听器节点
 * 
 * 监听状态机的状态，当进入 TRAJ 状态时发送触发信号给 visPlanner
 * 使 visPlanner 与状态机框架完全兼容
 */
class StateListenerNode {
public:
  StateListenerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  ~StateListenerNode() = default;

private:
  void state_callback(const std_msgs::Int32::ConstPtr& msg);
  void traj_timer_callback(const ros::TimerEvent& event);  // 定时器回调，检查TRAJ时间
  void target_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);  // 目标odom回调
  void broadcast_bspline_callback(const traj_utils::BsplineConstPtr& msg);  // B-spline回调，用于确认目标轨迹已发布
  
  // ROS节点句柄
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  
  // 订阅器和发布器
  ros::Subscriber state_sub_;
  ros::Subscriber target_odom_sub_;  // 订阅目标odom
  ros::Subscriber broadcast_bspline_sub_;  // 订阅 B-spline 轨迹，用于确认目标数据已发布
  ros::Publisher trigger_pub_;
  ros::Publisher waypoint_pub_;  // 直接发布waypoint消息（替代waypoint_generator）
  
  // 参数
  int drone_id_;
  std::string trigger_topic_;
  std::string target_odom_topic_;  // 目标odom话题
  std::string waypoint_topic_;  // waypoint发布话题
  double traj_duration_;  // TRAJ状态持续时间（秒），到时间后自动发送END_TRAJ命令
  double end_traj_wait_time_;  // END_TRAJ状态等待时间（秒），用于日志显示
  
  // 订阅器和发布器
  ros::Publisher state_cmd_pub_;  // 发布状态命令（用于发送END_TRAJ）
  
  // 状态变量
  int last_state_;
  bool trigger_sent_;
  bool in_traj_state_;
  bool traj_completed_;  // 标志：TRAJ任务是否已完成（一旦完成，不再允许重新进入TRAJ）
  ros::Time traj_start_time_;  // TRAJ状态开始时间
  ros::Timer traj_timer_;  // 定时器，用于检查TRAJ时间
  nav_msgs::Odometry::ConstPtr last_target_odom_;  // 保存最新的目标odom
  bool target_bspline_received_;  // 标志：是否已收到目标 B-spline 轨迹
  std::string broadcast_bspline_topic_;  // B-spline 话题名称
  double wait_for_bspline_timeout_;  // 等待 B-spline 的超时时间（秒）
};

#endif // STATE_LISTENER_NODE_HPP


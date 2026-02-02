#ifndef TARGET_ODOM_TO_BSPLINE_NODE_HPP
#define TARGET_ODOM_TO_BSPLINE_NODE_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <traj_utils/Bspline.h>
#include <Eigen/Dense>
#include <deque>
#include <vector>

/**
 * @brief 目标Odom到B-spline轨迹转换器
 * 
 * 订阅目标物体的odom消息，生成B-spline轨迹并发布给Ego Planner
 * 用于替代原来需要目标无人机运行完整Ego Planner的方式
 */
class TargetOdomToBsplineNode {
public:
  TargetOdomToBsplineNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  ~TargetOdomToBsplineNode() = default;

private:
  void target_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
  void generate_and_publish_bspline();
  
  // ROS节点句柄
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  
  // 订阅器和发布器
  ros::Subscriber target_odom_sub_;
  ros::Publisher bspline_pub_;
  
  // 参数
  std::string target_odom_topic_;
  std::string bspline_topic_;
  int target_drone_id_;  // 目标无人机ID（用于B-spline消息）
  double history_duration_;  // 历史位置保存时长 [s]
  double publish_period_;  // B-spline发布周期 [s]
  int min_points_;  // 生成B-spline所需的最少点数
  
  // 位置历史缓冲区
  struct PositionData {
    Eigen::Vector3d pos;
    ros::Time timestamp;
  };
  std::deque<PositionData> position_history_;
  
  // 定时器
  ros::Timer publish_timer_;
  
  // 状态
  ros::Time last_publish_time_;
  int traj_id_counter_;
};

#endif // TARGET_ODOM_TO_BSPLINE_NODE_HPP





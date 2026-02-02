#include "px4_test_framework/target_odom_to_bspline_node.hpp"
#include <bspline_opt/uniform_bspline.h>
#include <algorithm>
#include <cmath>
#include <ros/ros.h>

using namespace ego_planner;

TargetOdomToBsplineNode::TargetOdomToBsplineNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
  : nh_(nh)
  , nh_private_(nh_private)
  , traj_id_counter_(0)
{
  // 读取参数
  nh_private_.param<std::string>("target_odom_topic", target_odom_topic_, "/target/odom");
  nh_private_.param<std::string>("bspline_topic", bspline_topic_, "/broadcast_bspline");
  nh_private_.param("target_drone_id", target_drone_id_, 1);  // 默认目标无人机ID=1
  nh_private_.param("history_duration", history_duration_, 5.0);  // 保存5秒历史
  nh_private_.param("publish_period", publish_period_, 0.5);  // 每0.5秒发布一次
  nh_private_.param("min_points", min_points_, 10);  // 至少需要10个点才能生成B-spline
  
  ROS_INFO("=== Target Odom to B-spline Converter ===");
  ROS_INFO("Target odom topic: %s", target_odom_topic_.c_str());
  ROS_INFO("B-spline topic: %s", bspline_topic_.c_str());
  ROS_INFO("Target drone ID: %d", target_drone_id_);
  ROS_INFO("History duration: %.1f s", history_duration_);
  ROS_INFO("Publish period: %.1f s", publish_period_);
  ROS_INFO("Min points for B-spline: %d", min_points_);
  
  // 创建订阅器
  target_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
    target_odom_topic_, 10, &TargetOdomToBsplineNode::target_odom_callback, this);
  
  // 创建发布器
  bspline_pub_ = nh_.advertise<traj_utils::Bspline>(bspline_topic_, 10);
  
  // 创建定时器，定期发布B-spline轨迹
  publish_timer_ = nh_.createTimer(
    ros::Duration(publish_period_), 
    [this](const ros::TimerEvent&) { this->generate_and_publish_bspline(); });
  
  last_publish_time_ = ros::Time::now();
  
  ROS_INFO("Target odom to B-spline converter initialized!");
}

void TargetOdomToBsplineNode::target_odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  // 保存位置到历史缓冲区
  PositionData data;
  data.pos << msg->pose.pose.position.x, 
              msg->pose.pose.position.y, 
              msg->pose.pose.position.z;
  data.timestamp = msg->header.stamp;
  
  position_history_.push_back(data);
  
  // 清理过期的历史数据
  ros::Time cutoff_time = data.timestamp - ros::Duration(history_duration_);
  while (!position_history_.empty() && position_history_.front().timestamp < cutoff_time) {
    position_history_.pop_front();
  }
}

void TargetOdomToBsplineNode::generate_and_publish_bspline() {
  // 检查是否有足够的历史数据
  if (position_history_.size() < static_cast<size_t>(min_points_)) {
    ROS_WARN_THROTTLE(5.0, "Not enough position history (%zu < %d), skipping B-spline generation", 
                      position_history_.size(), min_points_);
    return;
  }
  
  // 提取位置点
  std::vector<Eigen::Vector3d> point_set;
  point_set.reserve(position_history_.size());
  
  for (const auto& data : position_history_) {
    point_set.push_back(data.pos);
  }
  
  // 如果点数太少，无法生成有效的B-spline
  if (point_set.size() < 4) {
    ROS_WARN_THROTTLE(5.0, "Too few points (%zu) for B-spline generation", point_set.size());
    return;
  }
  
  // 计算时间间隔（基于历史数据的实际时间差）
  double ts = 0.1;  // 默认时间间隔
  if (position_history_.size() >= 2) {
    double time_span = (position_history_.back().timestamp - position_history_.front().timestamp).toSec();
    if (time_span > 0.01) {  // 避免除零
      ts = time_span / (position_history_.size() - 1);
      ts = std::max(0.05, std::min(0.2, ts));  // 限制在合理范围内
    }
  }
  
  // 生成B-spline控制点
  // 使用简单的均匀采样方法
  // 为了生成有效的B-spline，我们需要至少4个控制点（order=3）
  int num_ctrl_pts = std::min(static_cast<int>(point_set.size()), 20);  // 最多20个控制点
  
  // 如果点数太多，进行下采样
  std::vector<Eigen::Vector3d> sampled_points;
  if (static_cast<int>(point_set.size()) > num_ctrl_pts) {
    // 均匀采样
    double step = static_cast<double>(point_set.size() - 1) / (num_ctrl_pts - 1);
    for (int i = 0; i < num_ctrl_pts; ++i) {
      int idx = static_cast<int>(i * step);
      idx = std::min(idx, static_cast<int>(point_set.size() - 1));
      sampled_points.push_back(point_set[idx]);
    }
  } else {
    sampled_points = point_set;
  }
  
  // 确保至少有4个点
  if (sampled_points.size() < 4) {
    ROS_WARN_THROTTLE(5.0, "Sampled points too few (%zu) for B-spline", sampled_points.size());
    return;
  }
  
  // 计算起点和终点的速度（用于B-spline参数化）
  std::vector<Eigen::Vector3d> start_end_derivatives;
  
  // 起点速度：使用前两个点的差分
  Eigen::Vector3d start_vel = Eigen::Vector3d::Zero();
  if (sampled_points.size() >= 2) {
    start_vel = (sampled_points[1] - sampled_points[0]) / ts;
  }
  
  // 终点速度：使用最后两个点的差分
  Eigen::Vector3d end_vel = Eigen::Vector3d::Zero();
  if (sampled_points.size() >= 2) {
    end_vel = (sampled_points.back() - sampled_points[sampled_points.size() - 2]) / ts;
  }
  
  // 起点和终点加速度设为0
  start_end_derivatives.push_back(start_vel);
  start_end_derivatives.push_back(end_vel);
  start_end_derivatives.push_back(Eigen::Vector3d::Zero());  // 起点加速度
  start_end_derivatives.push_back(Eigen::Vector3d::Zero());  // 终点加速度
  
  // 生成B-spline控制点
  Eigen::MatrixXd ctrl_pts;
  try {
    ego_planner::UniformBspline::parameterizeToBspline(ts, sampled_points, start_end_derivatives, ctrl_pts);
  } catch (const std::exception& e) {
    ROS_ERROR_THROTTLE(5.0, "Failed to parameterize B-spline: %s", e.what());
    return;
  }
  
  // 创建B-spline对象以获取knots
  ego_planner::UniformBspline pos_traj(ctrl_pts, 3, ts);
  Eigen::VectorXd knots = pos_traj.getKnot();
  
  // 构建B-spline消息
  traj_utils::Bspline bspline;
  bspline.drone_id = target_drone_id_;
  bspline.order = 3;
  bspline.traj_id = traj_id_counter_++;
  bspline.start_time = position_history_.front().timestamp;  // 使用历史数据的起始时间
  bspline.have_yaw = false;
  
  // 填充控制点
  bspline.pos_pts.reserve(ctrl_pts.cols());
  for (int i = 0; i < ctrl_pts.cols(); ++i) {
    geometry_msgs::Point pt;
    pt.x = ctrl_pts(0, i);
    pt.y = ctrl_pts(1, i);
    pt.z = ctrl_pts(2, i);
    bspline.pos_pts.push_back(pt);
  }
  
  // 填充knots
  bspline.knots.reserve(knots.rows());
  for (int i = 0; i < knots.rows(); ++i) {
    bspline.knots.push_back(knots(i));
  }
  
  // 发布B-spline轨迹
  bspline_pub_.publish(bspline);
  
  ROS_DEBUG_THROTTLE(2.0, "Published B-spline trajectory: %d control points, duration=%.2f s", 
                     static_cast<int>(ctrl_pts.cols()), 
                     (position_history_.back().timestamp - position_history_.front().timestamp).toSec());
}


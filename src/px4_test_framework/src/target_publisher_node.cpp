#include "px4_test_framework/target_publisher_node.hpp"
#include <bspline_opt/uniform_bspline.h>
#include <traj_utils/Bspline.h>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

TargetPublisherNode::TargetPublisherNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
  : nh_(nh)
  , nh_private_(nh_private)
  , trajectory_started_(false)
  , drone_id_(0)
  , current_state_(-1)
  , in_traj_state_(false)
{
  // 读取参数
  nh_private_.param("drone_id", drone_id_, 0);  // 无人机ID，用于订阅状态机状态
  int mode_int;
  nh_private_.param("mode", mode_int, 1);  // 默认圆周模式
  mode_ = static_cast<TrajectoryMode>(mode_int);
  
  nh_private_.param("timer_period", timer_period_, 0.02);  // 50Hz
  
  // 静止模式参数
  nh_private_.param("static_x", static_x_, 2.0);
  nh_private_.param("static_y", static_y_, 0.0);
  nh_private_.param("static_z", static_z_, 1.5);
  
  // 圆周模式参数
  nh_private_.param("circle_radius", circle_radius_, 2.0);
  nh_private_.param("circle_center_x", circle_center_x_, 0.0);
  nh_private_.param("circle_center_y", circle_center_y_, 0.0);
  nh_private_.param("circle_center_z", circle_center_z_, 1.5);
  nh_private_.param("circle_duration", circle_duration_, 20.0);
  nh_private_.param("circle_init_phase", circle_init_phase_, 0.0);
  nh_private_.param("circle_times", circle_times_, 1);
  
  nh_private_.param("ramp_up_time", ramp_up_time_, 3.0);
  nh_private_.param("ramp_down_time", ramp_down_time_, 3.0);
  nh_private_.param("stationary_time", stationary_time_, 3.0);
  
  nh_.param("use_sim_time", use_sim_time_, false);
  
  // 参数验证
  if (circle_times_ < 1) {
    ROS_WARN("circle_times must be >= 1, setting to 1");
    circle_times_ = 1;
  }
  
  // 计算轨迹参数（仅用于圆周模式）
  if (mode_ == TrajectoryMode::CIRCLE) {
    effective_duration_ = calculate_effective_duration();
    max_angular_vel_ = 2.0 * M_PI / effective_duration_;
    angular_acceleration_ = max_angular_vel_ / ramp_up_time_;
    
    double theta_ramp_up = 0.5 * max_angular_vel_ * ramp_up_time_;
    double theta_ramp_down = 0.5 * max_angular_vel_ * ramp_down_time_;
    double theta_ramps_total = theta_ramp_up + theta_ramp_down;
    double theta_required = circle_times_ * 2.0 * M_PI;
    double theta_constant = theta_required - theta_ramps_total;
    total_constant_duration_ = theta_constant / max_angular_vel_;
  }
  
  // 打印配置信息
  ROS_INFO("=== Target Publisher Node ===");
  std::string mode_str;
  switch (mode_) {
    case TrajectoryMode::STATIC:
      mode_str = "STATIC";
      ROS_INFO("Static position: [%.2f, %.2f, %.2f] m", static_x_, static_y_, static_z_);
      break;
    case TrajectoryMode::CIRCLE:
      mode_str = "CIRCLE";
      ROS_INFO("Circle radius: %.2f m", circle_radius_);
      ROS_INFO("Circle center: [%.2f, %.2f, %.2f] m", 
               circle_center_x_, circle_center_y_, circle_center_z_);
      ROS_INFO("Number of circles: %d", circle_times_);
      ROS_INFO("Motion duration: %.2f s", 
               ramp_up_time_ + total_constant_duration_ + ramp_down_time_);
      break;
    case TrajectoryMode::FIGURE_EIGHT:
      mode_str = "FIGURE_EIGHT";
      break;
  }
  ROS_INFO("Trajectory mode: %s", mode_str.c_str());
  ROS_INFO("Publish frequency: %.0f Hz", 1.0/timer_period_);
  ROS_INFO("Clock mode: %s", use_sim_time_ ? "SIM_TIME" : "SYSTEM_TIME");
  
  // 创建发布器 - 根据方法需求发布不同格式
  // visPlanner 需要 nav_msgs/Odometry 类型的 target 话题
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/target/odom", 10);
  // 其他格式
  position_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/target/position", 10);
  velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/target/velocity", 10);
  
  // 同时发布到 visPlanner 期望的话题名（如果不同）
  nh_private_.param<std::string>("elastic_target_topic", elastic_target_topic_, "/target/odom");
  elastic_target_pub_ = nh_.advertise<nav_msgs::Odometry>(elastic_target_topic_, 10);
  
  // 注意：不再发布 B-spline，由 Predictor 负责预测轨迹并发布 B-spline
  
  // 订阅状态机状态
  std::string state_topic = "/state/state_drone_" + std::to_string(drone_id_);
  state_sub_ = nh_.subscribe<std_msgs::Int32>(
    state_topic, 10, &TargetPublisherNode::state_callback, this);
  
  // 创建定时器
  timer_ = nh_.createTimer(ros::Duration(timer_period_), 
                          &TargetPublisherNode::timer_callback, this);
  
  // 不再有 B-spline 定时器
  
  ROS_INFO("Subscribed to state machine: %s", state_topic.c_str());
  ROS_INFO("Target publisher node initialized successfully!");
  ROS_INFO("Target will remain at initial position until TRAJ state is reached.");
}

void TargetPublisherNode::state_callback(const std_msgs::Int32::ConstPtr& msg) {
  int new_state = msg->data;
  
  // 检测状态变化
  if (new_state != current_state_) {
    ROS_INFO("State machine state changed: %d -> %d", current_state_, new_state);
    current_state_ = new_state;
    
    // 检查是否进入 TRAJ 状态（状态值 5）
    if (new_state == 5 && !in_traj_state_) {
      in_traj_state_ = true;
      if (use_sim_time_) {
        traj_entry_time_ = ros::Time::now();
      } else {
        traj_entry_time_system_ = std::chrono::steady_clock::now();
      }
      
      // 只发布 odom，不再发布 B-spline（由 Predictor 负责）
      ROS_INFO("✅ Entered TRAJ state - Target odometry will be published for Predictor");
      ROS_INFO("Target will start motion after %.1f seconds", stationary_time_);
    }
    
    // 检查是否离开 TRAJ 状态
    if (new_state != 5 && in_traj_state_) {
      in_traj_state_ = false;
      ROS_INFO("❌ Exited TRAJ state");
    }
  }
}

double TargetPublisherNode::calculate_effective_duration() {
  return circle_duration_;
}

double TargetPublisherNode::calculate_theta_at_time(double t) {
  double theta = 0.0;
  double omega_max = max_angular_vel_;
  double alpha = angular_acceleration_;
  double alpha_down = omega_max / ramp_down_time_;
  double t_up = ramp_up_time_;
  double t_const = total_constant_duration_;
  double t_down = ramp_down_time_;
  
  if (t <= t_up) {
    // 加速阶段
    theta = 0.5 * alpha * t * t;
  } else if (t <= t_up + t_const) {
    // 匀速阶段
    double theta_at_t_up = 0.5 * alpha * t_up * t_up;
    double dt = t - t_up;
    theta = theta_at_t_up + omega_max * dt;
  } else if (t <= t_up + t_const + t_down) {
    // 减速阶段
    double theta_at_t_up = 0.5 * alpha * t_up * t_up;
    double theta_at_start_down = theta_at_t_up + omega_max * t_const;
    double t_start_down = t_up + t_const;
    double dt = t - t_start_down;
    theta = theta_at_start_down + omega_max * dt - 0.5 * alpha_down * dt * dt;
  } else {
    // 运动完成
    double theta_at_t_up = 0.5 * alpha * t_up * t_up;
    double theta_at_start_down = theta_at_t_up + omega_max * t_const;
    theta = theta_at_start_down + omega_max * t_down - 0.5 * alpha_down * t_down * t_down;
  }
  
  return theta;
}

double TargetPublisherNode::calculate_angular_velocity_at_time(double t) {
  double omega_max = max_angular_vel_;
  double alpha = angular_acceleration_;
  double alpha_down = omega_max / ramp_down_time_;
  double t_up = ramp_up_time_;
  double t_const = total_constant_duration_;
  double t_down = ramp_down_time_;
  double t_start_down = t_up + t_const;
  
  double current_omega = 0.0;
  
  if (t <= t_up) {
    current_omega = alpha * t;
  } else if (t <= t_start_down) {
    current_omega = omega_max;
  } else if (t <= t_start_down + t_down) {
    double dt_down = t - t_start_down;
    current_omega = omega_max - alpha_down * dt_down;
    current_omega = std::max(0.0, current_omega);
  } else {
    current_omega = 0.0;
  }
  
  return current_omega;
}

void TargetPublisherNode::timer_callback(const ros::TimerEvent& event) {
  // 如果不在 TRAJ 状态，始终发布初始位置（静止）
  if (!in_traj_state_) {
    // 发布初始位置（圆周轨迹的起始点）
    double init_x, init_y, init_z;
    if (mode_ == TrajectoryMode::CIRCLE) {
      double theta_initial = circle_init_phase_;
      init_x = circle_center_x_ + circle_radius_ * std::cos(theta_initial);
      init_y = circle_center_y_ + circle_radius_ * std::sin(theta_initial);
      init_z = circle_center_z_;
    } else {
      // 静态模式或8字模式使用静态位置
      init_x = static_x_;
      init_y = static_y_;
      init_z = static_z_;
    }
    
    publish_target_odom(init_x, init_y, init_z, 0.0, 0.0, 0.0);
    publish_target_position(init_x, init_y, init_z);
    publish_target_velocity(0.0, 0.0, 0.0);
    return;
  }
  
  // 在 TRAJ 状态下，计算从进入 TRAJ 状态开始经过的时间
  double elapsed_since_traj = 0.0;
  if (use_sim_time_) {
    elapsed_since_traj = (ros::Time::now() - traj_entry_time_).toSec();
    elapsed_since_traj = std::max(0.0, elapsed_since_traj);
  } else {
    auto current_time_system = std::chrono::steady_clock::now();
    auto elapsed_duration = std::chrono::duration_cast<std::chrono::duration<double>>(
      current_time_system - traj_entry_time_system_);
    elapsed_since_traj = elapsed_duration.count();
  }
  
  // 如果还在静止等待阶段，发布初始位置
  if (elapsed_since_traj < stationary_time_) {
    double theta_initial = circle_init_phase_;
    double init_x = circle_center_x_ + circle_radius_ * std::cos(theta_initial);
    double init_y = circle_center_y_ + circle_radius_ * std::sin(theta_initial);
    double init_z = circle_center_z_;
    
    publish_target_odom(init_x, init_y, init_z, 0.0, 0.0, 0.0);
    publish_target_position(init_x, init_y, init_z);
    publish_target_velocity(0.0, 0.0, 0.0);
    
    ROS_INFO_THROTTLE(2.0, "[TRAJ WAITING] t=%.1f/%.1fs | Target at initial position",
                      elapsed_since_traj, stationary_time_);
    return;
  }
  
  // 静止阶段结束，开始运动轨迹
  // 计算运动时间（减去静止等待时间）
  double motion_time = elapsed_since_traj - stationary_time_;
  
  // 根据模式生成轨迹
  switch (mode_) {
    case TrajectoryMode::STATIC:
      generate_static_trajectory(motion_time);
      break;
    case TrajectoryMode::CIRCLE:
      generate_circular_trajectory(motion_time);
      break;
    case TrajectoryMode::FIGURE_EIGHT:
      generate_figure_eight_trajectory(motion_time);
      break;
  }
}

void TargetPublisherNode::generate_static_trajectory(double t) {
  publish_target_odom(static_x_, static_y_, static_z_, 0.0, 0.0, 0.0);
  publish_target_position(static_x_, static_y_, static_z_);
  publish_target_velocity(0.0, 0.0, 0.0);
  
  ROS_INFO_THROTTLE(2.0, "[STATIC] position=[%.2f, %.2f, %.2f]", 
                    static_x_, static_y_, static_z_);
}

void TargetPublisherNode::generate_circular_trajectory(double t) {
  // t 参数现在是从静止等待阶段结束后的运动时间
  double x, y, z, vx, vy, vz;
  
  // 计算角速度和角度
  double current_omega = calculate_angular_velocity_at_time(t);
  double theta = calculate_theta_at_time(t);
  double theta_with_phase = theta + circle_init_phase_;
  
  // 计算位置和速度
  x = circle_center_x_ + circle_radius_ * std::cos(theta_with_phase);
  y = circle_center_y_ + circle_radius_ * std::sin(theta_with_phase);
  z = circle_center_z_;
  
  double v_linear = current_omega * circle_radius_;
  vx = -v_linear * std::sin(theta_with_phase);
  vy =  v_linear * std::cos(theta_with_phase);
  vz = 0.0;
  
  ROS_INFO_THROTTLE(2.0, "[CIRCULAR] motion_time=%.1fs | position=[%.2f, %.2f, %.2f] | velocity=%.2f m/s",
                    t, x, y, z, v_linear);
  
  publish_target_odom(x, y, z, vx, vy, vz);
  publish_target_position(x, y, z);
  publish_target_velocity(vx, vy, vz);
}

void TargetPublisherNode::generate_figure_eight_trajectory(double t) {
  // 8字轨迹实现（待补充）
  ROS_WARN_ONCE("Figure-eight trajectory not implemented yet, using static mode");
  generate_static_trajectory(t);
}

void TargetPublisherNode::publish_target_odom(double x, double y, double z,
                                               double vx, double vy, double vz) {
  nav_msgs::Odometry msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.child_frame_id = "target";
  
  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.position.z = z;
  msg.pose.pose.orientation.w = 1.0;
  msg.pose.pose.orientation.x = 0.0;
  msg.pose.pose.orientation.y = 0.0;
  msg.pose.pose.orientation.z = 0.0;
  
  msg.twist.twist.linear.x = vx;
  msg.twist.twist.linear.y = vy;
  msg.twist.twist.linear.z = vz;
  
  // 发布到通用话题
  odom_pub_.publish(msg);
  
  // 同时发布到visPlanner专用话题（如果不同）
  if (elastic_target_topic_ != "/target/odom") {
    elastic_target_pub_.publish(msg);
  }
}

void TargetPublisherNode::publish_target_position(double x, double y, double z) {
  geometry_msgs::PointStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.point.x = x;
  msg.point.y = y;
  msg.point.z = z;
  
  position_pub_.publish(msg);
}

void TargetPublisherNode::publish_target_velocity(double vx, double vy, double vz) {
  geometry_msgs::TwistStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.twist.linear.x = vx;
  msg.twist.linear.y = vy;
  msg.twist.linear.z = vz;
  
  velocity_pub_.publish(msg);
}


// B-spline 生成函数已移除
// 现在由 Predictor 负责接收 /target/odom 并预测生成 B-spline 轨迹


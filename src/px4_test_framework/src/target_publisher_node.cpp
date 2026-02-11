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
  , motion_stopped_(false)
  , dshape_current_arc_length_(0.0)
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
  
  // 通用轨迹参数（8字和D型）
  nh_private_.param("trajectory_size", trajectory_size_, 2.0);
  nh_private_.param("trajectory_center_x", trajectory_center_x_, 0.0);
  nh_private_.param("trajectory_center_y", trajectory_center_y_, 0.0);
  nh_private_.param("trajectory_center_z", trajectory_center_z_, 1.5);
  nh_private_.param("trajectory_duration", trajectory_duration_, 20.0);
  nh_private_.param("trajectory_times", trajectory_times_, 1);
  
  nh_private_.param("ramp_up_time", ramp_up_time_, 3.0);
  nh_private_.param("ramp_down_time", ramp_down_time_, 3.0);
  nh_private_.param("stationary_time", stationary_time_, 3.0);
  nh_private_.param("motion_duration", motion_duration_, -1.0);  // -1表示无限制
  
  // Velocity-based parameters for D-shape trajectory
  nh_private_.param("max_linear_velocity", max_linear_velocity_, 1.5);
  nh_private_.param("linear_acceleration", linear_acceleration_, 0.4);
  
  // Auto-calculate trajectory parameters for D-shape
  if (mode_ == TrajectoryMode::D_SHAPE) {
    // Pre-calculate arc lengths of each Bezier segment
    calculate_dshape_segment_lengths();
    
    // Use actual total arc length for trajectory planning
    double path_length = dshape_total_arc_length_;
    
    // Calculate ramp times from acceleration
    ramp_up_time_ = max_linear_velocity_ / linear_acceleration_;
    ramp_down_time_ = max_linear_velocity_ / linear_acceleration_;
    
    // Calculate distances during acceleration phases
    double accel_distance = 0.5 * linear_acceleration_ * ramp_up_time_ * ramp_up_time_;
    double decel_distance = 0.5 * linear_acceleration_ * ramp_down_time_ * ramp_down_time_;
    
    // Remaining distance at constant velocity
    double const_distance = path_length - accel_distance - decel_distance;
    
    if (const_distance < 0) {
      // If trajectory too short to reach max velocity, adjust
      ROS_WARN("Trajectory too short to reach max velocity, adjusting parameters");
      double t_total = 2.0 * std::sqrt(path_length / linear_acceleration_);
      ramp_up_time_ = t_total / 2.0;
      ramp_down_time_ = t_total / 2.0;
      trajectory_duration_ = t_total;
    } else {
      // Calculate constant velocity time
      double const_time = const_distance / max_linear_velocity_;
      trajectory_duration_ = ramp_up_time_ + const_time + ramp_down_time_;
    }
    
    ROS_INFO("[D-Shape Arc-Length] total_length=%.2fm, duration=%.2fs, ramp_time=%.2fs, max_vel=%.2fm/s",
             path_length, trajectory_duration_, ramp_up_time_, max_linear_velocity_);
    ROS_INFO("[D-Shape Segments] lengths=[%.2f, %.2f, %.2f, %.2f]m",
             dshape_segment_lengths_[0], dshape_segment_lengths_[1], 
             dshape_segment_lengths_[2], dshape_segment_lengths_[3]);
  }
  
  nh_.param("use_sim_time", use_sim_time_, false);
  
  // 参数验证
  if (circle_times_ < 1) {
    ROS_WARN("circle_times must be >= 1, setting to 1");
    circle_times_ = 1;
  }
  
  // 计算轨迹参数
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
  } else if (mode_ == TrajectoryMode::FIGURE_EIGHT || mode_ == TrajectoryMode::D_SHAPE) {
    effective_duration_ = trajectory_duration_;
    max_angular_vel_ = 2.0 * M_PI / effective_duration_;
    angular_acceleration_ = max_angular_vel_ / ramp_up_time_;
    
    double theta_ramp_up = 0.5 * max_angular_vel_ * ramp_up_time_;
    double theta_ramp_down = 0.5 * max_angular_vel_ * ramp_down_time_;
    double theta_ramps_total = theta_ramp_up + theta_ramp_down;
    double theta_required = trajectory_times_ * 2.0 * M_PI;
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
      ROS_INFO("Trajectory size: %.2f m", trajectory_size_);
      ROS_INFO("Trajectory center: [%.2f, %.2f, %.2f] m", 
               trajectory_center_x_, trajectory_center_y_, trajectory_center_z_);
      ROS_INFO("Number of cycles: %d", trajectory_times_);
      ROS_INFO("Motion duration: %.2f s", 
               ramp_up_time_ + total_constant_duration_ + ramp_down_time_);
      break;
    case TrajectoryMode::D_SHAPE:
      mode_str = "D_SHAPE";
      ROS_INFO("Trajectory size: %.2f m", trajectory_size_);
      ROS_INFO("Trajectory center: [%.2f, %.2f, %.2f] m", 
               trajectory_center_x_, trajectory_center_y_, trajectory_center_z_);
      ROS_INFO("Number of cycles: %d", trajectory_times_);
      ROS_INFO("Motion duration: %.2f s", 
               ramp_up_time_ + total_constant_duration_ + ramp_down_time_);
      break;
  }
  ROS_INFO("Trajectory mode: %s", mode_str.c_str());
  ROS_INFO("Publish frequency: %.0f Hz", 1.0/timer_period_);
  ROS_INFO("Clock mode: %s", use_sim_time_ ? "SIM_TIME" : "SYSTEM_TIME");
  if (motion_duration_ > 0) {
    ROS_INFO("⏱️  Motion duration limit: %.1f seconds (will stop publishing after this)", motion_duration_);
  } else {
    ROS_INFO("Motion duration: unlimited");
  }
  
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
      motion_stopped_ = false;  // 重置停止标志
      if (use_sim_time_) {
        traj_entry_time_ = ros::Time::now();
      } else {
        traj_entry_time_system_ = std::chrono::steady_clock::now();
      }
      
      // 只发布 odom，不再发布 B-spline（由 Predictor 负责）
      ROS_INFO("✅ Entered TRAJ state - Target odometry will be published for Predictor");
      ROS_INFO("Target will start motion after %.1f seconds", stationary_time_);
      if (motion_duration_ > 0) {
        ROS_INFO("⏱️  Will stop publishing after %.1f seconds", motion_duration_);
      }
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
  // 如果运动已停止（超过时长），停止发布
  if (motion_stopped_) {
    ROS_INFO_THROTTLE(5.0, "⏹️  Motion stopped (duration limit reached) - no longer publishing");
    return;
  }
  
  // 如果不在 TRAJ 状态，始终发布初始位置（静止）
  if (!in_traj_state_) {
    // 发布初始位置（轨迹的起始点）
    double init_x, init_y, init_z;
    if (mode_ == TrajectoryMode::CIRCLE) {
      double theta_initial = circle_init_phase_;
      init_x = circle_center_x_ + circle_radius_ * std::cos(theta_initial);
      init_y = circle_center_y_ + circle_radius_ * std::sin(theta_initial);
      init_z = circle_center_z_;
    } else if (mode_ == TrajectoryMode::FIGURE_EIGHT) {
      // Figure-8: 起点在轨迹中心正前方1m
      init_x = trajectory_center_x_ + 1.0;
      init_y = trajectory_center_y_;
      init_z = trajectory_center_z_;
    } else if (mode_ == TrajectoryMode::D_SHAPE) {
      // D-shape: 起点在轨迹中心正前方1m
      init_x = trajectory_center_x_ + 1.0;
      init_y = trajectory_center_y_;
      init_z = trajectory_center_z_;
    } else {
      // 静态模式使用静态位置
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
  
  // 检查是否超过运动时长限制
  if (motion_duration_ > 0 && elapsed_since_traj >= motion_duration_) {
    if (!motion_stopped_) {
      motion_stopped_ = true;
      ROS_WARN("⏱️  Motion duration limit reached (%.1f seconds) - STOPPING publication", motion_duration_);
      ROS_WARN("🛑 Target publisher will no longer publish, planner should detect and exit TRAJ state");
    }
    return;
  }
  
  // 如果还在静止等待阶段，发布初始位置（根据轨迹类型）
  if (elapsed_since_traj < stationary_time_) {
    double init_x, init_y, init_z;
    
    if (mode_ == TrajectoryMode::CIRCLE) {
      double theta_initial = circle_init_phase_;
      init_x = circle_center_x_ + circle_radius_ * std::cos(theta_initial);
      init_y = circle_center_y_ + circle_radius_ * std::sin(theta_initial);
      init_z = circle_center_z_;
    } else if (mode_ == TrajectoryMode::FIGURE_EIGHT) {
      // Figure-8: 起点在轨迹中心正前方1m
      init_x = trajectory_center_x_ + 1.0;
      init_y = trajectory_center_y_;
      init_z = trajectory_center_z_;
    } else if (mode_ == TrajectoryMode::D_SHAPE) {
      // D-shape: 起点在轨迹中心正前方1m
      init_x = trajectory_center_x_ + 1.0;
      init_y = trajectory_center_y_;
      init_z = trajectory_center_z_;
    } else {
      // 静态模式
      init_x = static_x_;
      init_y = static_y_;
      init_z = static_z_;
    }
    
    publish_target_odom(init_x, init_y, init_z, 0.0, 0.0, 0.0);
    publish_target_position(init_x, init_y, init_z);
    publish_target_velocity(0.0, 0.0, 0.0);
    
    ROS_INFO_THROTTLE(2.0, "[TRAJ WAITING] t=%.1f/%.1fs | Target at initial position [%.2f, %.2f, %.2f]",
                      elapsed_since_traj, stationary_time_, init_x, init_y, init_z);
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
    case TrajectoryMode::D_SHAPE:
      generate_dshape_trajectory(motion_time);
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
  double x, y, z, vx, vy, vz;
  
  // 检查运动时长限制
  if (motion_duration_ > 0 && t >= motion_duration_) {
    if (!motion_stopped_) {
      motion_stopped_ = true;
      ROS_INFO("Motion duration limit reached (%.1f s), stopping at current position", motion_duration_);
    }
    return;  // 停止发布
  }
  
  // 计算归一化参数
  double param = calculate_normalized_parameter_at_time(t);
  double param_vel = calculate_parameter_velocity_at_time(t);
  
  // 检查轨迹是否完成
  double total_motion_time = ramp_up_time_ + total_constant_duration_ + ramp_down_time_;
  bool trajectory_complete = (param >= trajectory_times_) || (t >= total_motion_time);
  
  if (trajectory_complete) {
    double final_theta = trajectory_times_ * 2.0 * M_PI;
    double a = trajectory_size_;
    double x_local = a * std::sin(final_theta) * std::cos(final_theta) + 1.0;
    double y_local = a * std::sin(final_theta);
    
    x = trajectory_center_x_ + x_local;
    y = trajectory_center_y_ + y_local;
    z = trajectory_center_z_;
    vx = 0.0; vy = 0.0; vz = 0.0;
    
    publish_target_odom(x, y, z, vx, vy, vz);
    publish_target_position(x, y, z);
    publish_target_velocity(vx, vy, vz);
    
    ROS_INFO_THROTTLE(2.0, "[FIGURE8-COMPLETE] cycle=%.2f/%d | HOLDING",
                         param, trajectory_times_);
    return;
  }
  
  double theta = param * 2.0 * M_PI;
  double theta_dot = param_vel * 2.0 * M_PI;
  
  double a = trajectory_size_;
  double x_local = a * std::sin(theta) * std::cos(theta) + 1.0;
  double y_local = a * std::sin(theta);
  
  x = trajectory_center_x_ + x_local;
  y = trajectory_center_y_ + y_local;
  z = trajectory_center_z_;
  
  double vx_local = a * std::cos(2.0 * theta) * theta_dot;
  double vy_local = a * std::cos(theta) * theta_dot;
  
  vx = vx_local;
  vy = vy_local;
  vz = 0.0;
  
  double v_linear = std::sqrt(vx*vx + vy*vy);
  ROS_INFO_THROTTLE(2.0, "[FIGURE8] t=%.1fs | cycle=%.2f/%d | v=%.2f m/s",
                    t, param, trajectory_times_, v_linear);
  
  publish_target_odom(x, y, z, vx, vy, vz);
  publish_target_position(x, y, z);
  publish_target_velocity(vx, vy, vz);
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

double TargetPublisherNode::calculate_normalized_parameter_at_time(double t) {
  // This calculates a normalized parameter from 0 to 1 over the trajectory
  // Similar to calculate_theta_at_time but returns normalized value
  double param = 0.0;
  double param_max = 1.0;  // Maximum parameter value (1 complete cycle)
  double alpha = param_max / (ramp_up_time_ * ramp_up_time_);  // Acceleration coefficient
  double alpha_down = param_max / (ramp_down_time_ * ramp_down_time_);  // Deceleration coefficient
  double param_vel_max = param_max / ramp_up_time_;  // Maximum parameter velocity during ramp up
  
  // Recalculate for multi-cycle trajectories
  double total_param_required = trajectory_times_;
  param_max = total_param_required / (ramp_up_time_ + total_constant_duration_ + ramp_down_time_) * ramp_up_time_;
  alpha = param_max / ramp_up_time_;
  param_vel_max = alpha * ramp_up_time_;
  
  double t_up = ramp_up_time_;
  double t_const = total_constant_duration_;  
  double t_down = ramp_down_time_;
  
  // Phase 1: Acceleration
  if (t <= t_up) {
    param = 0.5 * alpha * t * t / t_up;
  }
  // Phase 2: Constant velocity
  else if (t <= t_up + t_const) {
    double param_at_t_up = 0.5 * alpha * t_up;
    double dt = t - t_up;
    param = param_at_t_up + param_vel_max * dt;
  }
  // Phase 3: Deceleration
  else if (t <= t_up + t_const + t_down) {
    double param_at_t_up = 0.5 * alpha * t_up;
    double param_at_start_down = param_at_t_up + param_vel_max * t_const;
    
    double t_start_down = t_up + t_const;
    double dt = t - t_start_down;
    double alpha_down_actual = param_vel_max / t_down;
    param = param_at_start_down + param_vel_max * dt - 0.5 * alpha_down_actual * dt * dt;
  }
  // Phase 4: Maintain final position
  else {
    param = total_param_required;
  }
  
  return param;
}

double TargetPublisherNode::calculate_parameter_velocity_at_time(double t) {
  // Calculate the rate of change of the normalized parameter
  double param_max = 1.0;
  double alpha = param_max / ramp_up_time_;
  double param_vel_max = alpha * ramp_up_time_;
  
  double t_up = ramp_up_time_;
  double t_const = total_constant_duration_;
  double t_down = ramp_down_time_;
  double t_start_down = t_up + t_const;
  
  double current_param_vel = 0.0;

  if (t <= t_up) {
    // Acceleration phase
    current_param_vel = alpha * t;
  }
  else if (t <= t_start_down) {
    // Constant velocity phase
    current_param_vel = param_vel_max;
  }
  else if (t <= t_start_down + t_down) {
    // Deceleration phase
    double dt_down = t - t_start_down;
    double alpha_down_actual = param_vel_max / t_down;
    current_param_vel = param_vel_max - alpha_down_actual * dt_down;
    current_param_vel = std::max(0.0, current_param_vel);
  }
  else {
    // Motion complete
    current_param_vel = 0.0;
  }
  
  return current_param_vel;
}

void TargetPublisherNode::generate_dshape_trajectory(double t) {
  double x, y, z, vx, vy, vz;
  
  // 检查运动时长限制
  if (motion_duration_ > 0 && t >= motion_duration_) {
    if (!motion_stopped_) {
      motion_stopped_ = true;
      ROS_INFO("Motion duration limit reached (%.1f s), stopping at current position", motion_duration_);
    }
    return;  // 停止发布
  }
  
  // Phase 2: D-shape with arc-length based tracking
  // NEW APPROACH: Calculate target arc length based on velocity profile,
  // then find position on curve at that arc length
  double motion_time = t;
  
  // Calculate current linear velocity based on acceleration profile
  double current_velocity = 0.0;
  double t_up = ramp_up_time_;
  double t_const = total_constant_duration_;
  double t_down = ramp_down_time_;
  std::string phase;
  
  if (motion_time <= t_up) {
    // Acceleration phase
    current_velocity = linear_acceleration_ * motion_time;
    phase = "ACCELERATING";
  } else if (motion_time <= t_up + t_const) {
    // Constant velocity phase
    current_velocity = max_linear_velocity_;
    phase = "CONSTANT";
  } else if (motion_time <= t_up + t_const + t_down) {
    // Deceleration phase
    double t_in_decel = motion_time - t_up - t_const;
    current_velocity = max_linear_velocity_ - linear_acceleration_ * t_in_decel;
    phase = "DECELERATING";
  } else {
    // Complete
    current_velocity = 0.0;
    phase = "COMPLETE";
  }
  
  // Calculate target arc length (cumulative distance traveled)
  double target_arc_length = 0.0;
  if (motion_time <= t_up) {
    // s = 0.5 * a * t²
    target_arc_length = 0.5 * linear_acceleration_ * motion_time * motion_time;
  } else if (motion_time <= t_up + t_const) {
    double s_accel = 0.5 * linear_acceleration_ * t_up * t_up;
    double dt = motion_time - t_up;
    target_arc_length = s_accel + max_linear_velocity_ * dt;
  } else if (motion_time <= t_up + t_const + t_down) {
    double s_accel = 0.5 * linear_acceleration_ * t_up * t_up;
    double s_const = max_linear_velocity_ * t_const;
    double t_in_decel = motion_time - t_up - t_const;
    target_arc_length = s_accel + s_const + 
                        max_linear_velocity_ * t_in_decel - 
                        0.5 * linear_acceleration_ * t_in_decel * t_in_decel;
  } else {
    target_arc_length = dshape_total_arc_length_ * trajectory_times_;
  }
  
  // Handle multiple cycles
  int current_cycle = static_cast<int>(target_arc_length / dshape_total_arc_length_);
  double arc_length_in_cycle = target_arc_length - current_cycle * dshape_total_arc_length_;
  
  // Check if trajectory is complete
  if (current_cycle >= trajectory_times_ || motion_time >= t_up + t_const + t_down) {
    // Hold at final position (back at start point)
    x = trajectory_center_x_ + 1.0;
    y = trajectory_center_y_;
    z = trajectory_center_z_;
    vx = 0.0;
    vy = 0.0;
    vz = 0.0;
    
    publish_target_odom(x, y, z, vx, vy, vz);
    publish_target_position(x, y, z);
    publish_target_velocity(vx, vy, vz);
    
    ROS_INFO_THROTTLE(2.0, "[DSHAPE-COMPLETE] t=%.1fs | cycle=%d/%d | arc_length=%.2f/%.2fm | HOLDING",
                         t, current_cycle + 1, trajectory_times_, target_arc_length, 
                         dshape_total_arc_length_ * trajectory_times_);
    return;
  }
  
  // Find which segment we're in based on arc length
  double a = trajectory_size_ / 2.0;
  double offset_x = 1.0;
  double straight_x = offset_x - 2.0 * a;
  
  int segment = 0;
  double arc_length_in_segment = arc_length_in_cycle;
  for (int i = 0; i < 4; ++i) {
    if (arc_length_in_cycle <= dshape_cumulative_lengths_[i + 1]) {
      segment = i;
      arc_length_in_segment = arc_length_in_cycle - dshape_cumulative_lengths_[i];
      break;
    }
  }
  
  // Get control points for current segment
  double P0x, P0y, P1x, P1y, P2x, P2y, P3x, P3y;
  
  if (segment == 0) {
    P0x = offset_x;              P0y = 0.0;
    P1x = offset_x;              P1y = a * 0.55;
    P2x = offset_x - a * 0.2;    P2y = a;
    P3x = offset_x - a;          P3y = a;
  } else if (segment == 1) {
    P0x = offset_x - a;          P0y = a;
    P1x = offset_x - a * 1.8;    P1y = a;
    P2x = straight_x;            P2y = a;
    P3x = straight_x;            P3y = a * 0.5;
  } else if (segment == 2) {
    P0x = straight_x;            P0y = a * 0.5;
    P1x = straight_x;            P1y = 0.0;
    P2x = straight_x;            P2y = -a * 0.5;
    P3x = straight_x;            P3y = -a;
  } else {
    P0x = straight_x;            P0y = -a;
    P1x = offset_x - a * 0.5;    P1y = -a;
    P2x = offset_x;              P2y = -a * 0.5;
    P3x = offset_x;              P3y = 0.0;
  }
  
  // Find parameter t on this segment that corresponds to the target arc length
  double t_local = find_bezier_parameter_from_arc_length(arc_length_in_segment, 
                                                          P0x, P0y, P1x, P1y, 
                                                          P2x, P2y, P3x, P3y);
  
  // Calculate position using Bezier formula
  double mt = 1.0 - t_local;
  double t2 = t_local * t_local;
  double mt2 = mt * mt;
  
  double x_local = mt * mt * mt * P0x + 3.0 * mt2 * t_local * P1x + 
                   3.0 * mt * t2 * P2x + t2 * t_local * P3x;
  double y_local = mt * mt * mt * P0y + 3.0 * mt2 * t_local * P1y + 
                   3.0 * mt * t2 * P2y + t2 * t_local * P3y;
  
  x = trajectory_center_x_ + x_local;
  y = trajectory_center_y_ + y_local;
  z = trajectory_center_z_;
  
  // Calculate velocity direction from Bezier tangent
  double vx_local = 3.0 * (mt2 * (P1x - P0x) + 
                           2.0 * mt * t_local * (P2x - P1x) + 
                           t2 * (P3x - P2x));
  double vy_local = 3.0 * (mt2 * (P1y - P0y) + 
                           2.0 * mt * t_local * (P2y - P1y) + 
                           t2 * (P3y - P2y));
  
  // Normalize tangent and multiply by current velocity
  double tangent_magnitude = std::sqrt(vx_local * vx_local + vy_local * vy_local);
  if (tangent_magnitude > 1e-6) {
    vx = (vx_local / tangent_magnitude) * current_velocity;
    vy = (vy_local / tangent_magnitude) * current_velocity;
  } else {
    vx = 0.0;
    vy = 0.0;
  }
  vz = 0.0;
  
  // Publish
  publish_target_odom(x, y, z, vx, vy, vz);
  publish_target_position(x, y, z);
  publish_target_velocity(vx, vy, vz);
  
  // Debug log
  double v_linear = std::sqrt(vx*vx + vy*vy);
  ROS_INFO_THROTTLE(2.0, "[DSHAPE-%s] t=%.1fs | seg=%d | arc=%.2f/%.2fm | v=%.2fm/s",
                       phase.c_str(), t, segment, arc_length_in_cycle, dshape_total_arc_length_, v_linear);
}
  
  // 计算归一化参数
  double param = calculate_normalized_parameter_at_time(t);
  double param_vel = calculate_parameter_velocity_at_time(t);
  
  // 检查轨迹是否完成
  double total_motion_time = ramp_up_time_ + total_constant_duration_ + ramp_down_time_;
  bool trajectory_complete = (param >= trajectory_times_) || (t >= total_motion_time);
  
  if (trajectory_complete) {
    x = trajectory_center_x_ + 1.0;
    y = trajectory_center_y_;
    z = trajectory_center_z_;
    vx = 0.0; vy = 0.0; vz = 0.0;
    
    publish_target_odom(x, y, z, vx, vy, vz);
    publish_target_position(x, y, z);
    publish_target_velocity(vx, vy, vz);
    
    ROS_INFO_THROTTLE(2.0, "[DSHAPE-COMPLETE] cycle=%.2f/%d | HOLDING",
                         param, trajectory_times_);
    return;
  }
  
  // D字型重新设计：更清晰的几何，左侧直边，右侧弧边
  // 起点在右侧 (1.0, 0)，左侧直边在 x = 1.0 - 2*a
  double a = trajectory_size_ / 2.0;  // 半宽/半径（基础 trajectory_size = 2.0）
  
  // D字型分段：
  // 0: 右下角（底部到中右）
  // 1: 右上角（中右到顶部）
  // 2: 左侧直边（顶部到底部）- 最长段
  // 3: 底部过渡（左下回到起点）
  
  // 时间分配：拐弯更缓和（增加拐弯段时间，降低拐弯速度）
  double segment_ratios[4] = {0.12, 0.20, 0.38, 0.30};  // [右下, 右上, 左直, 底部]
  double cumulative[5] = {0.0, 0.12, 0.32, 0.70, 1.0};
  
  // 找到当前段
  int segment = 0;
  double param_normalized = param - std::floor(param);  // 归一化到单周期 [0,1]
  for (int i = 0; i < 4; ++i) {
    if (param_normalized <= cumulative[i + 1]) {
      segment = i;
      break;
    }
  }
  
  // 段内局部参数 [0, 1]
  double t_local = (param_normalized - cumulative[segment]) / segment_ratios[segment];
  t_local = std::min(1.0, std::max(0.0, t_local));  // 限制到 [0,1]
  double t_local_dot = param_vel / segment_ratios[segment];
  
  // 控制点（相对于 trajectory_center）
  // offset_x = 1.0（起点），straight_x = 1.0 - 2*a（左边缘）
  double P0x, P0y, P1x, P1y, P2x, P2y, P3x, P3y;
  double offset_x = 1.0;
  double straight_x = offset_x - 2.0 * a;
  
  if (segment == 0) {
    // 段0：从右中 (1.0, 0) 向上弯曲
    P0x = offset_x;              P0y = 0.0;
    P1x = offset_x;              P1y = a * 0.55;
    P2x = offset_x - a * 0.2;    P2y = a;
    P3x = offset_x - a;          P3y = a;  // 到达顶部中间
  } else if (segment == 1) {
    // 段1：从顶部中间向左弯曲到直边顶点
    P0x = offset_x - a;          P0y = a;
    P1x = offset_x - a * 1.8;    P1y = a;
    P2x = straight_x;            P2y = a;
    P3x = straight_x;            P3y = a * 0.5;  // 进入直边下降段
  } else if (segment == 2) {
    // 段2：左侧直边（从 a*0.5 降到 -a）
    P0x = straight_x;            P0y = a * 0.5;
    P1x = straight_x;            P1y = 0.0;
    P2x = straight_x;            P2y = -a * 0.5;
    P3x = straight_x;            P3y = -a;
  } else {
    // 段3：从左下角绕回起点
    P0x = straight_x;            P0y = -a;
    P1x = offset_x - a * 0.5;    P1y = -a;
    P2x = offset_x;              P2y = -a * 0.5;
    P3x = offset_x;              P3y = 0.0;  // 回到起点
  }
  
  // 三次贝塞尔计算
  double mt = 1.0 - t_local;
  double t2 = t_local * t_local;
  double mt2 = mt * mt;
  
  double x_local = mt * mt * mt * P0x + 3.0 * mt2 * t_local * P1x + 
                   3.0 * mt * t2 * P2x + t2 * t_local * P3x;
  double y_local = mt * mt * mt * P0y + 3.0 * mt2 * t_local * P1y + 
                   3.0 * mt * t2 * P2y + t2 * t_local * P3y;
  
  x = trajectory_center_x_ + x_local;
  y = trajectory_center_y_ + y_local;
  z = trajectory_center_z_;
  
  // 速度：dB/dt = 3[(1-t)²(P₁-P₀) + 2(1-t)t(P₂-P₁) + t²(P₃-P₂)] * dt/dparam
  double vx_local = 3.0 * (mt2 * (P1x - P0x) + 
                           2.0 * mt * t_local * (P2x - P1x) + 
                           t2 * (P3x - P2x)) * t_local_dot;
  double vy_local = 3.0 * (mt2 * (P1y - P0y) + 
                           2.0 * mt * t_local * (P2y - P1y) + 
                           t2 * (P3y - P2y)) * t_local_dot;
  
  vx = vx_local;
  vy = vy_local;
  vz = 0.0;
  
  double v_linear = std::sqrt(vx*vx + vy*vy);
  ROS_INFO_THROTTLE(2.0, "[DSHAPE-%s] t=%.1fs | seg=%d | arc=%.2f/%.2fm | v=%.2fm/s",
                       phase.c_str(), t, segment, arc_length_in_cycle, dshape_total_arc_length_, v_linear);
  
  publish_target_odom(x, y, z, vx, vy, vz);
  publish_target_position(x, y, z);
  publish_target_velocity(vx, vy, vz);
}

double TargetPublisherNode::calculate_bezier_arc_length(double P0x, double P0y, double P1x, double P1y,
                                                          double P2x, double P2y, double P3x, double P3y,
                                                          double t_end)
{
  // Calculate arc length of cubic Bezier curve using numerical integration
  const int num_samples = 100;
  double arc_length = 0.0;
  
  double prev_x = P0x;
  double prev_y = P0y;
  
  int end_sample = static_cast<int>(t_end * num_samples);
  
  for (int i = 1; i <= end_sample; ++i) {
    double t = static_cast<double>(i) / num_samples;
    double mt = 1.0 - t;
    double t2 = t * t;
    double mt2 = mt * mt;
    
    // Calculate position at parameter t
    double x = mt * mt * mt * P0x + 3.0 * mt2 * t * P1x + 
               3.0 * mt * t2 * P2x + t2 * t * P3x;
    double y = mt * mt * mt * P0y + 3.0 * mt2 * t * P1y + 
               3.0 * mt * t2 * P2y + t2 * t * P3y;
    
    // Accumulate distance
    double dx = x - prev_x;
    double dy = y - prev_y;
    arc_length += std::sqrt(dx * dx + dy * dy);
    
    prev_x = x;
    prev_y = y;
  }
  
  return arc_length;
}

double TargetPublisherNode::find_bezier_parameter_from_arc_length(double target_length,
                                                                   double P0x, double P0y, double P1x, double P1y,
                                                                   double P2x, double P2y, double P3x, double P3y)
{
  // Use binary search to find parameter t that gives target arc length
  if (target_length <= 0.0) {
    return 0.0;
  }
  
  // Get total length
  double total_length = calculate_bezier_arc_length(P0x, P0y, P1x, P1y, P2x, P2y, P3x, P3y, 1.0);
  
  if (target_length >= total_length) {
    return 1.0;
  }
  
  // Binary search
  double t_low = 0.0;
  double t_high = 1.0;
  const double tolerance = 0.001;  // Arc length tolerance in meters
  const int max_iterations = 20;
  
  for (int iter = 0; iter < max_iterations; ++iter) {
    double t_mid = (t_low + t_high) / 2.0;
    double arc_at_mid = calculate_bezier_arc_length(P0x, P0y, P1x, P1y, P2x, P2y, P3x, P3y, t_mid);
    
    if (std::abs(arc_at_mid - target_length) < tolerance) {
      return t_mid;
    }
    
    if (arc_at_mid < target_length) {
      t_low = t_mid;
    } else {
      t_high = t_mid;
    }
  }
  
  return (t_low + t_high) / 2.0;
}

void TargetPublisherNode::calculate_dshape_segment_lengths()
{
  // Pre-calculate arc lengths for each Bezier segment
  double a = trajectory_size_ / 2.0;
  double offset_x = 1.0;
  double straight_x = offset_x - 2.0 * a;
  
  // Segment 0
  double P0x = offset_x;              double P0y = 0.0;
  double P1x = offset_x;              double P1y = a * 0.55;
  double P2x = offset_x - a * 0.2;    double P2y = a;
  double P3x = offset_x - a;          double P3y = a;
  dshape_segment_lengths_[0] = calculate_bezier_arc_length(P0x, P0y, P1x, P1y, P2x, P2y, P3x, P3y);
  
  // Segment 1
  P0x = offset_x - a;          P0y = a;
  P1x = offset_x - a * 1.8;    P1y = a;
  P2x = straight_x;            P2y = a;
  P3x = straight_x;            P3y = a * 0.5;
  dshape_segment_lengths_[1] = calculate_bezier_arc_length(P0x, P0y, P1x, P1y, P2x, P2y, P3x, P3y);
  
  // Segment 2
  P0x = straight_x;            P0y = a * 0.5;
  P1x = straight_x;            P1y = 0.0;
  P2x = straight_x;            P2y = -a * 0.5;
  P3x = straight_x;            P3y = -a;
  dshape_segment_lengths_[2] = calculate_bezier_arc_length(P0x, P0y, P1x, P1y, P2x, P2y, P3x, P3y);
  
  // Segment 3
  P0x = straight_x;            P0y = -a;
  P1x = offset_x - a * 0.5;    P1y = -a;
  P2x = offset_x;              P2y = -a * 0.5;
  P3x = offset_x;              P3y = 0.0;
  dshape_segment_lengths_[3] = calculate_bezier_arc_length(P0x, P0y, P1x, P1y, P2x, P2y, P3x, P3y);
  
  // Calculate cumulative lengths
  dshape_cumulative_lengths_[0] = 0.0;
  dshape_total_arc_length_ = 0.0;
  for (int i = 0; i < 4; ++i) {
    dshape_total_arc_length_ += dshape_segment_lengths_[i];
    dshape_cumulative_lengths_[i + 1] = dshape_total_arc_length_;
  }
}



// B-spline 生成函数已移除
// 现在由 Predictor 负责接收 /target/odom 并预测生成 B-spline 轨迹


// Finite-state machine for PX4 offboard control (ROS 1 + MAVROS)
// Adapted from ROS2 version by Yichao Gao 
// Modified: Added automatic END_TRAJ to LAND transition after 5s wait

#include "offboard_state_machine/offboard_fsm_node.hpp"
#include "offboard_state_machine/utils.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cmath>

/*  MJerkSegment Implementation                                       */

MJerkSegment MJerkSegment::build(
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& v0,
    const Eigen::Vector3d& a0,
    const Eigen::Vector3d& pf,
    const Eigen::Vector3d& vf,
    const Eigen::Vector3d& af,
    double T,
    ros::Time t0)
{
  // Solve quintic polynomial for each axis
  auto solve_axis = [](double p0, double v0, double a0, 
                      double pf, double vf, double af, double T) {
    Eigen::Matrix<double,6,6> M;
    M << 1, 0,   0,    0,     0,      0,
        0, 1,   0,    0,     0,      0,
        0, 0,   2,    0,     0,      0,
        1, T, T*T, T*T*T, T*T*T*T, T*T*T*T*T,
        0, 1, 2*T, 3*T*T, 4*T*T*T, 5*T*T*T*T,
        0, 0,   2,   6*T,  12*T*T,  20*T*T*T;
    
    Eigen::Matrix<double,6,1> b;
    b << p0, v0, a0, pf, vf, af;
    Eigen::Matrix<double,6,1> results = M.fullPivLu().solve(b);
    return results;
  };
  
  MJerkSegment seg;
  seg.ax = solve_axis(p0.x(), v0.x(), a0.x(), pf.x(), vf.x(), af.x(), T);
  seg.ay = solve_axis(p0.y(), v0.y(), a0.y(), pf.y(), vf.y(), af.y(), T);
  seg.az = solve_axis(p0.z(), v0.z(), a0.z(), pf.z(), vf.z(), af.z(), T);
  seg.t0 = t0;
  seg.T = T;
  return seg;
}

void MJerkSegment::sample(const ros::Time& now,
                          Eigen::Vector3d& p,
                          Eigen::Vector3d& v,
                          Eigen::Vector3d& a) const
{
  double t = (now - t0).toSec();
  
  // Clamp negative time due to clock jitter
  if (t < -0.01) {
    t = 0.0;
  }
  
  t = std::clamp(t, 0.0, T);
  
  double t2 = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t;
  
  // Evaluate quintic polynomial
  auto eval = [&](const Eigen::Matrix<double,6,1>& c) {
    double pos = c.coeff(0) + c.coeff(1)*t + c.coeff(2)*t2 + 
                 c.coeff(3)*t3 + c.coeff(4)*t4 + c.coeff(5)*t5;
    double vel = c.coeff(1) + 2.0*c.coeff(2)*t + 3.0*c.coeff(3)*t2 + 
                 4.0*c.coeff(4)*t3 + 5.0*c.coeff(5)*t4;
    double acc = 2.0*c.coeff(2) + 6.0*c.coeff(3)*t + 12.0*c.coeff(4)*t2 + 
                 20.0*c.coeff(5)*t3;
    return std::array<double,3>{pos, vel, acc};
  };
  
  auto rx = eval(ax), ry = eval(ay), rz = eval(az);
  p = {rx[0], ry[0], rz[0]};
  v = {rx[1], ry[1], rz[1]};
  a = {rx[2], ry[2], rz[2]};
}

bool MJerkSegment::finished(const ros::Time& now) const
{
  return (now - t0).toSec() >= T;
}

double MJerkSegment::get_max_velocity() const
{
  double max_vel_sq = 0.0;
  const int num_samples = 100;
  
  for (int i = 0; i <= num_samples; ++i) {
    double t = T * static_cast<double>(i) / static_cast<double>(num_samples);
    double t2 = t*t, t3 = t2*t, t4 = t3*t;
    
    double vx = ax.coeff(1) + 2.0*ax.coeff(2)*t + 3.0*ax.coeff(3)*t2 + 
                4.0*ax.coeff(4)*t3 + 5.0*ax.coeff(5)*t4;
    double vy = ay.coeff(1) + 2.0*ay.coeff(2)*t + 3.0*ay.coeff(3)*t2 + 
                4.0*ay.coeff(4)*t3 + 5.0*ay.coeff(5)*t4;
    double vz = az.coeff(1) + 2.0*az.coeff(2)*t + 3.0*az.coeff(3)*t2 + 
                4.0*az.coeff(4)*t3 + 5.0*az.coeff(5)*t4;
    
    double vel_sq = vx*vx + vy*vy + vz*vz;
    max_vel_sq = std::max(max_vel_sq, vel_sq);
  }
  
  return std::sqrt(max_vel_sq);
}


/*  Constructor                                                       */

OffboardFSM::OffboardFSM(ros::NodeHandle& nh, ros::NodeHandle& nh_private, int drone_id)
: nh_(nh)
, nh_private_(nh_private)
, drone_id_(drone_id)
, initial_arming_complete_(false)
, current_state_(FsmState::INIT)
, offb_counter_(0)
, takeoff_start_count_(0)
, takeoff_complete_count_(-1)
, landing_start_count_(0)
, use_attitude_control_(false)
, odom_ready_(false)
, vel_initialized_(false)
, has_final_setpoint_(false)
, final_setpoint_hold_count_(0)
, hover_x_(0.0)
, hover_y_(0.0)
, hover_z_(1.2)
, offboard_cmd_count_(0)
, arm_cmd_count_(0)
, takeoff_pos_x_(0.0)
, takeoff_pos_y_(0.0)
, landing_x_(0.0)
, landing_y_(0.0)
, landing_start_z_(0.0)
, current_x_(0.0)
, current_y_(0.0)
, current_z_(0.0)
, last_z_(0.0)
{
  // Get parameters
  nh_private_.param("takeoff_alt", takeoff_alt_, 1.2);
  nh_private_.param("takeoff_time", takeoff_time_s_, 3.0);
  nh_private_.param("climb_rate", climb_rate_, 1.0);
  nh_private_.param("landing_time", landing_time_s_, 5.0);
  nh_private_.param("circle_radius", circle_radius_, 1.4);
  nh_private_.param("inward_offset", inward_offset_, 0.80);
  nh_private_.param("num_drones", num_drones_, 6);
  nh_private_.param("timer_period", timer_period_s_, 0.02);
  nh_private_.param("alt_tol", alt_tol_, 0.03);
  nh_private_.param("circle_radius_traj", radius_, 3.0);
  nh_private_.param("circle_period", period_s_, 20.0);
  nh_private_.param("goto_x", goto_x_, std::numeric_limits<double>::quiet_NaN());
  nh_private_.param("goto_y", goto_y_, std::numeric_limits<double>::quiet_NaN());
  nh_private_.param("goto_z", goto_z_, std::numeric_limits<double>::quiet_NaN());
  nh_private_.param("goto_tol", goto_tol_, 0.1);
  nh_private_.param("goto_max_vel", goto_max_vel_, 0.8);
  nh_private_.param("goto_accel_time", goto_accel_time_, 2.0);
  nh_private_.param("landing_max_vel", landing_max_vel_, 0.3);
  nh_private_.param("end_traj_wait_time", end_traj_wait_time_, 5.0);
  traj_position_disabled_ = false;
  nh_private_.param("payload_offset_x", payload_offset_x_, 0.0);
  nh_private_.param("payload_offset_y", payload_offset_y_, 0.0);
  // TRAJ control mode is fixed to ATTITUDE control for SO3 controller
  // SO3 controller uses AttitudeTarget (not VehicleRatesSetpoint/CTBR)
  // No longer reading from config - always use attitude control in TRAJ state
  
  in_goto_transition_ = false;
  goto_duration_ = 0.0;
  goto_start_x_ = 0.0;
  goto_start_y_ = 0.0;
  goto_start_z_ = 0.0;
  
  // Calculate formation position
  double theta    = 2.0 * M_PI * drone_id_ / static_cast<double>(num_drones_);
  double r_target = -inward_offset_;
  takeoff_pos_x_  =  r_target * std::sin(theta) + payload_offset_x_;
  takeoff_pos_y_  =  r_target * std::cos(theta) + payload_offset_y_;
  
  climb_rate_ = takeoff_alt_ / takeoff_time_s_;

  ROS_INFO("FSM drone %d: alt=%.2fm, goto_vel=%.2fm/s, land_vel=%.2fm/s, land_time=%.1fs, wait=%.1fs",
           drone_id_, takeoff_alt_, goto_max_vel_, landing_max_vel_, landing_time_s_, end_traj_wait_time_);
  ROS_INFO("TRAJ control mode: ATTITUDE control - Fixed for SO3 controller (not RATE/CTBR)");

  // Publishers
  setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
      "/mavros/setpoint_raw/local", 10);
  state_pub_ = nh_.advertise<std_msgs::Int32>(
      "/state/state_drone_" + std::to_string(drone_id_), 10);
  
  // Subscribers
  state_sub_ = nh_.subscribe<mavros_msgs::State>(
      "/mavros/state", 10, &OffboardFSM::state_cb, this);
  local_pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      "/mavros/local_position/pose", 10, &OffboardFSM::local_pos_cb, this);
  local_vel_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>(
      "/mavros/local_position/velocity_local", 10, &OffboardFSM::local_vel_cb, this);
  state_cmd_sub_ = nh_.subscribe<std_msgs::Int32>(
      "/state/command_drone_" + std::to_string(drone_id_), 10, 
      &OffboardFSM::state_cmd_cb, this);
  
  // Service clients
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
      "/mavros/cmd/arming");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
      "/mavros/set_mode");
  
  // Timer
  timer_ = nh_.createTimer(ros::Duration(timer_period_s_), 
                           &OffboardFSM::timer_cb, this);
  
  ROS_INFO("Timer: %.0fHz", 1.0/timer_period_s_);
}


/*  Callbacks                                                         */

void OffboardFSM::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_mavros_state_ = *msg;
}

void OffboardFSM::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_x_ = msg->pose.position.x;
  current_y_ = msg->pose.position.y;
  current_z_ = msg->pose.position.z;
  odom_ready_ = true;
}

void OffboardFSM::local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  current_vel_ = Eigen::Vector3d(
      msg->twist.linear.x, 
      msg->twist.linear.y, 
      msg->twist.linear.z);
  vel_initialized_ = true;
}

void OffboardFSM::state_cmd_cb(const std_msgs::Int32::ConstPtr& msg)
{
  int s = msg->data;
  if (s < static_cast<int>(FsmState::INIT) ||
      s > static_cast<int>(FsmState::DONE))
    return;

  FsmState new_state = static_cast<FsmState>(s);

  if (new_state == FsmState::LAND) {
    landing_start_count_ = offb_counter_;
    landing_start_z_     = current_z_;  // ENU: positive altitude
    landing_x_           = current_x_;
    landing_y_           = current_y_;
    
    Eigen::Vector3d p_target(current_x_, current_y_, 0.0);
    start_mjerk_segment(p_target, landing_time_s_, 
                       Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                       landing_max_vel_);
    ROS_INFO("Manual LAND from [%.2f, %.2f, %.2f]", 
             landing_x_, landing_y_, landing_start_z_);
  }

  if (new_state == FsmState::TRAJ) {
    state_start_time_     = ros::Time::now();
    // Fixed to use attitude control for SO3 controller
    // SO3 controller publishes AttitudeTarget to /mavros/setpoint_raw/attitude
    // This is ATTITUDE control (not RATE/CTBR control)
    use_attitude_control_ = true;
    traj_position_disabled_ = false;  // Reset flag to disable position control once
    ROS_INFO("Entering TRAJ state with ATTITUDE control (SO3 controller)");
  }

  if (new_state == FsmState::END_TRAJ) {
    traj_position_disabled_ = false;  // Reset flag when leaving TRAJ state
    hover_x_ = current_x_;
    hover_y_ = current_y_;
    hover_z_ = current_z_;
    ROS_INFO("END_TRAJ: Hover at [%.2f, %.2f, %.2f]",
             hover_x_, hover_y_, hover_z_);
  }

  current_state_ = new_state;
  ROS_WARN("State override to %d", s);
}

bool OffboardFSM::has_goto_target() const
{
  return std::isfinite(goto_x_) &&
         std::isfinite(goto_y_) &&
         std::isfinite(goto_z_);
}


/*  Duration Calculation with Velocity Limiting                      */

double OffboardFSM::calculate_optimal_duration(
    const Eigen::Vector3d& p_start,
    const Eigen::Vector3d& p_target,
    const Eigen::Vector3d& v_start,
    double max_vel) const
{
  double dist = (p_target - p_start).norm();
  
  const double VELOCITY_SCALE_FACTOR = 1.875;
  double duration = VELOCITY_SCALE_FACTOR * dist / max_vel;
  
  double v0_mag = v_start.norm();
  if (v0_mag > 0.1) {
    duration = std::max(duration, (dist + v0_mag * 1.0) / max_vel);
  }
  
  duration = std::max(duration, std::max(goto_accel_time_, 2.0));
  
  const int MAX_ITERATIONS = 15;
  const double VELOCITY_MARGIN = 0.95;
  
  for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
    MJerkSegment test_seg = MJerkSegment::build(
        p_start, v_start, Eigen::Vector3d::Zero(),
        p_target, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        duration, ros::Time(0));
    
    double actual_max_vel = test_seg.get_max_velocity();
    
    if (actual_max_vel <= max_vel * 1.05) {
      ROS_DEBUG("Duration %.2fs, v_max=%.3f (iter %d)",
                duration, actual_max_vel, iter);
      return duration;
    }
    
    double scale = actual_max_vel / (max_vel * VELOCITY_MARGIN);
    duration *= scale;
    
    if (iter % 5 == 0) {
      ROS_DEBUG("Iter %d: v=%.3f, T→%.2fs",
                iter, actual_max_vel, duration);
    }
  }
  
  ROS_WARN("Duration optimization timeout, T=%.2fs", duration);
  return duration;
}


/*  Start Minimum Jerk Segment                                       */

void OffboardFSM::start_mjerk_segment(const Eigen::Vector3d& p_target,
                                      double initial_duration,
                                      const Eigen::Vector3d& v_target,
                                      const Eigen::Vector3d& a_target,
                                      double max_vel_override)
{
  Eigen::Vector3d p0(current_x_, current_y_, current_z_);
  
  if (std::abs(p0.z()) < 0.05) {
    p0.z() = 0.0;
  }
  
  Eigen::Vector3d v0 = vel_initialized_ ? current_vel_ : Eigen::Vector3d::Zero();
  Eigen::Vector3d a0 = Eigen::Vector3d::Zero();
  
  // Use override if provided (positive value), otherwise use goto_max_vel_
  double effective_max_vel = (max_vel_override > 0.0) ? max_vel_override : goto_max_vel_;
  
  double duration = initial_duration;
  const int MAX_ITERATIONS = 15;
  
  for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
    MJerkSegment candidate = MJerkSegment::build(
        p0, v0, a0, p_target, v_target, a_target, duration, ros::Time::now());
    
    double actual_max_vel = candidate.get_max_velocity();
    
    if (actual_max_vel <= effective_max_vel * 1.05) {
      active_seg_ = candidate;
      
      ROS_INFO("Seg [%.2f,%.2f,%.2f]→[%.2f,%.2f,%.2f] T=%.2fs v=%.3f (limit=%.3f)",
               p0.x(), p0.y(), p0.z(), 
               p_target.x(), p_target.y(), p_target.z(), 
               duration, actual_max_vel, effective_max_vel);
      return;
    }
    
    double scale = actual_max_vel / (effective_max_vel * 0.92);
    duration *= scale;
    
    ROS_DEBUG("Iter %d: v=%.3f>%.3f, T→%.2fs",
              iter, actual_max_vel, effective_max_vel, duration);
  }
  
  active_seg_ = MJerkSegment::build(
      p0, v0, a0, p_target, v_target, a_target, duration, ros::Time::now());
  
  double final_vel = active_seg_->get_max_velocity();
  ROS_WARN("Duration failed, T=%.2fs v=%.3f>%.3f",
           duration, final_vel, effective_max_vel);
}


/*  Publish Trajectory Setpoint using MAVROS                         */

void OffboardFSM::publish_current_setpoint()
{
  if (current_state_ == FsmState::TRAJ) {
    // In TRAJ state, stop publishing position setpoints completely
    // Let traj_server (Elastic Tracker) take full control via PositionTarget messages
    // This matches RealFlight_ros implementation - no setpoint publishing in TRAJ state
    return;
  }
  
  mavros_msgs::PositionTarget sp;
  sp.header.stamp = ros::Time::now();
  sp.header.frame_id = "map";
  // PX4 only supports MAV_FRAME_LOCAL_NED (1) for SET_POSITION_TARGET_LOCAL_NED
  // MAVROS will automatically convert ENU coordinates to NED for PX4
  // We provide ENU coordinates, MAVROS handles the conversion
  sp.coordinate_frame = 1;  // MAV_FRAME_LOCAL_NED (PX4 requirement)
  
  // Set type mask - we will set position, velocity, and acceleration
  sp.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  
  sp.yaw = 0.0;  // Face North in ENU (0 = North, M_PI/2 = East)

  if (active_seg_.has_value()) {
    Eigen::Vector3d p, v, a;
    active_seg_->sample(ros::Time::now(), p, v, a);
    
    // ENU: z should be positive (up), clamp to ground level (0.0)
    const double GROUND_TOLERANCE = 0.05;
    if (p.z() < GROUND_TOLERANCE) {
      ROS_WARN_THROTTLE(1.0, "z=%.3f corrected to ground level", p.z());
      p.z() = 0.0;
      v.z() = 0.0;
      a.z() = 0.0;
    }
    
    sp.position.x = p.x();
    sp.position.y = p.y();
    sp.position.z = p.z();
    sp.velocity.x = v.x();
    sp.velocity.y = v.y();
    sp.velocity.z = v.z();
    sp.acceleration_or_force.x = a.x();
    sp.acceleration_or_force.y = a.y();
    sp.acceleration_or_force.z = a.z();
    
    sp.type_mask = 0;  // Use position, velocity, and acceleration
    
    // Debug output for takeoff
    if (current_state_ == FsmState::TAKEOFF) {
      ROS_INFO_THROTTLE(0.5, "TAKEOFF setpoint: pos=[%.3f,%.3f,%.3f] vel=[%.3f,%.3f,%.3f]",
                       p.x(), p.y(), p.z(), v.x(), v.y(), v.z());
    }
    
    final_setpoint_hold_count_ = 0;
    
    if (active_seg_->finished(ros::Time::now())) {
      final_position_ = p;
      final_velocity_ = v;
      final_acceleration_ = a;
      has_final_setpoint_ = true;
      
      ROS_INFO("Seg done v=[%.3f,%.3f,%.3f]",
               v.x(), v.y(), v.z());
      active_seg_.reset();
    }
  } 
  else if (has_final_setpoint_ && 
           (current_state_ == FsmState::TAKEOFF || 
            current_state_ == FsmState::GOTO ||
            current_state_ == FsmState::LAND)) {
    
    sp.position.x = final_position_.x();
    sp.position.y = final_position_.y();
    sp.position.z = final_position_.z();
    
    // Exponential velocity decay for smooth transition
    const double DECAY_TC = 0.05;
    double decay = std::exp(-timer_period_s_ / DECAY_TC);
    final_velocity_ *= decay;
    
    sp.velocity.x = final_velocity_.x();
    sp.velocity.y = final_velocity_.y();
    sp.velocity.z = final_velocity_.z();
    sp.acceleration_or_force.x = 0.0;
    sp.acceleration_or_force.y = 0.0;
    sp.acceleration_or_force.z = 0.0;
    
    sp.type_mask = 0;
    
    final_setpoint_hold_count_++;
    
    // For TAKEOFF, keep publishing setpoint longer to ensure altitude is reached
    int max_hold_count = (current_state_ == FsmState::TAKEOFF) ? 50 : 10;
    
    if (final_setpoint_hold_count_ > max_hold_count || final_velocity_.norm() < 0.01) {
      has_final_setpoint_ = false;
      final_setpoint_hold_count_ = 0;
      ROS_INFO("Steady state (hold_count=%d)", final_setpoint_hold_count_);
      
      // For TAKEOFF, if we're entering steady state but haven't reached altitude,
      // continue publishing takeoff setpoint in the else branch
      if (current_state_ == FsmState::TAKEOFF) {
        double actual_alt = current_z_;
        if (actual_alt < (takeoff_alt_ - alt_tol_)) {
          ROS_WARN("TAKEOFF: Entering steady state but altitude not reached (%.3f < %.3f)",
                   actual_alt, takeoff_alt_ - alt_tol_);
        }
      }
    }
  }
  else {
    has_final_setpoint_ = false;
    final_setpoint_hold_count_ = 0;
    
    // Only position control
    sp.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                   mavros_msgs::PositionTarget::IGNORE_VY |
                   mavros_msgs::PositionTarget::IGNORE_VZ |
                   mavros_msgs::PositionTarget::IGNORE_AFX |
                   mavros_msgs::PositionTarget::IGNORE_AFY |
                   mavros_msgs::PositionTarget::IGNORE_AFZ |
                   mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    
    switch (current_state_) {
      case FsmState::INIT:
      case FsmState::ARMING:
        sp.position.x = takeoff_pos_x_;
        sp.position.y = takeoff_pos_y_;
        sp.position.z = 0.05;
        break;
        
      case FsmState::HOVER:
      case FsmState::END_TRAJ:
        sp.position.x = hover_x_;
        sp.position.y = hover_y_;
        sp.position.z = hover_z_;
        break;
        
      case FsmState::TAKEOFF:
        sp.position.x = takeoff_pos_x_;
        sp.position.y = takeoff_pos_y_;
        sp.position.z = takeoff_alt_;
        // Debug: Log setpoint when trajectory is not active
        ROS_INFO_THROTTLE(1.0, "TAKEOFF fallback setpoint: [%.3f,%.3f,%.3f] (current_z=%.3f)",
                         takeoff_pos_x_, takeoff_pos_y_, takeoff_alt_, current_z_);
        break;
        
      case FsmState::GOTO:
        if (has_goto_target()) {
          sp.position.x = goto_x_;
          sp.position.y = goto_y_;
          sp.position.z = goto_z_;
        } else if (odom_ready_) {
          sp.position.x = current_x_;
          sp.position.y = current_y_;
          sp.position.z = current_z_;
        } else {
          sp.position.x = 0.0;
          sp.position.y = 0.0;
          sp.position.z = 0.05;
        }
        break;
        
      case FsmState::LAND:
        sp.position.x = landing_x_;
        sp.position.y = landing_y_;
        sp.position.z = 0.0;
        break;
        
      case FsmState::DONE:
        sp.position.x = landing_x_;
        sp.position.y = landing_y_;
        sp.position.z = 0.0;
        break;
        
      case FsmState::TRAJ:
        ROS_WARN_THROTTLE(5.0, "TRAJ in fallback");
        break;
    }
  }
  
  // Ground validation (ENU: z should be >= 0)
  const double GROUND_TOL = 0.05;
  if (sp.position.z < GROUND_TOL) {
    ROS_ERROR_THROTTLE(1.0, "z=%.3f fixed to ground level", sp.position.z);
    sp.position.z = 0.0;
  }
  
  // NaN fallback - safety check for invalid setpoints
  if (!std::isfinite(sp.position.x) || 
      !std::isfinite(sp.position.y) || 
      !std::isfinite(sp.position.z)) {
    ROS_ERROR_THROTTLE(1.0, "Invalid SP detected, using fallback");
    if (odom_ready_) {
      sp.position.x = current_x_;
      sp.position.y = current_y_;
      sp.position.z = std::max(current_z_, 0.0);  // ENU: ensure positive altitude
    } else {
      sp.position.x = 0.0;
      sp.position.y = 0.0;
      sp.position.z = 0.05;
    }
    sp.velocity.x = 0.0;
    sp.velocity.y = 0.0;
    sp.velocity.z = 0.0;
  }
  
  setpoint_pub_.publish(sp);
}


/*  Main Timer Callback - State Machine Logic                        */

void OffboardFSM::timer_cb(const ros::TimerEvent& event)
{
  publish_current_setpoint();
  
  switch (current_state_) {
  case FsmState::INIT:
    if (offb_counter_ >= 20) {
      ROS_INFO_ONCE("Arming sequence start");
      current_state_ = FsmState::ARMING;
      offb_counter_ = 0;
    }
    break;

  case FsmState::ARMING: {
    bool is_offboard = current_mavros_state_.mode == "OFFBOARD";
    bool is_armed = current_mavros_state_.armed;
    
    if (is_offboard && is_armed) {
      initial_arming_complete_ = true;
      ROS_INFO("Drone %d armed in offboard", drone_id_);
      current_state_       = FsmState::TAKEOFF;
      takeoff_start_count_ = offb_counter_;
      last_z_              = current_z_;
      
      Eigen::Vector3d p_start(current_x_, current_y_, current_z_);
      Eigen::Vector3d p_target(takeoff_pos_x_, takeoff_pos_y_, takeoff_alt_);
      double optimal_duration = calculate_optimal_duration(
          p_start, p_target, current_vel_, goto_max_vel_);
      
      double actual_duration = std::max(optimal_duration, takeoff_time_s_);
      start_mjerk_segment(p_target, actual_duration);
      
      ROS_INFO("Takeoff T=%.2fs", actual_duration);
      
      offb_counter_        = 0;
      offboard_cmd_count_  = 0;
      arm_cmd_count_       = 0;
    } else {
      if (!is_offboard && !initial_arming_complete_ && offboard_cmd_count_ % 50 == 0) {
        try_set_offboard_and_arm();
        ROS_INFO_THROTTLE(1.0, "Req offboard");
      }
      offboard_cmd_count_++;
      
      if (is_offboard && !initial_arming_complete_ && !is_armed && arm_cmd_count_ % 50 == 0) {
        try_set_offboard_and_arm();
        ROS_INFO_THROTTLE(1.0, "Req arm");
      }
      if (is_offboard) arm_cmd_count_++;
    }
    break;
  }

  case FsmState::TAKEOFF: {
    if (!active_seg_.has_value()) {
      double actual_alt = current_z_;  // ENU: positive altitude
      
      if (takeoff_complete_count_ < 0) {
        bool alt_ok = actual_alt >= (takeoff_alt_ - alt_tol_);
        bool vel_ok = std::abs(current_z_ - last_z_) / timer_period_s_ < 0.1;
        
        ROS_INFO_THROTTLE(1.0, "TAKEOFF: alt=%.3f (target=%.3f, tol=%.3f), vel_ok=%d, alt_ok=%d",
                         actual_alt, takeoff_alt_, alt_tol_, vel_ok, alt_ok);
        
        if (alt_ok && vel_ok) {
          takeoff_complete_count_ = offb_counter_;
          ROS_INFO("Alt %.2fm reached", actual_alt);
        } else {
          // Keep publishing takeoff setpoint until altitude is reached
          // This ensures PX4 continues to receive setpoints
          ROS_INFO_THROTTLE(1.0, "TAKEOFF: Waiting for altitude (current=%.3f, target=%.3f)",
                           actual_alt, takeoff_alt_);
        }
      }
      
      if (takeoff_complete_count_ >= 0) {
        double hover_time = (offb_counter_ - takeoff_complete_count_) * timer_period_s_;
        if (hover_time >= 2.0) {
          if (has_goto_target()) {
            Eigen::Vector3d p_start(current_x_, current_y_, current_z_);
            Eigen::Vector3d p_target(goto_x_, goto_y_, goto_z_);
            
            double duration = calculate_optimal_duration(
                p_start, p_target, current_vel_, goto_max_vel_);
            
            start_mjerk_segment(p_target, duration);
            
            double dist = (p_target - p_start).norm();
            current_state_ = FsmState::GOTO;
            ROS_INFO("GOTO %.2fm T=%.1fs", dist, duration);
          } else {
            hover_x_ = takeoff_pos_x_;
            hover_y_ = takeoff_pos_y_;
            hover_z_ = takeoff_alt_;
            current_state_ = FsmState::HOVER;
            ROS_INFO("HOVER");
          }
          state_start_time_ = ros::Time::now();
          offb_counter_ = 0;
        }
      }
    } else {
      // Trajectory is still active, log progress
      ROS_INFO_THROTTLE(1.0, "TAKEOFF: Trajectory active, current_z=%.3f, target_z=%.3f",
                       current_z_, takeoff_alt_);
    }
    
    last_z_ = current_z_;
    break;
  }

  case FsmState::GOTO: {
    if (!has_goto_target()) {
      hover_x_ = current_x_;
      hover_y_ = current_y_;
      hover_z_ = current_z_;
      current_state_ = FsmState::HOVER;
      ROS_WARN("No target, HOVER");
      break;
    }

    if (!active_seg_.has_value()) {
      double dx = current_x_ - goto_x_;
      double dy = current_y_ - goto_y_;
      double dz = current_z_ - goto_z_;
      double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
      
      if (dist < goto_tol_) {
        ROS_INFO("GOTO done err=%.3fm", dist);
        hover_x_ = goto_x_;
        hover_y_ = goto_y_;
        hover_z_ = goto_z_;
        current_state_ = FsmState::HOVER;
        state_start_time_ = ros::Time::now();
        offb_counter_ = 0;
      } else {
        ROS_WARN_THROTTLE(1.0, "Seg ended err=%.3fm", dist);
      }
    }
    break;
  }

  case FsmState::HOVER:
    if (use_attitude_control_) {
      use_attitude_control_ = false;
    }
    break;

  case FsmState::TRAJ:
    if (!use_attitude_control_) {
      use_attitude_control_ = true;
      state_start_time_ = ros::Time::now();
    }
    break;

  case FsmState::END_TRAJ: {
    if (use_attitude_control_) {
      use_attitude_control_ = false;
      state_start_time_ = ros::Time::now();
      ROS_INFO("Traj complete, hovering for %.1fs before auto-landing", 
               end_traj_wait_time_);
    }
    
    // Check if wait time has elapsed
    double elapsed = (ros::Time::now() - state_start_time_).toSec();
    if (elapsed >= end_traj_wait_time_) {
      // Automatically transition to LAND
      landing_start_count_ = offb_counter_;
      landing_start_z_     = current_z_;  // ENU: positive altitude
      landing_x_           = current_x_;  // Use END_TRAJ position
      landing_y_           = current_y_;  // Use END_TRAJ position
      
      // Create polynomial trajectory from current position to ground
      Eigen::Vector3d p_target(current_x_, current_y_, 0.0);
      start_mjerk_segment(p_target, landing_time_s_, 
                         Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                         landing_max_vel_);
      
      current_state_ = FsmState::LAND;
      ROS_INFO("AUTO LAND from [%.2f, %.2f, %.2f] after %.1fs hover (T=%.1fs, v_max=%.2f)", 
               landing_x_, landing_y_, landing_start_z_, elapsed,
               landing_time_s_, landing_max_vel_);
    }
    break;
  }

  case FsmState::LAND: {
    if (!active_seg_.has_value()) {
      double alt = current_z_;  // ENU: positive altitude
      if (alt <= 0.1) {
        ROS_INFO("Landed");
        current_state_ = FsmState::DONE;
        
        // Disarm
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = false;
        if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
          ROS_INFO("Vehicle disarmed");
        }
      }
    }
    break;
  }

  case FsmState::DONE:
    break;
  }

  std_msgs::Int32 st; 
  st.data = static_cast<int>(current_state_);
  state_pub_.publish(st);
  
  ++offb_counter_;
}

void OffboardFSM::try_set_offboard_and_arm()
{
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  
  if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
    ROS_DEBUG("Offboard enabled");
  }
  
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  
  if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
    ROS_DEBUG("Vehicle armed");
  }
}

void OffboardFSM::publish_offboard_mode()
{
  // Not needed for MAVROS - mode is handled by service calls
}


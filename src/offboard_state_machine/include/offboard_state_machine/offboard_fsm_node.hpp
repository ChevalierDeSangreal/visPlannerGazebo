// offboard_fsm_node.hpp
// Header file for PX4 offboard control FSM with ROS1 + MAVROS
// Adapted from Elastic-Tracker for visPlanner

#ifndef OFFBOARD_FSM_NODE_HPP_
#define OFFBOARD_FSM_NODE_HPP_

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <Eigen/Dense>
#include <optional>

// FSM states
enum class FsmState {
  INIT = 0,
  ARMING = 1,
  TAKEOFF = 2,
  GOTO = 3,
  HOVER = 4,
  TRAJ = 5,
  END_TRAJ = 6,
  LAND = 7,
  DONE = 8
};

// Minimum jerk trajectory segment
struct MJerkSegment {
  Eigen::Matrix<double,6,1> ax, ay, az;  // Quintic polynomial coefficients
  ros::Time t0;                           // Start time
  double T{0.0};                          // Duration
  
  // Static factory method to build a minimum jerk trajectory
  static MJerkSegment build(
      const Eigen::Vector3d& p0,  // Initial position
      const Eigen::Vector3d& v0,  // Initial velocity
      const Eigen::Vector3d& a0,  // Initial acceleration
      const Eigen::Vector3d& pf,  // Final position
      const Eigen::Vector3d& vf,  // Final velocity (usually zero)
      const Eigen::Vector3d& af,  // Final acceleration (usually zero)
      double T,                    // Duration
      ros::Time t0);              // Start time
  
  // Sample the trajectory at a given time
  void sample(const ros::Time& now,
              Eigen::Vector3d& p,
              Eigen::Vector3d& v,
              Eigen::Vector3d& a) const;
  
  // Check if trajectory is complete
  bool finished(const ros::Time& now) const;
  
  // Get maximum velocity magnitude along the trajectory
  double get_max_velocity() const;
};

// ============================================================================
// OFFBOARD FSM CLASS
// ============================================================================
class OffboardFSM
{
public:
  explicit OffboardFSM(ros::NodeHandle& nh, ros::NodeHandle& nh_private, int drone_id);

private:
  // Callbacks
  void timer_cb(const ros::TimerEvent& event);
  void state_cb(const mavros_msgs::State::ConstPtr& msg);
  void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void state_cmd_cb(const std_msgs::Int32::ConstPtr& msg);

  // Helper functions
  void publish_offboard_mode();
  void publish_current_setpoint();
  void try_set_offboard_and_arm();
  bool has_goto_target() const;
  
  // Minimum jerk trajectory planning
  void start_mjerk_segment(const Eigen::Vector3d& p_target,
                           double duration,
                           const Eigen::Vector3d& v_target = Eigen::Vector3d::Zero(),
                           const Eigen::Vector3d& a_target = Eigen::Vector3d::Zero(),
                           double max_vel_override = -1.0);
  
  // Calculate optimal duration to respect velocity limits
  double calculate_optimal_duration(const Eigen::Vector3d& p_start,
                                   const Eigen::Vector3d& p_target,
                                   const Eigen::Vector3d& v_start,
                                   double max_vel) const;
  
  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  
  // Publishers
  ros::Publisher setpoint_pub_;
  ros::Publisher state_pub_;
  
  // Subscribers
  ros::Subscriber state_sub_;
  ros::Subscriber local_pos_sub_;
  ros::Subscriber local_vel_sub_;
  ros::Subscriber state_cmd_sub_;
  
  // Service clients
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  
  // Timer
  ros::Timer timer_;

  // Parameters
  int drone_id_;
  double takeoff_alt_;
  double takeoff_time_s_;
  double climb_rate_;
  double landing_time_s_;
  double circle_radius_;
  double inward_offset_;
  int num_drones_;
  double timer_period_s_;
  double alt_tol_;
  double radius_;
  double period_s_;
  bool initial_arming_complete_; 
  
  // GOTO parameters
  double goto_x_;
  double goto_y_;
  double goto_z_;
  double goto_tol_;
  double goto_max_vel_;
  double goto_accel_time_;
  ros::Time goto_start_time_;
  double goto_start_x_, goto_start_y_, goto_start_z_;
  double goto_duration_;
  bool in_goto_transition_;
  double end_traj_wait_time_;
  bool traj_position_disabled_;  // Track if position control has been disabled in TRAJ state
  
  // Payload offset
  double payload_offset_x_;
  double payload_offset_y_;

  // State variables
  FsmState current_state_;
  int offb_counter_;
  int takeoff_start_count_;
  int takeoff_complete_count_;
  int landing_start_count_;
  
  // Command tracking
  int offboard_cmd_count_;
  int arm_cmd_count_;
  
  // Position tracking
  double takeoff_pos_x_;
  double takeoff_pos_y_;
  double hover_x_;
  double hover_y_;
  double hover_z_;
  double landing_x_;
  double landing_y_;
  double landing_start_z_;
  double landing_max_vel_;
  bool vel_initialized_;
  bool has_final_setpoint_;
  int final_setpoint_hold_count_;
  
  // Current position from odometry
  double current_x_;
  double current_y_;
  double current_z_;
  double last_z_;
  bool odom_ready_;
  
  // Control mode
  bool use_attitude_control_;
  
  // MAVROS state
  mavros_msgs::State current_mavros_state_;
  
  // Time tracking
  ros::Time state_start_time_;

  Eigen::Vector3d final_position_{0, 0, 0};
  Eigen::Vector3d final_velocity_{0, 0, 0};
  Eigen::Vector3d final_acceleration_{0, 0, 0};
  
  // MINIMUM JERK TRAJECTORY STATE
  std::optional<MJerkSegment> active_seg_;
  
  // Velocity estimation for smooth transitions
  Eigen::Vector3d current_vel_{0.0, 0.0, 0.0};
  Eigen::Vector3d last_pos_{0.0, 0.0, 0.0};
};

#endif  // OFFBOARD_FSM_NODE_HPP_


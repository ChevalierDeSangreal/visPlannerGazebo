#ifndef TARGET_PUBLISHER_NODE_HPP
#define TARGET_PUBLISHER_NODE_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int32.h>
#include <Eigen/Dense>
#include <chrono>

/**
 * @brief 目标轨迹发布器节点
 * 
 * 发布目标的位置和速度信息，支持多种运动模式：
 * - 静止模式：固定位置
 * - 圆周模式：圆形轨迹运动
 * - 8字模式：8字轨迹运动
 * 
 * 参考RealFlight_ros的target_publisher实现
 */
class TargetPublisherNode {
public:
  TargetPublisherNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  ~TargetPublisherNode() = default;

private:
  enum class TrajectoryMode {
    STATIC = 0,      // 静止
    CIRCLE = 1,      // 圆周运动
    FIGURE_EIGHT = 2,// 8字运动
    D_SHAPE = 3      // D型运动
  };
  
  void timer_callback(const ros::TimerEvent& event);
  void state_callback(const std_msgs::Int32::ConstPtr& msg);
  void generate_static_trajectory(double t);
  void generate_circular_trajectory(double t);
  void generate_figure_eight_trajectory(double t);
  void generate_dshape_trajectory(double t);
  
  double calculate_theta_at_time(double t);
  double calculate_angular_velocity_at_time(double t);
  double calculate_normalized_parameter_at_time(double t);
  double calculate_parameter_velocity_at_time(double t);
  double calculate_effective_duration();
  
  /**
   * @brief Calculate arc length of a cubic Bezier curve using numerical integration
   */
  double calculate_bezier_arc_length(double P0x, double P0y, double P1x, double P1y,
                                     double P2x, double P2y, double P3x, double P3y,
                                     double t_end = 1.0);
  
  /**
   * @brief Find parameter t on Bezier curve given target arc length from start
   */
  double find_bezier_parameter_from_arc_length(double target_length,
                                                double P0x, double P0y, double P1x, double P1y,
                                                double P2x, double P2y, double P3x, double P3y);
  
  /**
   * @brief Pre-calculate D-shape segment arc lengths
   */
  void calculate_dshape_segment_lengths();
  
  void publish_target_odom(double x, double y, double z, 
                           double vx, double vy, double vz);
  void publish_target_position(double x, double y, double z);
  void publish_target_velocity(double vx, double vy, double vz);
  
  // ROS节点句柄
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  
  // 发布器和订阅器
  ros::Publisher odom_pub_;           // 发布nav_msgs/Odometry，兼容Elastic-Tracker
  ros::Publisher elastic_target_pub_;  // Elastic-Tracker专用目标话题
  ros::Publisher position_pub_;       // 发布geometry_msgs/PointStamped，神经网络控制使用
  ros::Publisher velocity_pub_;        // 发布geometry_msgs/TwistStamped
  ros::Subscriber state_sub_;         // 订阅状态机状态
  
  // 定时器
  ros::Timer timer_;
  
  // 参数
  TrajectoryMode mode_;
  double timer_period_;           // 发布周期 [s]
  
  // 静止模式参数
  double static_x_, static_y_, static_z_;
  
  // 圆周模式参数
  double circle_radius_;          // 圆半径 [m]
  double circle_center_x_;        // 圆心X坐标 [m]
  double circle_center_y_;        // 圆心Y坐标 [m]
  double circle_center_z_;        // 圆心Z坐标 [m]
  double circle_duration_;        // 单圈时间 [s]
  double circle_init_phase_;      // 初始相位 [rad]
  int circle_times_;              // 圆圈数量
  
  // 通用轨迹参数（用于8字和D型）
  double trajectory_size_;        // 轨迹尺寸 [m]
  double trajectory_center_x_;    // 轨迹中心X [m]
  double trajectory_center_y_;    // 轨迹中心Y [m]
  double trajectory_center_z_;    // 轨迹中心Z [m]
  double trajectory_duration_;    // 轨迹周期 [s] (auto-calculated for D-shape)
  int trajectory_times_;          // 轨迹循环次数
  
  // 运动参数
  double ramp_up_time_;           // 加速时间 [s] (auto-calculated for D-shape)
  double ramp_down_time_;         // 减速时间 [s] (auto-calculated for D-shape)
  double stationary_time_;        // 静止时间 [s]
  
  // Velocity-based parameters for D-shape trajectory
  double max_linear_velocity_;    // 最大线速度 [m/s] (for D-shape)
  double linear_acceleration_;    // 线加速度/减速度 [m/s²] (for D-shape)
  
  // 计算得到的参数
  double effective_duration_;     // 有效单圈时间
  double max_angular_vel_;        // 最大角速度
  double angular_acceleration_;   // 角加速度
  double total_constant_duration_;// 匀速运动总时间
  
  // D-shape弧长跟踪参数（NEW: 替代基于时间的比例）
  double dshape_total_arc_length_;     // D型轨迹的总弧长 [m]
  double dshape_segment_lengths_[4];   // 每个Bezier段的弧长 [m]
  double dshape_cumulative_lengths_[5]; // 累积弧长 [0, L0, L0+L1, ..., total]
  double dshape_current_arc_length_;   // 当前已行进的弧长 [m] (用于弧长跟踪)
  
  // 状态变量
  bool trajectory_started_;
  ros::Time start_time_;
  std::chrono::steady_clock::time_point start_time_system_;
  bool use_sim_time_;
  
  // 状态机相关
  int drone_id_;                      // 无人机ID
  int current_state_;                 // 当前状态机状态
  bool in_traj_state_;                // 是否在TRAJ状态
  ros::Time traj_entry_time_;         // 进入TRAJ状态的时间
  std::chrono::steady_clock::time_point traj_entry_time_system_;  // 系统时间版本
  
  // 话题配置
  std::string elastic_target_topic_;  // Elastic-Tracker目标话题名
};

#endif // TARGET_PUBLISHER_NODE_HPP


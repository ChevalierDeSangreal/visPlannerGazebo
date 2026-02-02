#ifndef TRACKING_VISUALIZER_NODE_HPP
#define TRACKING_VISUALIZER_NODE_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <deque>
#include <vector>
#include <fstream>
#include <memory>
#include <cmath>

/**
 * @brief 追踪结果可视化节点
 * 
 * 功能：
 * 1. 订阅无人机和目标的位置信息
 * 2. 记录并可视化历史轨迹
 * 3. 计算和显示跟踪误差（距离误差、速度误差）
 * 4. 保存数据到CSV文件用于后续分析
 * 5. 发布RViz markers进行实时可视化
 * 6. 监听状态机状态，在TRAJ状态时才记录数据
 */
class TrackingVisualizerNode {
public:
  TrackingVisualizerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  ~TrackingVisualizerNode();

private:
  // 回调函数
  void drone_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
  void target_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
  void state_callback(const std_msgs::Int32::ConstPtr& msg);
  void visualization_timer_callback(const ros::TimerEvent& event);
  
  // 可视化函数
  void publish_trajectories();
  void publish_error_markers();
  void publish_statistics();
  
  // 工具函数
  double calculate_distance_error();
  double calculate_velocity_error();
  void save_data_to_file();
  void compute_statistics();
  
  // ROS节点句柄
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  
  // 订阅器
  ros::Subscriber drone_odom_sub_;
  ros::Subscriber target_odom_sub_;
  ros::Subscriber state_sub_;
  
  // 发布器
  ros::Publisher drone_path_pub_;
  ros::Publisher target_path_pub_;
  ros::Publisher error_marker_pub_;
  ros::Publisher error_value_pub_;
  ros::Publisher distance_error_pub_;
  ros::Publisher velocity_error_pub_;
  
  // 定时器
  ros::Timer visualization_timer_;
  
  // 参数
  int max_trajectory_points_;      // 最大轨迹点数
  double visualization_rate_;      // 可视化频率 [Hz]
  bool save_to_file_;              // 是否保存数据到文件
  std::string output_file_path_;   // 输出文件路径
  std::string drone_topic_;        // 无人机话题
  std::string target_topic_;       // 目标话题
  int drone_id_;                   // 无人机ID
  bool only_record_in_traj_;        // 是否只在TRAJ状态记录数据
  
  // 状态机状态
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
  FsmState current_state_;
  bool in_traj_state_;
  ros::Time traj_start_time_;  // TRAJ状态开始时间，用于计算相对时间戳
  
  // 数据存储
  struct TrackingData {
    ros::Time timestamp;
    geometry_msgs::Point drone_pos;
    geometry_msgs::Point target_pos;
    geometry_msgs::Vector3 drone_vel;
    geometry_msgs::Vector3 target_vel;
    double distance_error;
    double velocity_error;
  };
  
  std::deque<TrackingData> tracking_history_;
  nav_msgs::Path drone_path_;
  nav_msgs::Path target_path_;
  
  // 当前状态
  nav_msgs::Odometry current_drone_odom_;
  nav_msgs::Odometry current_target_odom_;
  bool drone_received_;
  bool target_received_;
  
  // 统计信息
  double mean_distance_error_;
  double max_distance_error_;
  double rms_distance_error_;
  int data_count_;
  
  // 输出文件流
  std::unique_ptr<std::ofstream> output_file_;
  
  // 可视化颜色
  struct Color {
    float r, g, b, a;
  };
  Color drone_color_;
  Color target_color_;
  Color error_color_;
};

#endif // TRACKING_VISUALIZER_NODE_HPP


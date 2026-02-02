#include "tracking_visualizer/tracking_visualizer_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Int32.h>

TrackingVisualizerNode::TrackingVisualizerNode(ros::NodeHandle& nh, 
                                               ros::NodeHandle& nh_private)
  : nh_(nh), nh_private_(nh_private),
    drone_received_(false), target_received_(false),
    mean_distance_error_(0.0), max_distance_error_(0.0),
    rms_distance_error_(0.0), data_count_(0),
    current_state_(FsmState::INIT), in_traj_state_(false),
    traj_start_time_(ros::Time(0))
{
  // 读取参数
  nh_private_.param<int>("max_trajectory_points", max_trajectory_points_, 1000);
  nh_private_.param<double>("visualization_rate", visualization_rate_, 10.0);
  nh_private_.param<bool>("save_to_file", save_to_file_, true);
  nh_private_.param<std::string>("output_file_path", output_file_path_, 
                                  "/tmp/elastic_tracker_tracking_data.csv");
  nh_private_.param<std::string>("drone_topic", drone_topic_, 
                                  "/drone0/mavros/local_position/odom");
  nh_private_.param<std::string>("target_topic", target_topic_, 
                                  "/target/odom");
  nh_private_.param<int>("drone_id", drone_id_, 0);
  nh_private_.param<bool>("only_record_in_traj", only_record_in_traj_, true);
  
  // 颜色配置 - 使用临时变量读取参数
  double temp_r, temp_g, temp_b, temp_a;
  
  nh_private_.param<double>("drone_color/r", temp_r, 0.0);
  nh_private_.param<double>("drone_color/g", temp_g, 0.0);
  nh_private_.param<double>("drone_color/b", temp_b, 1.0);
  nh_private_.param<double>("drone_color/a", temp_a, 1.0);
  drone_color_ = {static_cast<float>(temp_r), static_cast<float>(temp_g), 
                  static_cast<float>(temp_b), static_cast<float>(temp_a)};
  
  nh_private_.param<double>("target_color/r", temp_r, 1.0);
  nh_private_.param<double>("target_color/g", temp_g, 0.0);
  nh_private_.param<double>("target_color/b", temp_b, 0.0);
  nh_private_.param<double>("target_color/a", temp_a, 1.0);
  target_color_ = {static_cast<float>(temp_r), static_cast<float>(temp_g), 
                   static_cast<float>(temp_b), static_cast<float>(temp_a)};
  
  nh_private_.param<double>("error_color/r", temp_r, 1.0);
  nh_private_.param<double>("error_color/g", temp_g, 1.0);
  nh_private_.param<double>("error_color/b", temp_b, 0.0);
  nh_private_.param<double>("error_color/a", temp_a, 0.5);
  error_color_ = {static_cast<float>(temp_r), static_cast<float>(temp_g), 
                  static_cast<float>(temp_b), static_cast<float>(temp_a)};
  
  // 初始化路径
  drone_path_.header.frame_id = "world";
  target_path_.header.frame_id = "world";
  
  // 订阅器
  drone_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
    drone_topic_, 10, &TrackingVisualizerNode::drone_odom_callback, this);
  target_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
    target_topic_, 10, &TrackingVisualizerNode::target_odom_callback, this);
  
  // 订阅状态机状态
  std::string state_topic = "/state/state_drone_" + std::to_string(drone_id_);
  state_sub_ = nh_.subscribe<std_msgs::Int32>(
    state_topic, 10, &TrackingVisualizerNode::state_callback, this);
  
  // 发布器
  drone_path_pub_ = nh_.advertise<nav_msgs::Path>(
    "/tracking_viz/drone_path", 10);
  target_path_pub_ = nh_.advertise<nav_msgs::Path>(
    "/tracking_viz/target_path", 10);
  error_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
    "/tracking_viz/error_markers", 10);
  distance_error_pub_ = nh_.advertise<std_msgs::Float64>(
    "/tracking_viz/distance_error", 10);
  velocity_error_pub_ = nh_.advertise<std_msgs::Float64>(
    "/tracking_viz/velocity_error", 10);
  
  // 定时器
  visualization_timer_ = nh_.createTimer(
    ros::Duration(1.0 / visualization_rate_),
    &TrackingVisualizerNode::visualization_timer_callback, this);
  
  // 如果需要保存文件，打开文件流
  if (save_to_file_) {
    output_file_ = std::make_unique<std::ofstream>(output_file_path_);
    if (output_file_->is_open()) {
      // 写入CSV表头
      *output_file_ << "timestamp,drone_x,drone_y,drone_z,drone_roll,drone_pitch,drone_yaw,"
                    << "target_x,target_y,target_z,"
                    << "drone_vx,drone_vy,drone_vz,"
                    << "target_vx,target_vy,target_vz,"
                    << "distance_error,velocity_error\n";
      ROS_INFO("Saving tracking data to: %s", output_file_path_.c_str());
    } else {
      ROS_WARN("Failed to open output file: %s", output_file_path_.c_str());
      save_to_file_ = false;
    }
  }
  
  ROS_INFO("=== Tracking Visualizer for Elastic-Tracker ===");
  ROS_INFO("Drone ID: %d", drone_id_);
  ROS_INFO("State topic: %s", state_topic.c_str());
  ROS_INFO("Drone topic: %s", drone_topic_.c_str());
  ROS_INFO("Target topic: %s", target_topic_.c_str());
  ROS_INFO("Max trajectory points: %d", max_trajectory_points_);
  ROS_INFO("Visualization rate: %.1f Hz", visualization_rate_);
  ROS_INFO("Save to file: %s", save_to_file_ ? "Yes" : "No");
  ROS_INFO("Only record in TRAJ state: %s", only_record_in_traj_ ? "Yes" : "No");
  ROS_INFO("Tracking visualizer initialized!");
}

TrackingVisualizerNode::~TrackingVisualizerNode() {
  // 确保所有数据都写入磁盘
  if (save_to_file_ && output_file_ && output_file_->is_open()) {
    output_file_->flush();  // 确保缓冲区数据写入
    output_file_->close();
    ROS_INFO("Data saved to: %s", output_file_path_.c_str());
  }
  
  // 打印统计信息
  if (data_count_ > 0) {
    ROS_INFO("=== Tracking Statistics ===");
    ROS_INFO("Total data points: %d", data_count_);
    ROS_INFO("Mean distance error: %.4f m", mean_distance_error_);
    ROS_INFO("Max distance error: %.4f m", max_distance_error_);
    ROS_INFO("RMS distance error: %.4f m", rms_distance_error_);
  }
}

void TrackingVisualizerNode::state_callback(const std_msgs::Int32::ConstPtr& msg) {
  FsmState new_state = static_cast<FsmState>(msg->data);
  
  if (new_state != current_state_) {
    ROS_INFO("State changed: %d -> %d", static_cast<int>(current_state_), msg->data);
    current_state_ = new_state;
    
    // 检测进入TRAJ状态
    if (new_state == FsmState::TRAJ && !in_traj_state_) {
      in_traj_state_ = true;
      traj_start_time_ = ros::Time::now();  // 记录TRAJ状态开始时间
      ROS_INFO("✅ Entered TRAJ state - Starting tracking data recording");
      
      // 清空之前的数据
      if (only_record_in_traj_) {
        drone_path_.poses.clear();
        target_path_.poses.clear();
        tracking_history_.clear();
        mean_distance_error_ = 0.0;
        max_distance_error_ = 0.0;
        rms_distance_error_ = 0.0;
        data_count_ = 0;
      }
    }
    
    // 检测离开TRAJ状态
    if (new_state != FsmState::TRAJ && in_traj_state_) {
      in_traj_state_ = false;
      ROS_INFO("Left TRAJ state - Stopped tracking data recording");
      
      // 打印统计信息
      if (data_count_ > 0) {
        ROS_INFO("=== Final Tracking Statistics ===");
        ROS_INFO("Total data points: %d", data_count_);
        ROS_INFO("Mean distance error: %.4f m", mean_distance_error_);
        ROS_INFO("Max distance error: %.4f m", max_distance_error_);
        ROS_INFO("RMS distance error: %.4f m", rms_distance_error_);
      }
    }
  }
}

void TrackingVisualizerNode::drone_odom_callback(
    const nav_msgs::Odometry::ConstPtr& msg) {
  current_drone_odom_ = *msg;
  drone_received_ = true;
  
  // 只在TRAJ状态或不限制时添加到轨迹
  if (!only_record_in_traj_ || in_traj_state_) {
    // 添加到轨迹
    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
    drone_path_.poses.push_back(pose);
    
    // 限制轨迹点数
    if (drone_path_.poses.size() > static_cast<size_t>(max_trajectory_points_)) {
      drone_path_.poses.erase(drone_path_.poses.begin());
    }
  }
  
  // 如果目标也收到了，计算误差并记录（只在TRAJ状态或不限制时）
  if (target_received_ && (!only_record_in_traj_ || in_traj_state_)) {
    // 从四元数提取yaw角
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    TrackingData data;
    data.timestamp = msg->header.stamp;
    data.drone_pos = msg->pose.pose.position;
    data.target_pos = current_target_odom_.pose.pose.position;
    data.drone_vel = msg->twist.twist.linear;
    data.target_vel = current_target_odom_.twist.twist.linear;
    data.distance_error = calculate_distance_error();
    data.velocity_error = calculate_velocity_error();
    
    tracking_history_.push_back(data);
    
    // 限制历史数据点数
    if (tracking_history_.size() > static_cast<size_t>(max_trajectory_points_)) {
      tracking_history_.pop_front();
    }
    
    // 保存到文件
    if (save_to_file_ && output_file_ && output_file_->is_open()) {
      // 计算相对时间戳（从TRAJ状态开始的时间）
      double relative_time = 0.0;
      if (traj_start_time_.toSec() > 0.0) {
        ros::Time current_time = ros::Time::now();
        relative_time = (current_time - traj_start_time_).toSec();
      } else {
        // 如果traj_start_time_未设置，使用消息头时间戳或当前时间
        ros::Time timestamp = data.timestamp;
        if (timestamp.toSec() == 0.0 || timestamp.toSec() < 1e9) {
          timestamp = ros::Time::now();
        }
        relative_time = timestamp.toSec();
      }
      *output_file_ << relative_time << ","
                    << data.drone_pos.x << "," << data.drone_pos.y << "," 
                    << data.drone_pos.z << "," << roll << "," << pitch << "," << yaw << ","
                    << data.target_pos.x << "," << data.target_pos.y << "," 
                    << data.target_pos.z << ","
                    << data.drone_vel.x << "," << data.drone_vel.y << "," 
                    << data.drone_vel.z << ","
                    << data.target_vel.x << "," << data.target_vel.y << "," 
                    << data.target_vel.z << ","
                    << data.distance_error << "," << data.velocity_error << "\n";
      // 立即flush，确保数据写入磁盘
      output_file_->flush();
    }
    
    // 更新统计信息
    compute_statistics();
  }
}

void TrackingVisualizerNode::target_odom_callback(
    const nav_msgs::Odometry::ConstPtr& msg) {
  current_target_odom_ = *msg;
  target_received_ = true;
  
  // 只在TRAJ状态或不限制时添加到轨迹
  if (!only_record_in_traj_ || in_traj_state_) {
    // 添加到轨迹
    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
    target_path_.poses.push_back(pose);
    
    // 限制轨迹点数
    if (target_path_.poses.size() > static_cast<size_t>(max_trajectory_points_)) {
      target_path_.poses.erase(target_path_.poses.begin());
    }
  }
}

void TrackingVisualizerNode::visualization_timer_callback(
    const ros::TimerEvent& event) {
  if (!drone_received_ || !target_received_) {
    return;
  }
  
  publish_trajectories();
  publish_error_markers();
  publish_statistics();
}

void TrackingVisualizerNode::publish_trajectories() {
  // 发布无人机轨迹
  drone_path_.header.stamp = ros::Time::now();
  drone_path_pub_.publish(drone_path_);
  
  // 发布目标轨迹
  target_path_.header.stamp = ros::Time::now();
  target_path_pub_.publish(target_path_);
}

void TrackingVisualizerNode::publish_error_markers() {
  visualization_msgs::MarkerArray marker_array;
  
  // 1. 连线marker - 显示当前误差
  visualization_msgs::Marker line_marker;
  line_marker.header.frame_id = "world";
  line_marker.header.stamp = ros::Time::now();
  line_marker.ns = "error_line";
  line_marker.id = 0;
  line_marker.type = visualization_msgs::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::Marker::ADD;
  line_marker.scale.x = 0.02;  // 线宽
  line_marker.color.r = error_color_.r;
  line_marker.color.g = error_color_.g;
  line_marker.color.b = error_color_.b;
  line_marker.color.a = error_color_.a;
  
  line_marker.points.push_back(current_drone_odom_.pose.pose.position);
  line_marker.points.push_back(current_target_odom_.pose.pose.position);
  marker_array.markers.push_back(line_marker);
  
  // 2. 文本marker - 显示误差数值
  visualization_msgs::Marker text_marker;
  text_marker.header.frame_id = "world";
  text_marker.header.stamp = ros::Time::now();
  text_marker.ns = "error_text";
  text_marker.id = 1;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::Marker::ADD;
  
  // 文本位置在无人机和目标中间
  text_marker.pose.position.x = 
    (current_drone_odom_.pose.pose.position.x + 
     current_target_odom_.pose.pose.position.x) / 2.0;
  text_marker.pose.position.y = 
    (current_drone_odom_.pose.pose.position.y + 
     current_target_odom_.pose.pose.position.y) / 2.0;
  text_marker.pose.position.z = 
    (current_drone_odom_.pose.pose.position.z + 
     current_target_odom_.pose.pose.position.z) / 2.0 + 0.3;
  
  text_marker.scale.z = 0.15;  // 文本大小
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;
  
  double dist_error = calculate_distance_error();
  char text[100];
  snprintf(text, sizeof(text), "Error: %.3f m", dist_error);
  text_marker.text = text;
  marker_array.markers.push_back(text_marker);
  
  // 3. 球体marker - 标记无人机位置
  visualization_msgs::Marker drone_marker;
  drone_marker.header.frame_id = "world";
  drone_marker.header.stamp = ros::Time::now();
  drone_marker.ns = "drone_position";
  drone_marker.id = 2;
  drone_marker.type = visualization_msgs::Marker::SPHERE;
  drone_marker.action = visualization_msgs::Marker::ADD;
  drone_marker.pose = current_drone_odom_.pose.pose;
  drone_marker.scale.x = 0.2;
  drone_marker.scale.y = 0.2;
  drone_marker.scale.z = 0.2;
  drone_marker.color.r = drone_color_.r;
  drone_marker.color.g = drone_color_.g;
  drone_marker.color.b = drone_color_.b;
  drone_marker.color.a = drone_color_.a;
  marker_array.markers.push_back(drone_marker);
  
  // 4. 球体marker - 标记目标位置
  visualization_msgs::Marker target_marker;
  target_marker.header.frame_id = "world";
  target_marker.header.stamp = ros::Time::now();
  target_marker.ns = "target_position";
  target_marker.id = 3;
  target_marker.type = visualization_msgs::Marker::SPHERE;
  target_marker.action = visualization_msgs::Marker::ADD;
  target_marker.pose = current_target_odom_.pose.pose;
  target_marker.scale.x = 0.15;
  target_marker.scale.y = 0.15;
  target_marker.scale.z = 0.15;
  target_marker.color.r = target_color_.r;
  target_marker.color.g = target_color_.g;
  target_marker.color.b = target_color_.b;
  target_marker.color.a = target_color_.a;
  marker_array.markers.push_back(target_marker);
  
  // 发布markers
  error_marker_pub_.publish(marker_array);
}

void TrackingVisualizerNode::publish_statistics() {
  // 发布距离误差
  std_msgs::Float64 dist_error_msg;
  dist_error_msg.data = calculate_distance_error();
  distance_error_pub_.publish(dist_error_msg);
  
  // 发布速度误差
  std_msgs::Float64 vel_error_msg;
  vel_error_msg.data = calculate_velocity_error();
  velocity_error_pub_.publish(vel_error_msg);
  
  // 定期打印统计信息
  static ros::Time last_print_time = ros::Time::now();
  if ((ros::Time::now() - last_print_time).toSec() > 5.0) {
    ROS_INFO("Tracking Stats | Mean Error: %.4f m | Max Error: %.4f m | RMS: %.4f m | Points: %d",
             mean_distance_error_, max_distance_error_, 
             rms_distance_error_, data_count_);
    last_print_time = ros::Time::now();
  }
}

double TrackingVisualizerNode::calculate_distance_error() {
  const auto& dp = current_drone_odom_.pose.pose.position;
  const auto& tp = current_target_odom_.pose.pose.position;
  
  double dx = dp.x - tp.x;
  double dy = dp.y - tp.y;
  double dz = dp.z - tp.z;
  
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

double TrackingVisualizerNode::calculate_velocity_error() {
  const auto& dv = current_drone_odom_.twist.twist.linear;
  const auto& tv = current_target_odom_.twist.twist.linear;
  
  double dvx = dv.x - tv.x;
  double dvy = dv.y - tv.y;
  double dvz = dv.z - tv.z;
  
  return std::sqrt(dvx*dvx + dvy*dvy + dvz*dvz);
}

void TrackingVisualizerNode::compute_statistics() {
  if (tracking_history_.empty()) {
    return;
  }
  
  data_count_ = tracking_history_.size();
  
  // 计算平均距离误差
  double sum_error = 0.0;
  double sum_squared_error = 0.0;
  max_distance_error_ = 0.0;
  
  for (const auto& data : tracking_history_) {
    sum_error += data.distance_error;
    sum_squared_error += data.distance_error * data.distance_error;
    if (data.distance_error > max_distance_error_) {
      max_distance_error_ = data.distance_error;
    }
  }
  
  mean_distance_error_ = sum_error / data_count_;
  rms_distance_error_ = std::sqrt(sum_squared_error / data_count_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "tracking_visualizer");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  TrackingVisualizerNode node(nh, nh_private);
  
  ros::spin();
  
  return 0;
}


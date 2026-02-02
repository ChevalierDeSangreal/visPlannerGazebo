#include <ros/ros.h>
#include <quadrotor_msgs/OccMap3d.h>
#include <std_msgs/Header.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "empty_map_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Get parameters
  std::string map_topic;
  nh_private.param<std::string>("map_topic", map_topic, "gridmap_inflate");
  
  double publish_rate;
  nh_private.param<double>("publish_rate", publish_rate, 10.0);

  // Create publisher - use relative topic name to respect namespace
  // If node is in a group ns (e.g., /drone0), topic will be /drone0/gridmap_inflate
  ros::Publisher map_pub = nh.advertise<quadrotor_msgs::OccMap3d>(map_topic, 1, true);

  // Create empty map message
  quadrotor_msgs::OccMap3d map_msg;
  map_msg.header.frame_id = "world";
  map_msg.resolution = 0.1;  // 10cm resolution
  map_msg.size_x = 100;       // 10m x 10m x 10m map
  map_msg.size_y = 100;
  map_msg.size_z = 100;
  map_msg.offset_x = -50;     // Center at origin
  map_msg.offset_y = -50;
  map_msg.offset_z = -50;
  // Empty data array (all free space)
  map_msg.data.resize(map_msg.size_x * map_msg.size_y * map_msg.size_z, 0);

  // Get the actual resolved topic name for logging
  std::string resolved_topic = nh.resolveName(map_topic);
  ROS_WARN("[Empty Map Publisher] Publishing empty map to topic: %s (resolved: %s)", 
           map_topic.c_str(), resolved_topic.c_str());
  ROS_WARN("[Empty Map Publisher] Map size: %dx%dx%d, Resolution: %.2f m", 
           map_msg.size_x, map_msg.size_y, map_msg.size_z, map_msg.resolution);

  ros::Rate rate(publish_rate);
  while (ros::ok()) {
    map_msg.header.stamp = ros::Time::now();
    map_pub.publish(map_msg);
    rate.sleep();
  }

  return 0;
}


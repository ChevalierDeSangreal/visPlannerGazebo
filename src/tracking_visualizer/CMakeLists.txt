cmake_minimum_required(VERSION 3.0.2)
project(tracking_visualizer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    nav_msgs 
    geometry_msgs 
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 追踪可视化节点
add_executable(tracking_visualizer_node 
  src/tracking_visualizer_node.cpp
)

target_link_libraries(tracking_visualizer_node
  ${catkin_LIBRARIES}
)

# 安装
install(TARGETS tracking_visualizer_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)


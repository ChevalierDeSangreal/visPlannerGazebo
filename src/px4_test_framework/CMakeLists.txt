cmake_minimum_required(VERSION 3.0.2)
project(px4_test_framework)

## Compile as C++14
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  mavros_msgs
  quadrotor_msgs
  traj_utils
  bspline_opt
  tf2
  tf2_ros
  tf2_geometry_msgs
)

## Find Eigen3
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES px4_test_framework
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    geometry_msgs 
    nav_msgs 
    sensor_msgs 
    mavros_msgs 
    quadrotor_msgs
    traj_utils
    bspline_opt
    tf2
    tf2_ros
  DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../planner/bspline_opt/include
)

## Declare a C++ library (optional, for shared code)
add_library(${PROJECT_NAME}
  src/target_publisher_node.cpp
  src/state_listener_node.cpp
  src/pos_cmd_to_mavros_node.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  bspline_opt
  traj_utils
)

## Target publisher node
add_executable(target_publisher_node src/target_publisher_main.cpp)
add_dependencies(target_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(target_publisher_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  bspline_opt
  traj_utils
  ${EIGEN3_LIBRARIES}
)
target_include_directories(target_publisher_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../planner/bspline_opt/include
)

## State listener node
add_executable(state_listener_node src/state_listener_main.cpp)
add_dependencies(state_listener_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_listener_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Position command to MAVROS converter node
add_executable(pos_cmd_to_mavros_node src/pos_cmd_to_mavros_main.cpp)
add_dependencies(pos_cmd_to_mavros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pos_cmd_to_mavros_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Empty map publisher node
add_executable(empty_map_publisher_node src/empty_map_publisher_main.cpp)
add_dependencies(empty_map_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(empty_map_publisher_node
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

## Mark executables for installation
install(TARGETS target_publisher_node state_listener_node pos_cmd_to_mavros_node empty_map_publisher_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

## Mark other files for installation (launch, config, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)


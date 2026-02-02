cmake_minimum_required(VERSION 3.0.2)
project(offboard_state_machine)

## C++17
add_compile_options(-std=c++17)

## Find catkin and dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  mavros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    geometry_msgs 
    sensor_msgs 
    mavros_msgs 
    tf2 
    tf2_ros
    tf2_geometry_msgs
  DEPENDS EIGEN3
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(offboard_fsm_node
  src/offboard_fsm_node.cpp
  src/main.cpp
)

target_compile_features(offboard_fsm_node PUBLIC cxx_std_17)

## Link libraries
target_link_libraries(offboard_fsm_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS offboard_fsm_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark headers for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

## Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


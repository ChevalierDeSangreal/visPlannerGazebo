cmake_minimum_required(VERSION 2.8.3)
project(bezier_predict)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env
  cv_bridge
  traj_utils
  bspline_opt
)

# OOQP 库路径 - 必须在 catkin_package 之前定义
set(OOQP_ROOT $ENV{HOME}/ooqp_install)
set(OOQP_INCLUDE_DIR ${OOQP_ROOT}/include)
set(OOQP_INCLUDE_DIR_SRC ${OOQP_ROOT}/include/ooqp/src)
set(OOQP_LIB_DIR ${OOQP_ROOT}/lib)

# OOQP 头文件需要多个子目录路径
set(OOQP_INCLUDE_DIRS
    ${OOQP_INCLUDE_DIR}
    ${OOQP_INCLUDE_DIR_SRC}
    ${OOQP_INCLUDE_DIR_SRC}/Abstract
    ${OOQP_INCLUDE_DIR_SRC}/Vector
    ${OOQP_INCLUDE_DIR_SRC}/QpGen
    ${OOQP_INCLUDE_DIR_SRC}/QpSolvers
    ${OOQP_INCLUDE_DIR_SRC}/LinearSolvers/Ma27Solver
    ${OOQP_INCLUDE_DIR_SRC}/SparseLinearAlgebra
    ${OOQP_INCLUDE_DIR_SRC}/DenseLinearAlgebra
)

catkin_package(
 INCLUDE_DIRS include ${OOQP_INCLUDE_DIRS}
 LIBRARIES bezier_predict
#  CATKIN_DEPENDS plan_env 
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OOQP_INCLUDE_DIRS}
)

add_library( bezier_predict STATIC
    src/bezier_predict.cpp 
    src/predictor.cpp 
    )

target_link_libraries( bezier_predict
    ${catkin_LIBRARIES} 
    ${OOQP_LIB_DIR}/libooqpgensparse.a
    ${OOQP_LIB_DIR}/libooqpsparse.a
    ${OOQP_LIB_DIR}/libooqpgondzio.a
    ${OOQP_LIB_DIR}/libooqpbase.a
    ${OOQP_LIB_DIR}/libma27.a
    blas
    lapack
    gfortran
    )  

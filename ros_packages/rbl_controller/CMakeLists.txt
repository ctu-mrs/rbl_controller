cmake_minimum_required(VERSION 3.0.2)
project(rbl_controller)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  mrs_msgs
  mrs_lib
  
)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

find_package(
  Boost REQUIRED COMPONENTS
  )

catkin_package(
  LIBRARIES RBLController
  CATKIN_DEPENDS roscpp mrs_lib mrs_msgs geometry_msgs nav_msgs
  DEPENDS Eigen
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

## Declare C++ library
add_library(RBLController src/rbl_controller.cpp)

## Link libraries
target_link_libraries(RBLController
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

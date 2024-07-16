cmake_minimum_required(VERSION 3.0.2)
project(rbl_controller)

# Find catkin packages
find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  mrs_msgs
  mrs_lib
  visualization_msgs
  pcl_ros
)

# Find Eigen
find_package(Eigen3 REQUIRED)

# Find Boost
find_package(Boost REQUIRED)

# Set Eigen variables
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN_LIBRARIES})

# Declare catkin package
catkin_package(
  LIBRARIES RBLController 
  CATKIN_DEPENDS roscpp mrs_lib mrs_msgs geometry_msgs nav_msgs 
  DEPENDS Eigen
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

# Declare C++ library
add_library(RBLController src/rbl_controller.cpp)
#add_library(lidar_clusters src/lidar_clusters.cpp)

# Link libraries for the RBLController library
target_link_libraries(RBLController
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

# Declare the lidar_clusters executable
#add_executable(lidar_clusters_node src/lidar_clusters.cpp)

# Link libraries for the lidar_clusters executable
#target_link_libraries(lidar_clusters
#  ${catkin_LIBRARIES}
#)


cmake_minimum_required(VERSION 3.0.2)
project(rbl_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-std=c++17)

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
add_executable(continuous_marker_clear src/clear.cpp)
target_link_libraries(continuous_marker_clear ${catkin_LIBRARIES})
# Link libraries for the lidar_clusters executable
#target_link_libraries(lidar_clusters
#  ${catkin_LIBRARIES}
#)


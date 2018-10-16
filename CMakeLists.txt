cmake_minimum_required(VERSION 2.8.3)
project(concaveteam)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  tf2
  tf2_ros
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

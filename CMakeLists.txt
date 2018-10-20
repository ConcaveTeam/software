cmake_minimum_required(VERSION 2.8.3)
project(concaveteam)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  tf2_ros
  image_transport
  cv_bridge
)

find_package(OpenCV 3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(track_mono src/TrackMono/TrackMono.cpp)
target_link_libraries(track_mono ${catkin_LIBRARIES} ${OpenCV_LIbs})

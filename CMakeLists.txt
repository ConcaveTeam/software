cmake_minimum_required(VERSION 2.8.3)
project(concaveteam)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  image_transport
  cv_bridge
)

find_package(OpenCV 3 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_pub src/image_pub/image_pub.cpp)
target_link_libraries(image_pub ${catkin_LIBRARIES} ${OpenCV_LIBS})

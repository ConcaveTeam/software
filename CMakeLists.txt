cmake_minimum_required(VERSION 2.8.3)
project(concaveteam)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  tf2_ros
  image_transport
  cv_bridge
  message_generation
  )

find_package(OpenCV 3 REQUIRED)
find_package(PCL REQUIRED)

add_message_files(
  FILES
  Spherical.msg
  )

add_service_files(
  FILES
  Trigger.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package(
  CATKIN_DEPENDS
  roscpp
  message_runtime
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(track_mono src/TrackMono/TrackMono.cpp)
add_executable(point2d_to_spherical src/Point2DToSpherical/Point2DToSpherical.cpp)
add_executable(track_3d src/track3d/track3d.cpp)

target_link_libraries(track_mono ${catkin_LIBRARIES} ${OpenCV_LIbs})
target_link_libraries(point2d_to_spherical ${catkin_LIBRARIES})
target_link_libraries(track_3d ${PCL_COMMON_LIBRARIES} ${catkin_LIBRARIES})

add_dependencies(point2d_to_spherical ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

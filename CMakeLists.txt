cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_sampler)

find_package(catkin REQUIRED pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS})

add_executable(sampler src/sampler.cpp)
target_link_libraries(sampler ${catkin_LIBRARIES})

add_executable(remove_ground src/remove_ground.cpp)
target_link_libraries(remove_ground ${catkin_LIBRARIES})
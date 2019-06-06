cmake_minimum_required(VERSION 2.8.3)
project(lab1)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(lab1_node src/lab1.cpp)

target_link_libraries(lab1_node
  ${catkin_LIBRARIES}
)



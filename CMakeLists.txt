cmake_minimum_required(VERSION 3.0.2)
project(project1)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


## Declare a catkin package
catkin_package()


## Build

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odometry_pub src/odometry_pub.cpp)
target_link_libraries(odometry_pub ${catkin_LIBRARIES})

  
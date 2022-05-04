cmake_minimum_required(VERSION 3.0.2)
project(project1)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  Reset.srv
)

add_message_files(
  FILES
  Wheel_speed_msg.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
 )

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)



## Build

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odometry_pub src/odometry_pub.cpp)
target_link_libraries(odometry_pub ${catkin_LIBRARIES})

add_executable(control_pub src/control_pub.cpp)
target_link_libraries(control_pub ${catkin_LIBRARIES})

add_dependencies(control_pub ${catkin_EXPORTED_TARGETS}) #see slide 29 lab 4
#add_dependencies(control_sub ${catkin_EXPORTED_TARGETS}) 

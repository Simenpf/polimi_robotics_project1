cmake_minimum_required(VERSION 3.0.2)
project(project1)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf2_ros
  dynamic_reconfigure
)

## Add custom message, service, dynamic reconfiguring and their dependencies
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

 generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odometry_node src/odometry_node.cpp src/encoder_computations.cpp)
add_dependencies(
  odometry_node 
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages
  ${PROJECT_NAME}_gencfg)
target_link_libraries(odometry_node ${catkin_LIBRARIES})

add_executable(command_node src/command_node.cpp src/encoder_computations.cpp)
add_dependencies(command_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(command_node ${catkin_LIBRARIES})

add_executable(control_node src/control_node.cpp)
add_dependencies(
  control_node 
  ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages
  ${PROJECT_NAME}_gencfg)
target_link_libraries(control_node ${catkin_LIBRARIES})  

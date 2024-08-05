cmake_minimum_required(VERSION 2.8.3)
if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

project(plugin_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
   FILES
   RobotState.msg
 )
 
generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
 )
 
catkin_package(
  CATKIN_DEPENDS message_runtime 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

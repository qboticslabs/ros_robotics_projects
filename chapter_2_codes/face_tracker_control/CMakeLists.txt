cmake_minimum_required(VERSION 2.8.3)
project(face_tracker_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamixel_controllers
  roscpp
  rospy
  std_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
   FILES
   centroid.msg
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
  
 )


catkin_package(
  CATKIN_DEPENDS dynamixel_controllers roscpp rospy std_msgs

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(face_tracker_controller src/face_tracker_controller.cpp)
target_link_libraries(face_tracker_controller ${catkin_LIBRARIES})


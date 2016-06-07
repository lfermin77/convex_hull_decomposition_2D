cmake_minimum_required(VERSION 2.8.3)
project(obstacle2convex)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  nav_msgs
  cv_bridge
  image_transport
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu2tf
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


## Declare a cpp executable
 add_executable(obstacle2convex src/obstacle2convex.cpp)

 target_link_libraries(obstacle2convex
   ${catkin_LIBRARIES}



 )


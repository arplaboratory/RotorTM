cmake_minimum_required(VERSION 2.8.3)
project(rotor_tm_traj)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  message_generation)

add_service_files(
  DIRECTORY srv
  FILES
    Vec4.srv
    Circle.srv
    Line.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES 
  CATKIN_DEPENDS
    rospy
    nav_msgs
    sensor_msgs
    geometry_msgs
    std_msgs
    message_runtime)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)


## Mark cpp header files for installation
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

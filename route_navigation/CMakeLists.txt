cmake_minimum_required(VERSION 2.8.3)
project(route_navigation)

find_package(catkin REQUIRED COMPONENTS
  tf
  actionlib
  geometry_msgs
  move_base_msgs
  nav_msgs
  ropod_ros_msgs
  maneuver_navigation
)

# catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    tf
    actionlib
    geometry_msgs
    move_base_msgs
    nav_msgs
    ropod_ros_msgs
    maneuver_navigation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

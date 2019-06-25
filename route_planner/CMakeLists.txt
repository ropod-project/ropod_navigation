cmake_minimum_required(VERSION 2.8.3)
project(route_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  ropod_ros_msgs
  genmsg
  actionlib_msgs
  actionlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  ropod_ros_msgs
  actionlib_msgs
)

add_library(route_planner ${PROJECT_SOURCE_DIR}/src/route_planner.cpp ${HEADER_FILES})
add_dependencies(route_planner ropod_ros_msgs_generate_messages_cpp)
target_link_libraries(route_planner ${LIBS} ${catkin_LIBRARIES})

add_library(simple_osm_route_planner ${PROJECT_SOURCE_DIR}/src/simple_osm_route_planner.cpp)
add_dependencies(simple_osm_route_planner ropod_ros_msgs_generate_messages_cpp)
target_link_libraries(simple_osm_route_planner ${LIBS} ${catkin_LIBRARIES} route_planner)

add_library(napoleon_driving_planner ${PROJECT_SOURCE_DIR}/src/napoleon_driving_planner.cpp)
add_dependencies(napoleon_driving_planner ropod_ros_msgs_generate_messages_cpp)
target_link_libraries(napoleon_driving_planner ${LIBS} ${catkin_LIBRARIES} route_planner)

add_library(osm_sub_areas_route_planner ${PROJECT_SOURCE_DIR}/src/osm_sub_areas_route_planner.cpp)
add_dependencies(osm_sub_areas_route_planner ropod_ros_msgs_generate_messages_cpp)
target_link_libraries(osm_sub_areas_route_planner ${LIBS} ${catkin_LIBRARIES} route_planner)

add_executable(route_planner_node ${PROJECT_SOURCE_DIR}/src/route_planner_node.cpp ${HEADER_FILES})
add_dependencies(route_planner_node ropod_ros_msgs_generate_messages_cpp)
target_link_libraries(route_planner_node ${LIBS} ${catkin_LIBRARIES} simple_osm_route_planner osm_sub_areas_route_planner napoleon_driving_planner)

add_executable(route_planner_test_amk ${PROJECT_SOURCE_DIR}/test/route_planner_test_amk.cpp ${HEADER_FILES})
add_dependencies(route_planner_test_amk ropod_ros_msgs_generate_messages_cpp)
target_link_libraries(route_planner_test_amk ${LIBS} ${catkin_LIBRARIES})

add_executable(route_planner_test_brsu ${PROJECT_SOURCE_DIR}/test/route_planner_test_brsu.cpp ${HEADER_FILES})
add_dependencies(route_planner_test_brsu ropod_ros_msgs_generate_messages_cpp)
target_link_libraries(route_planner_test_brsu ${LIBS} ${catkin_LIBRARIES})

add_executable(route_planner_visualizer ${PROJECT_SOURCE_DIR}/test/route_planner_visualizer.cpp ${HEADER_FILES})
add_dependencies(route_planner_visualizer ropod_ros_msgs_generate_messages_cpp)
target_link_libraries(route_planner_visualizer ${LIBS} ${catkin_LIBRARIES})

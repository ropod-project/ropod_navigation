#ifndef ROUTE_PLANNER_HPP
#define ROUTE_PLANNER_HPP

/* ROS includes */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/tf.h>

/* C++ */
#include <iostream>
#include <queue>


/* OSM messages */
#include <ropod_ros_msgs/osm_node.h>
#include <ropod_ros_msgs/osm_way.h>
#include <ropod_ros_msgs/osm_relation.h>
#include <ropod_ros_msgs/osm_tag.h>
#include <ropod_ros_msgs/osm_member.h>
#include <ropod_ros_msgs/osm_query.h>

#include <ropod_ros_msgs/route_planner.h>


class RoutePlanner
{
private:

public:
  ros::NodeHandle nh;

  RoutePlanner();
  virtual ~RoutePlanner();

  bool routePlannerServiceCallback(ropod_ros_msgs::route_planner::Request &req,ropod_ros_msgs::route_planner::Response &res);
  virtual std::vector<ropod_ros_msgs::Area> compute_route(std::vector<ropod_ros_msgs::Area>) = 0;
  std::vector<ropod_ros_msgs::Area> compute_orientations(std::vector<ropod_ros_msgs::Area>);
};

#endif /* ROUTE_PLANNER_HPP */

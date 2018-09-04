#ifndef ROUTE_PLANNER_HPP
#define ROUTE_PLANNER_HPP

/* ROS includes */
#include <ros/ros.h>
#include <std_msgs/String.h>

/* C++ */
#include <iostream>


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
  ros::ServiceClient osm_query_client;

  RoutePlanner();
  virtual ~RoutePlanner();

  bool routePlannerServiceCallback(ropod_ros_msgs::route_planner::Request &req,ropod_ros_msgs::route_planner::Response &res);
};

#endif /* ROUTE_PLANNER_HPP */

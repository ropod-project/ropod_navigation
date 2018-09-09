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
#include <ropod_ros_msgs/OSMNode.h>
#include <ropod_ros_msgs/OSMWay.h>
#include <ropod_ros_msgs/OSMRelation.h>
#include <ropod_ros_msgs/OSMTag.h>
#include <ropod_ros_msgs/OSMMember.h>
#include <ropod_ros_msgs/OSMQuery.h>

#include <ropod_ros_msgs/RoutePlanner.h>


class RoutePlanner
{
private:

public:
  ros::NodeHandle nh;

  RoutePlanner();
  virtual ~RoutePlanner();

  bool routePlannerServiceCallback(ropod_ros_msgs::RoutePlanner::Request &req,ropod_ros_msgs::RoutePlanner::Response &res);
  virtual std::vector<ropod_ros_msgs::Area> compute_route(std::vector<ropod_ros_msgs::Area>) = 0;
  std::vector<ropod_ros_msgs::Area> compute_orientations(std::vector<ropod_ros_msgs::Area>);
};

#endif /* ROUTE_PLANNER_HPP */

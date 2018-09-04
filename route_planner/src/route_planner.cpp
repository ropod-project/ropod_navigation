#include <route_planner/route_planner.hpp>


RoutePlanner::RoutePlanner(): nh("~")
{
}

RoutePlanner::~RoutePlanner()
{
}

bool RoutePlanner::routePlannerServiceCallback(ropod_ros_msgs::route_planner::Request &req,ropod_ros_msgs::route_planner::Response &res)
{
  ROS_INFO("Service called");
}


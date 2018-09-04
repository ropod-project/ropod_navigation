#include <route_planner/route_planner.hpp>


RoutePlanner::RoutePlanner(): nh("~")
{
}

RoutePlanner::~RoutePlanner()
{
}

bool RoutePlanner::routePlannerServiceCallback(ropod_ros_msgs::route_planner::Request &req,ropod_ros_msgs::route_planner::Response &res)
{
  std::vector<ropod_ros_msgs::Area> path_areas;
  for(std::vector<ropod_ros_msgs::Area>::const_iterator curr_area = req.areas.begin(); curr_area != req.areas.end(); ++curr_area)
  {
     path_areas.push_back(*curr_area);
  }
  std::vector<ropod_ros_msgs::Area> path_areas2 = this->compute_route(path_areas);  // implemented in derived classes
  res.areas = path_areas2;
  return true;
}


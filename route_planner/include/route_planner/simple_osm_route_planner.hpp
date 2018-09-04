#ifndef SIMPLE_OSM_ROUTE_PLANNER_HPP
#define SIMPLE_OSM_ROUTE_PLANNER_HPP

#include <route_planner/route_planner.hpp>


class SimpleOSMRoutePlanner: public RoutePlanner
{
public:
  virtual std::vector<ropod_ros_msgs::Area> compute_route(std::vector<ropod_ros_msgs::Area>);
  
};

#endif /* SIMPLE_OSM_ROUTE_PLANNER_HPP */

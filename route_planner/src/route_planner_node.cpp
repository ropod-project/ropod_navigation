#include <route_planner/simple_osm_route_planner.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "route_planner");
  ros::NodeHandle nh("~");

  SimpleOSMRoutePlanner route_planner;

  ros::ServiceServer service = nh.advertiseService("route_planner_service", &SimpleOSMRoutePlanner::routePlannerServiceCallback, (RoutePlanner*)&route_planner);

  ros::spin();

  return 0;
}
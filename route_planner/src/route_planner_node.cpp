#include <route_planner/route_planner.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "route_planner");
  ros::NodeHandle nh("~");

  RoutePlanner route_planner;

  ros::ServiceServer service = nh.advertiseService("route_planner_service", &RoutePlanner::routePlannerServiceCallback, (RoutePlanner*)&route_planner);

  ros::spin();

  return 0;
}
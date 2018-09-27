// #include <route_planner/simple_osm_route_planner.hpp>
#include <route_planner/osm_waypoints_route_planner.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "route_planner");
  ros::NodeHandle nh("~");

  // SimpleOSMRoutePlanner route_planner;
  OSMWaypointsRoutePlanner route_planner;

  ros::spin();

  return 0;
}
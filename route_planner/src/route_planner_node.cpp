// #include <route_planner/simple_osm_route_planner.hpp>
#include <route_planner/osm_sub_areas_route_planner.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "route_planner");
  ros::NodeHandle nh("~");

  // SimpleOSMRoutePlanner route_planner;
  OSMSubAreasRoutePlanner route_planner;

  ros::spin();

  return 0;
}
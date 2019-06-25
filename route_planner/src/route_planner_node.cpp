// #include <route_planner/simple_osm_route_planner.hpp>
//#include <route_planner/osm_sub_areas_route_planner.hpp>
#include <route_planner/napoleon_driving_planner.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "route_planner");
  ros::NodeHandle nh("~");

  // SimpleOSMRoutePlanner route_planner;
  // OSMSubAreasRoutePlanner route_planner;
  NapoleonDrivingPlanner route_planner;
  ROS_INFO("Route planner ready!");

  ros::spin();

  return 0;
}

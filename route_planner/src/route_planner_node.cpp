#include <route_planner/route_planner.hpp>
#include <route_planner/simple_osm_route_planner.hpp>
#include <route_planner/osm_sub_areas_route_planner.hpp>
#include <route_planner/napoleon_driving_planner.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "route_planner");
    ros::NodeHandle nh("~");

    std::string route_plan_type;
    RoutePlanner* route_planner = NULL;

    nh.param<std::string>("route_plan_type", route_plan_type, "OSM-SUBAREA");

    if (route_plan_type == "NAPOLEON")
    {
        route_planner = new NapoleonDrivingPlanner;
        ROS_INFO("Selected route planner: Napoleon");
    }
    else if (route_plan_type == "SIMPLE-OSM")
    {
        route_planner = new SimpleOSMRoutePlanner;
        ROS_INFO("Selected route planner: Simple OSM");
    }
    else
    {
        route_planner = new OSMSubAreasRoutePlanner;
        ROS_INFO("Selected route planner: OSM-Subarea");
    }

    ROS_INFO("Route planner ready!");
    
    ros::spin();
    return 0;
}

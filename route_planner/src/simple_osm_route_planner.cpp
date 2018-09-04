#include <route_planner/simple_osm_route_planner.hpp>


std::vector<ropod_ros_msgs::Area> SimpleOSMRoutePlanner::compute_route(std::vector<ropod_ros_msgs::Area> path_areas)
{
  ros::ServiceClient osm_query_client = nh.serviceClient<ropod_ros_msgs::osm_query>("/ropod_wm_mediator/osm_query");
  ropod_ros_msgs::osm_query osm_query_msg;

  for (auto it1 = path_areas.begin(); it1 != path_areas.end(); it1++) {

    for (auto it2 = it1->waypoints.begin(); it2 != it1->waypoints.end(); it2++)
    {
      osm_query_msg.request.ids = { std::stoi(it2->area_id) };   
      osm_query_msg.request.query_type = "info";
      osm_query_msg.request.data_type = "node";

      if (osm_query_client.call(osm_query_msg))
      {
        it2->waypoint_pose.position.x = osm_query_msg.response.nodes[0].x;
        it2->waypoint_pose.position.y = osm_query_msg.response.nodes[0].y;
        // std::cout << osm_query_msg.response.nodes[0].x << std::endl;
        // std::cout << osm_query_msg.response.nodes[0].y << std::endl;
      }
      else
      {
        ROS_ERROR("Failed to call service osm_query");
      }
    }
  }

  return path_areas;
}
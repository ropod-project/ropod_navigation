/*
Computes route solely based on topological nodes present in OSM
*/
#include <route_planner/simple_osm_route_planner.hpp>

std::vector<ropod_ros_msgs::Area> SimpleOSMRoutePlanner::compute_route(std::vector<ropod_ros_msgs::Area> path_areas)
{
    for (auto it1 = path_areas.begin(); it1 != path_areas.end(); it1++) {
        int no_of_waypts = 0;
        for (auto it2 = it1->waypoints.begin(); it2 != it1->waypoints.end(); it2++)
        {
            ropod_ros_msgs::Position p  = CallGetWayptPositionAction(std::stoi(it2->area_id));   
            
            it2->waypoint_pose.position.x = p.x;
            it2->waypoint_pose.position.y = p.y;

            no_of_waypts++;
        }
        // For doors and other waypoints which doesn't have any local areas
        if(no_of_waypts == 0)
        {
            ropod_ros_msgs::Waypoint way_pt;
            way_pt.semantic_id = it1->name;
            way_pt.area_id = it1->area_id;

            ropod_ros_msgs::Position p  = CallGetWayptPositionAction(std::stoi(it1->area_id));  
            
            way_pt.waypoint_pose.position.x = p.x;
            way_pt.waypoint_pose.position.y = p.y;

            it1->waypoints.push_back(way_pt);
        }
    }
  return path_areas;
}

/*
Computes orientation based on next pose
*/
std::vector<ropod_ros_msgs::Area> SimpleOSMRoutePlanner::compute_orientations(std::vector<ropod_ros_msgs::Area> path_areas)
{
    bool isFirstWaypt = true;
    double last_x = 0;
    double last_y = 0;
    double last_orientation = 0;
    // ropod_ros_msgs::Waypoint *wpt_addr_ref = nullptr;
    std::queue<double> orientations;


    for (auto it1 = path_areas.begin(); it1 != path_areas.end(); it1++) {

        for (auto it2 = it1->waypoints.begin(); it2 != it1->waypoints.end(); it2++)
        {
            if(!isFirstWaypt)
            {
              double angle = -atan2(it2->waypoint_pose.position.y-last_y,it2->waypoint_pose.position.x-last_x)*180/3.1457;        
              // wpt_addr_ref->waypoint_pose.orientation.z = angle;
              orientations.push(angle);
              last_orientation = angle;
            }
            else
            {
              isFirstWaypt = false;
            }
            last_x = it2->waypoint_pose.position.x;
            last_y = it2->waypoint_pose.position.y;           
            //wpt_addr_ref = it2;
        }
    }
    orientations.push(last_orientation);

    for (auto it1 = path_areas.begin(); it1 != path_areas.end(); it1++) {

        for (auto it2 = it1->waypoints.begin(); it2 != it1->waypoints.end(); it2++)
        {
            tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, orientations.front());
            std::cout << "(" << it2->waypoint_pose.position.x << "," << it2->waypoint_pose.position.y << "," << orientations.front() << ")" << std::endl; 
            q.normalize();
            it2->waypoint_pose.orientation.x = q.x();
            it2->waypoint_pose.orientation.y = q.y();
            it2->waypoint_pose.orientation.z = q.z();
            it2->waypoint_pose.orientation.w = q.w();
            orientations.pop();
        }
    }
    return path_areas;
}
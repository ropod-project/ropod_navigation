/*
Computes route solely based on topological nodes present in OSM
*/
#include <route_planner/simple_osm_route_planner.hpp>

std::vector<ropod_ros_msgs::Area> SimpleOSMRoutePlanner::compute_route(std::vector<ropod_ros_msgs::Area> path_areas)
{
    for (auto it1 = path_areas.begin(); it1 != path_areas.end(); it1++) {
        int no_of_sub_areas = 0;
        for (auto it2 = it1->sub_areas.begin(); it2 != it1->sub_areas.end(); it2++)
        {
            ropod_ros_msgs::Position p  = CallGetTopologyNodeAction(std::stoi(it2->area_id), "local_area");   
            
            it2->waypoint_pose.position.x = p.x;
            it2->waypoint_pose.position.y = p.y;

            no_of_sub_areas++;
        }
        // For doors and other sub_areas which doesn't have any local areas
        if(no_of_sub_areas == 0)
        {
            ropod_ros_msgs::SubArea sub_area;
            sub_area.semantic_id = it1->name;
            sub_area.area_id = it1->area_id;

            ropod_ros_msgs::Position p  = CallGetTopologyNodeAction(std::stoi(it1->area_id), "door");  
            
            sub_area.waypoint_pose.position.x = p.x;
            sub_area.waypoint_pose.position.y = p.y;

            it1->sub_areas.push_back(sub_area);
        }
    }
  return path_areas;
}

/*
Computes orientation based on next pose
*/
std::vector<ropod_ros_msgs::Area> SimpleOSMRoutePlanner::compute_orientations(std::vector<ropod_ros_msgs::Area> path_areas)
{
    bool isFirstSubArea = true;
    double last_x = 0;
    double last_y = 0;
    double last_orientation = 0;
    // ropod_ros_msgs::Waypoint *wpt_addr_ref = nullptr;
    std::queue<double> orientations;


    for (auto it1 = path_areas.begin(); it1 != path_areas.end(); it1++) {

        for (auto it2 = it1->sub_areas.begin(); it2 != it1->sub_areas.end(); it2++)
        {
            if(!isFirstSubArea)
            {
              double angle = -atan2(it2->waypoint_pose.position.y-last_y,it2->waypoint_pose.position.x-last_x)*180/3.1457;        
              // wpt_addr_ref->waypoint_pose.orientation.z = angle;
              orientations.push(angle);
              last_orientation = angle;
            }
            else
            {
              isFirstSubArea = false;
            }
            last_x = it2->waypoint_pose.position.x;
            last_y = it2->waypoint_pose.position.y;           
            //wpt_addr_ref = it2;
        }
    }
    orientations.push(last_orientation);

    for (auto it1 = path_areas.begin(); it1 != path_areas.end(); it1++) {

        for (auto it2 = it1->sub_areas.begin(); it2 != it1->sub_areas.end(); it2++)
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
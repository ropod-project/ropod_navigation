#include <route_planner/route_planner.hpp>


RoutePlanner::RoutePlanner(): nh("~")
{
}

RoutePlanner::~RoutePlanner()
{
}

bool RoutePlanner::routePlannerServiceCallback(ropod_ros_msgs::route_planner::Request &req,ropod_ros_msgs::route_planner::Response &res)
{
  std::vector<ropod_ros_msgs::Area> path_areas;
  for(std::vector<ropod_ros_msgs::Area>::const_iterator curr_area = req.areas.begin(); curr_area != req.areas.end(); ++curr_area)
  {
     path_areas.push_back(*curr_area);
  }
  std::vector<ropod_ros_msgs::Area> path_areas2 = this->compute_route(path_areas);  // implemented in derived classes
  res.areas = path_areas2;
  return true;
}

std::vector<ropod_ros_msgs::Area> RoutePlanner::compute_orientations(std::vector<ropod_ros_msgs::Area> path_areas)
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


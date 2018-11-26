#ifndef OSM_SUB_AREAS_ROUTE_PLANNER_HPP
#define OSM_SUB_AREAS_ROUTE_PLANNER_HPP

#include <route_planner/route_planner.hpp>

class OSMSubAreasRoutePlanner: public RoutePlanner
{
public:
  virtual std::vector<ropod_ros_msgs::Area> compute_route(std::vector<ropod_ros_msgs::Area>);
  virtual std::vector<ropod_ros_msgs::Area> compute_orientations(std::vector<ropod_ros_msgs::Area>); 

private:  
  void request_nodes_and_tags(std::vector<ropod_ros_msgs::SubArea>::iterator wayp_areas_it, std::vector<std::vector<ropod_ros_msgs::Area>::iterator>::iterator sub_areas_areait,std::vector<ropod_ros_msgs::Position>& area_nodes, std::string&  area_type);
  double compute_distance_node_to_line(ropod_ros_msgs::Position corridor_node, ropod_ros_msgs::Position line_node_1, ropod_ros_msgs::Position line_node_2);
  geometry_msgs::Pose compute_sub_areas_overlap(std::vector<ropod_ros_msgs::Position>* corridor_nodes_ptr, std::vector<ropod_ros_msgs::Position>*  corridor_intersection_nodes_ptr, bool curr_area_is_corridor);
  bool is_junction(std::string  area_type);
  
};

#endif /* OSM_SUB_AREAS_ROUTE_PLANNER_HPP */

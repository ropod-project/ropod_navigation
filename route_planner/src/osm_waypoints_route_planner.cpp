/*
Computes route based on the intersection between corridors and intersections
*/
#include <route_planner/osm_waypoints_route_planner.hpp>

#include <ropod_ros_msgs/RobotAction.h>

bool OSMWaypointsRoutePlanner::is_junction(std::string  area_type)
{

     return   (area_type == "junction");

}

double OSMWaypointsRoutePlanner::compute_distance_node_to_line(ropod_ros_msgs::Position corridor_node, ropod_ros_msgs::Position line_node_1, ropod_ros_msgs::Position line_node_2)
{    
    
    // First compute the coefficients a*x + b*y + c = 0 of the line defined by the two nodes. https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    double dist_node_to_line;
    double a = line_node_2.y - line_node_1.y;
    double b = line_node_2.x - line_node_1.x;
    double c = line_node_2.x*line_node_1.y - line_node_2.y*line_node_1.x;
    double x0 = corridor_node.x;
    double y0 = corridor_node.y;
    
    double x_line_closest, y_line_closest;
    const double min_value = 10^(-5);
    
    if ( a>=min_value || b>=min_value)
    {        
        // Compute point on line closest to node
        x_line_closest = ( b * (  b*x0 - a*y0 )- a*c ) / ( a*a + b*b );
        y_line_closest = ( a * ( -b*x0 + a*y0 )- b*c ) / ( a*a + b*b );
        
        // Check that that point lies between the two points        
        if ( ( line_node_1.x <= x_line_closest && x_line_closest<= line_node_2.x )   && ( line_node_1.y <= y_line_closest && y_line_closest<= line_node_2.y ) )
        {
            dist_node_to_line = std::sqrt( (x_line_closest-x0)*(x_line_closest-x0) + (y_line_closest-y0)*(y_line_closest-y0));            
        }
        else
        {
            // update the distance with the minimum distance to one of the nodes
            double dist_to_line_node1 = std::sqrt( (line_node_1.x-x0)*(line_node_1.x-x0) + (line_node_1.y-y0)*(line_node_1.y-y0));
            double dist_to_line_node2 = std::sqrt( (line_node_2.x-x0)*(line_node_2.x-x0) + (line_node_2.y-y0)*(line_node_2.y-y0));
            dist_node_to_line = std::min(dist_to_line_node1,dist_to_line_node2);
        }   
        
    }
    else{
        // No line defined
        ROS_ERROR("Two node points too close to each other");
    }
    
    return dist_node_to_line;
}

geometry_msgs::Pose OSMWaypointsRoutePlanner::compute_waypoint_areas_overlap(std::vector<ropod_ros_msgs::Position>* corridor_nodes_ptr, std::vector<ropod_ros_msgs::Position>*  corridor_junction_nodes_ptr, bool curr_area_is_corridor)
{
    geometry_msgs::Pose waypoint_overlap;
    
    // For each node of the corridor_nodes, check the closest distance to the lines that two adjacent nodes form of the corridor_or_intersection_nodes.
    // This is done because the waypoints always should be in a corridor entrance or exit.
    // Then the two closest nodes represent the overlap. 
    // The orientation is determined by pointing otwards the corridor when curr-next areas are corridor-intersection or inwards when curr-next areas are intersection-corridor.
    ropod_ros_msgs::Position closest_node;
    ropod_ros_msgs::Position second_closest_node;
    ropod_ros_msgs::Position farthest_node;    
    
    double dist_closest_node = DBL_MAX;
    double dist_second_closest_node = DBL_MAX;
    double dist_farthest_node = 0.0;
    
    for ( auto corridor_nodes_it = (*corridor_nodes_ptr).begin(); corridor_nodes_it != (*corridor_nodes_ptr).end(); corridor_nodes_it++ )
    {

        // These iterations goes from the beginning to the one second-to-last.
        
        double shortest_dist_node_to_line = DBL_MAX;
         double dist_node_to_line;
        
        for ( auto corridor_intersection_nodes_it = (*corridor_junction_nodes_ptr).begin(); corridor_intersection_nodes_it != (*corridor_junction_nodes_ptr).end()-1 ; corridor_intersection_nodes_it++ )
        {
//             std::cout << "line tests" << std::endl;
//             std::cout << (*corridor_intersection_nodes_it).x << " , " << (*corridor_intersection_nodes_it).y << std::endl;
//             std::cout << (*(corridor_intersection_nodes_it+1)).x << " , " << (*(corridor_intersection_nodes_it+1)).y << std::endl;
            double dist_node_to_line = compute_distance_node_to_line(*corridor_nodes_it, *corridor_intersection_nodes_it,  *(corridor_intersection_nodes_it+1));
            
            if( dist_node_to_line < shortest_dist_node_to_line)
                shortest_dist_node_to_line = dist_node_to_line;
                
            
        }
        
         
        // Last iteration takes last node and the first one.        
        dist_node_to_line = compute_distance_node_to_line(*corridor_nodes_it, * ((*corridor_junction_nodes_ptr).end()-1),  * (*corridor_junction_nodes_ptr).begin() );
        
        if( dist_node_to_line < shortest_dist_node_to_line)
                shortest_dist_node_to_line = dist_node_to_line;
        
        if (shortest_dist_node_to_line < dist_closest_node)
        {
            dist_second_closest_node = dist_closest_node;
            second_closest_node = closest_node;                
            dist_closest_node = shortest_dist_node_to_line;
            closest_node = *corridor_nodes_it;                
            
        }
        else if (shortest_dist_node_to_line < dist_second_closest_node)
        {
            dist_second_closest_node = shortest_dist_node_to_line;
            second_closest_node = *corridor_nodes_it;     
        }
        if (shortest_dist_node_to_line > dist_farthest_node)
        {
            dist_farthest_node = shortest_dist_node_to_line;
            farthest_node = *corridor_nodes_it;                               
        }   
    }
    
    
    /* Next compute the waypoint: position based on the middle line between the two closest nodes and orientation based on that and the farthest node */
    waypoint_overlap.position.x = 0.5*(closest_node.x + second_closest_node.x);
    waypoint_overlap.position.y = 0.5*(closest_node.y + second_closest_node.y);
    waypoint_overlap.position.z = 0.0;
    // For the orientation we assume that the corridor is longer in the direction of movement
    double yaw_furthest_point_to_wayp = std::atan2(farthest_node.y-waypoint_overlap.position.y, farthest_node.x-waypoint_overlap.position.x);
    double yaw_edge_line =  std::atan2( closest_node.y - second_closest_node.y, closest_node.x - second_closest_node.x);
    double yaw_waypoint_inwards;
    double yaw_waypoint_outwards;
    
    if( std::cos( (yaw_edge_line + 0.5*M_PI) - yaw_furthest_point_to_wayp ) > 0 )
    {
        yaw_waypoint_inwards = (yaw_edge_line + 0.5*M_PI);
        yaw_waypoint_outwards = (yaw_edge_line - 0.5*M_PI);
    }
    else
    {
        yaw_waypoint_inwards = (yaw_edge_line - 0.5*M_PI);
        yaw_waypoint_outwards = (yaw_edge_line + 0.5*M_PI);        
    }
    
    double yaw_waypoint;
    if( curr_area_is_corridor )
        yaw_waypoint =  yaw_waypoint_outwards;           
    else
        yaw_waypoint =  yaw_waypoint_inwards;    
        
    tf::Quaternion temp_quat;
    temp_quat.setRPY(0,0, yaw_waypoint);
    waypoint_overlap.orientation.w = temp_quat.getW();
    waypoint_overlap.orientation.x = temp_quat.getX();
    waypoint_overlap.orientation.y = temp_quat.getY();
    waypoint_overlap.orientation.z = temp_quat.getZ();
    
     std::cout << "Node process results" << std::endl;
     std::cout << closest_node.x << " , " << closest_node.y << std::endl;
     std::cout << second_closest_node.x << " , " << second_closest_node.y << std::endl;
     std::cout << waypoint_overlap.position.x << " , " << waypoint_overlap.position.y << std::endl;
    
    
    return waypoint_overlap;
}

void OSMWaypointsRoutePlanner::request_nodes_and_tags(std::vector<ropod_ros_msgs::Waypoint>::iterator wayp_areas_it, std::vector<std::vector<ropod_ros_msgs::Area>::iterator>::iterator wayp_areas_areait,std::vector<ropod_ros_msgs::Position>& area_nodes, std::string&  area_type)
{
    ropod_ros_msgs::Shape wayp_area_shape;
    
    wayp_area_shape = RoutePlanner::CallGetWayptShapeAction(std::stoi(wayp_areas_it->area_id));
    

    area_nodes = wayp_area_shape.vertices;
    std::cout << "Shape request" << std::endl;
    for (auto it_vert = area_nodes.begin(); it_vert != area_nodes.end(); it_vert++ )
    {
        //it_vert->y = - it_vert->y;
        std::cout << it_vert->x << " , " <<it_vert->y << std::endl;
    }    
    area_type = (*wayp_areas_areait)->type;
}

std::vector<ropod_ros_msgs::Area> OSMWaypointsRoutePlanner::compute_route(std::vector<ropod_ros_msgs::Area> path_areas)
{
  
  
  std::vector<ropod_ros_msgs::Position> curr_area_nodes;
  std::vector<ropod_ros_msgs::Position> next_area_nodes;
  std::string curr_area_type;
  std::string next_area_type;
  std::vector<ropod_ros_msgs::Position>* corridor_nodes_ptr;
  std::vector<ropod_ros_msgs::Position>* corridor_junction_nodes_ptr;
  
  std::vector<ropod_ros_msgs::Waypoint> path_osm_wayp; // Here we extract all osm waypoint which has a finer granularity than waypoints.
  std::vector<std::vector<ropod_ros_msgs::Area>::iterator> wayp_area_iterators; // list of pointers to iterator corresponfign to each waypoint
  
  // Extract waypoints areas from areas
  
    for (std::vector<ropod_ros_msgs::Area>::iterator area_it = path_areas.begin(); area_it != path_areas.end(); area_it++) {
        int no_of_waypts = 0;
        for (std::vector<ropod_ros_msgs::Waypoint>::iterator wayp_area_it = area_it->waypoints.begin(); wayp_area_it != area_it->waypoints.end(); wayp_area_it++)
        {

            path_osm_wayp.push_back(*wayp_area_it);
            wayp_area_iterators.push_back(area_it);
            no_of_waypts++;
        }
        // For doors and other waypoints which doesn't have any local areas.  later onIgnore them! For now we just use the position of the waypoint node
        if(no_of_waypts == 0)
        {
            ropod_ros_msgs::Waypoint way_pt;
            way_pt.semantic_id = area_it->name;
            way_pt.area_id = area_it->area_id;

            ropod_ros_msgs::Position p  = CallGetWayptPositionAction(std::stoi(area_it->area_id));  
            
            way_pt.waypoint_pose.position.x = p.x;
            way_pt.waypoint_pose.position.y = p.y;

            area_it->waypoints.push_back(way_pt);
        }else
        {
            area_it->waypoints.clear(); // Clear because later we can insert more waypoints than originally we had
        }
    }  
  
  if (path_osm_wayp.size() < 2 )
  {
      ROS_ERROR("A minimum of two waypoint areas is needed");      
      return path_areas;
  }
  
  ropod_ros_msgs::Waypoint area_waypoint_overlap;
  geometry_msgs::Pose waypoint_overlap;
  
  /* First area is treated in a different way since only one waypoint is needed, not two */
  // Initially request the nodes for first and second area.
  std::vector<ropod_ros_msgs::Waypoint>::iterator wayp_areas_it = path_osm_wayp.begin();
  std::vector<std::vector<ropod_ros_msgs::Area>::iterator>::iterator wayp_areas_areait = wayp_area_iterators.begin();
  request_nodes_and_tags(wayp_areas_it,wayp_areas_areait, curr_area_nodes,curr_area_type);
  request_nodes_and_tags(wayp_areas_it+1, wayp_areas_areait+1,next_area_nodes,next_area_type);
  
  bool is_curr_area_junction = is_junction(curr_area_type);
  bool is_next_area_junction = is_junction(next_area_type);
  if(is_curr_area_junction && is_next_area_junction)
      ROS_ERROR("Two adjacent junctions are not allowed");
  
  bool curr_area_is_corridor;
  if( !is_curr_area_junction )
  {
      curr_area_is_corridor = true;
      corridor_nodes_ptr = &curr_area_nodes;
      corridor_junction_nodes_ptr = &next_area_nodes;
  }
  else
  {
      curr_area_is_corridor = false;
      corridor_nodes_ptr = &next_area_nodes;
      corridor_junction_nodes_ptr = &curr_area_nodes;      
  }
     
  waypoint_overlap = compute_waypoint_areas_overlap(corridor_nodes_ptr,corridor_junction_nodes_ptr,curr_area_is_corridor);
  area_waypoint_overlap = *(wayp_areas_it);
  area_waypoint_overlap.waypoint_pose = waypoint_overlap;
  (*wayp_areas_areait)->waypoints.push_back(area_waypoint_overlap);

  curr_area_nodes.clear();
  std::copy(next_area_nodes.begin(),next_area_nodes.end(),std::back_inserter(curr_area_nodes));
  next_area_nodes.clear();
  curr_area_type  =  next_area_type;
  
  /* Next continue for other areas */
  for ( wayp_areas_it = path_osm_wayp.begin()+1; wayp_areas_it != (path_osm_wayp.end()-1); wayp_areas_it++) {      
    wayp_areas_areait++;

    

    // each area has a waypoint of the previous intersection. In the planner this is not a problem since intersections will be left out (The outmessage shoudl be different!)
    // Only when there is two adjacent corridors waypoints will be repeated. Later this can be checked.
//     area_waypoint_overlap = *(wayp_areas_it);
//     area_waypoint_overlap.waypoint_pose = waypoint_overlap;
//     (*wayp_areas_areait)->waypoints.push_back(area_waypoint_overlap);
    
    
    // request the nodes of the next area  
    request_nodes_and_tags(wayp_areas_it+1, wayp_areas_areait+1, next_area_nodes, next_area_type);
    is_curr_area_junction = is_junction(curr_area_type);
    is_next_area_junction = is_junction(next_area_type);
    if(is_curr_area_junction && is_next_area_junction)
        ROS_ERROR("Two adjacent junctions are not allowed");

    if( !is_curr_area_junction )
    {
        curr_area_is_corridor = true;
        corridor_nodes_ptr = &curr_area_nodes;
        corridor_junction_nodes_ptr = &next_area_nodes;
    }
    else
    {
        curr_area_is_corridor = false;
        corridor_nodes_ptr = &next_area_nodes;
        corridor_junction_nodes_ptr = &curr_area_nodes;      
    }  
            
    area_waypoint_overlap = *(wayp_areas_it);    
    waypoint_overlap = compute_waypoint_areas_overlap(corridor_nodes_ptr,corridor_junction_nodes_ptr, curr_area_is_corridor);
    area_waypoint_overlap.waypoint_pose = waypoint_overlap;
    (*wayp_areas_areait)->waypoints.push_back(area_waypoint_overlap);    
    
    // finally, copy next_area to current_area
    curr_area_nodes.clear();
    std::copy(next_area_nodes.begin(),next_area_nodes.end(),std::back_inserter(curr_area_nodes));
    next_area_nodes.clear();
    curr_area_type  =  next_area_type;

  }
  
  /* Finally process last area, add last found overlap */
  wayp_areas_it = (path_osm_wayp.end()-1);
  wayp_areas_areait = (wayp_area_iterators.end()-1);
  area_waypoint_overlap = *(wayp_areas_it);
  area_waypoint_overlap.waypoint_pose = waypoint_overlap;
  (*wayp_areas_areait)->waypoints.push_back(area_waypoint_overlap);
  
  
  
  // show results
  
  
    for (std::vector<ropod_ros_msgs::Area>::iterator area_it = path_areas.begin(); area_it != path_areas.end(); area_it++) {
        int no_of_waypts = 0;
        for (std::vector<ropod_ros_msgs::Waypoint>::iterator wayp_area_it = area_it->waypoints.begin(); wayp_area_it != area_it->waypoints.end(); wayp_area_it++)
        {
            std::cout << "Waypoint" << std::endl;
            std::cout << "Type: " << area_it->type << std::endl;
            std::cout << "ID: " << wayp_area_it->area_id << std::endl;
            std::cout << "pos(" << wayp_area_it->waypoint_pose.position.x << "," << wayp_area_it->waypoint_pose.position.y << ")" << std::endl; 
            std::cout << "quat(" << wayp_area_it->waypoint_pose.orientation.w << "," << wayp_area_it->waypoint_pose.orientation.x << "," << wayp_area_it->waypoint_pose.orientation.y << "," << wayp_area_it->waypoint_pose.orientation.z << ") \n" << std::endl; 
           
        }
    }  
  
  return path_areas;
}

std::vector<ropod_ros_msgs::Area> OSMWaypointsRoutePlanner::compute_orientations(std::vector<ropod_ros_msgs::Area> path_areas)
{
    // The other function is already computing orientations
    return path_areas;
}

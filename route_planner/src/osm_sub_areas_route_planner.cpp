/*
Computes route based on the intersection between corridors and intersections
*/
#include <route_planner/osm_sub_areas_route_planner.hpp>

#include <ropod_ros_msgs/RobotAction.h>

bool OSMSubAreasRoutePlanner::is_junction(std::string area_type)
{
    return (area_type == "junction" | area_type == "door");
}

double OSMSubAreasRoutePlanner::compute_distance_node_to_line(ropod_ros_msgs::Position corridor_node, ropod_ros_msgs::Position line_node_1, ropod_ros_msgs::Position line_node_2)
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
        if ( ( line_node_1.x <= x_line_closest && x_line_closest<= line_node_2.x ) && ( line_node_1.y <= y_line_closest && y_line_closest<= line_node_2.y ) )
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

geometry_msgs::Pose OSMSubAreasRoutePlanner::compute_sub_areas_overlap(std::vector<ropod_ros_msgs::Position>* corridor_nodes_ptr, std::vector<ropod_ros_msgs::Position>*  corridor_junction_nodes_ptr, bool curr_area_is_corridor)
{
    geometry_msgs::Pose sub_area_overlap;
    ropod_ros_msgs::Position mid_fursthest_point;

    // For each node of the corridor_nodes, check the closest distance to the lines that two adjacent nodes form of the corridor_or_intersection_nodes.
    // This is done because the waypoints always should be in a corridor entrance or exit.
    // Then the two closest nodes represent the overlap.
    // The orientation is determined by pointing otwards the corridor when curr-next areas are corridor-intersection or inwards when curr-next areas are intersection-corridor.
    std::vector<double> ordered_shortest_node_line_distances;
    std::vector<unsigned int> distance_ordered_node_indexes;
    unsigned int node_counter = 0;

    // we iterate until the penultimate node because the first and last nodes are always the same in the OSM map
    for ( auto corridor_nodes_it = (*corridor_nodes_ptr).begin(); corridor_nodes_it != (*corridor_nodes_ptr).end()-1; corridor_nodes_it++ )
    {
        // this iteration goes from the beginning to the one second-to-last.
        double shortest_dist_node_to_line = DBL_MAX;
        double dist_node_to_line;

        for ( auto corridor_intersection_nodes_it = (*corridor_junction_nodes_ptr).begin(); corridor_intersection_nodes_it != (*corridor_junction_nodes_ptr).end()-1 ; corridor_intersection_nodes_it++ )
        {
            // std::cout << "line tests" << std::endl;
            // std::cout << (*corridor_intersection_nodes_it).x << " , " << (*corridor_intersection_nodes_it).y << std::endl;
            // std::cout << (*(corridor_intersection_nodes_it+1)).x << " , " << (*(corridor_intersection_nodes_it+1)).y << std::endl;
            double dist_node_to_line = compute_distance_node_to_line(*corridor_nodes_it, *corridor_intersection_nodes_it,  *(corridor_intersection_nodes_it+1));

            if( dist_node_to_line < shortest_dist_node_to_line)
                shortest_dist_node_to_line = dist_node_to_line;
        }

        // Last iteration takes last node and the first one.
        dist_node_to_line = compute_distance_node_to_line(*corridor_nodes_it, * ((*corridor_junction_nodes_ptr).end()-1),  * (*corridor_junction_nodes_ptr).begin() );

        if( dist_node_to_line < shortest_dist_node_to_line)
                shortest_dist_node_to_line = dist_node_to_line;

        // we keep the node-to-line distances sorted and
        // keep track of the sorted node indices
        auto dist_iterator = ordered_shortest_node_line_distances.begin();
        auto node_idx_iterator = distance_ordered_node_indexes.begin();
        while (dist_iterator != ordered_shortest_node_line_distances.end() &&
               shortest_dist_node_to_line > *dist_iterator)
        {
            dist_iterator += 1;
            node_idx_iterator += 1;
        }
        ordered_shortest_node_line_distances.insert(dist_iterator, shortest_dist_node_to_line);
        distance_ordered_node_indexes.insert(node_idx_iterator, node_counter);

        node_counter += 1;
    }

    // we take the two shortest distances as well as the longest one
    double dist_closest_node = ordered_shortest_node_line_distances[0];
    double dist_second_closest_node = ordered_shortest_node_line_distances[1];
    double dist_farthest_node = ordered_shortest_node_line_distances[ordered_shortest_node_line_distances.size()-1];
    
    for (int it = 0; it<ordered_shortest_node_line_distances.size(); it++)
    {
        printf("\ndist node %d: %f",it, ordered_shortest_node_line_distances[it]);
    }

    // we take the two closest nodes as well as the farthest one
    ropod_ros_msgs::Position closest_node = (*corridor_nodes_ptr)[distance_ordered_node_indexes[0]];
    ropod_ros_msgs::Position second_closest_node = (*corridor_nodes_ptr)[distance_ordered_node_indexes[1]];
    ropod_ros_msgs::Position second_farthest_node = (*corridor_nodes_ptr)[distance_ordered_node_indexes[distance_ordered_node_indexes.size()-2]];
    ropod_ros_msgs::Position farthest_node = (*corridor_nodes_ptr)[distance_ordered_node_indexes.size()-1];

    /* Next compute the waypoint: position based on the middle line between the two closest nodes and orientation based on that and the farthest node */
    sub_area_overlap.position.x = 0.5*(closest_node.x + second_closest_node.x);
    sub_area_overlap.position.y = 0.5*(closest_node.y + second_closest_node.y);
    sub_area_overlap.position.z = 0.0;
    
    mid_fursthest_point.x = 0.5*(farthest_node.x + second_farthest_node.x);
    mid_fursthest_point.y = 0.5*(farthest_node.y + second_farthest_node.y);
    
    /*double yaw_waypoint_inwards = std::atan2(mid_fursthest_point.y-sub_area_overlap.position.y, mid_fursthest_point.x-sub_area_overlap.position.x);
    double yaw_waypoint_outwards = yaw_waypoint_inwards+ M_PI;
  */  
    
    // For the orientation we assume that the corridor is longer in the direction of movement
    double yaw_furthest_point_to_wayp = std::atan2(mid_fursthest_point.y-sub_area_overlap.position.y, mid_fursthest_point.x-sub_area_overlap.position.x);
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
    printf("\nyaw_waypoint_outwards %f",yaw_waypoint_outwards);
    printf("\nyaw_waypoint_inwards %f",yaw_waypoint_inwards);
    if( curr_area_is_corridor )
        yaw_waypoint =  yaw_waypoint_outwards;
    else
        yaw_waypoint =  yaw_waypoint_inwards;

    tf::Quaternion temp_quat;
    temp_quat.setRPY(0,0, yaw_waypoint);
    sub_area_overlap.orientation.w = temp_quat.getW();
    sub_area_overlap.orientation.x = temp_quat.getX();
    sub_area_overlap.orientation.y = temp_quat.getY();
    sub_area_overlap.orientation.z = temp_quat.getZ();

     std::cout << "\nNode process results" << std::endl;
     std::cout << closest_node.x << " , " << closest_node.y << std::endl;
     std::cout << second_closest_node.x << " , " << second_closest_node.y << std::endl;
     std::cout << sub_area_overlap.position.x << " , " << sub_area_overlap.position.y << std::endl;


    return sub_area_overlap;
}

bool OSMSubAreasRoutePlanner::request_nodes_and_tags(std::vector<ropod_ros_msgs::SubArea>::iterator sub_areas_it, std::vector<std::vector<ropod_ros_msgs::Area>::iterator>::iterator sub_areas_areait,std::vector<ropod_ros_msgs::Position>& area_nodes, std::string&  area_type)
{
    ropod_ros_msgs::Shape sub_area_shape;

//     if ((*sub_areas_areait)->type == "door")
//         sub_area_shape = RoutePlanner::CallGetShapeAction(std::stoi(sub_areas_it->id),"door");
//     else
        sub_area_shape = RoutePlanner::CallGetShapeAction(std::stoi(sub_areas_it->id),"local_area");
    if (sub_area_shape.vertices.empty())
    {
        ROS_ERROR_STREAM("GetShape for area " << sub_areas_it->id << " failed. Aborting.");
        return false;
    }

    area_nodes = sub_area_shape.vertices;
    std::cout << "Shape request" << std::endl;
    for (auto it_vert = area_nodes.begin(); it_vert != area_nodes.end(); it_vert++ )
    {
        //it_vert->y = - it_vert->y;
        std::cout << it_vert->x << " , " <<it_vert->y << std::endl;
    }
    area_type = (*sub_areas_areait)->type;
    return true;
}

std::vector<ropod_ros_msgs::Area> OSMSubAreasRoutePlanner::compute_route(std::vector<ropod_ros_msgs::Area> path_areas)
{


    std::vector<ropod_ros_msgs::Position> curr_area_nodes;
    std::vector<ropod_ros_msgs::Position> next_area_nodes;
    std::string curr_area_type;
    std::string next_area_type;
    std::vector<ropod_ros_msgs::Position>* corridor_nodes_ptr;
    std::vector<ropod_ros_msgs::Position>* corridor_junction_nodes_ptr;

    std::vector<ropod_ros_msgs::SubArea> path_osm_wayp; // Here we extract all osm waypoint which has a finer granularity than waypoints.
    std::vector<std::vector<ropod_ros_msgs::Area>::iterator> sub_area_iterators; // list of pointers to iterator corresponfign to each waypoint

    // Extract waypoints areas from areas

      for (std::vector<ropod_ros_msgs::Area>::iterator area_it = path_areas.begin(); area_it != path_areas.end(); area_it++) {
          int no_of_waypts = 0;

          // For doors, which doesn't have any local areas, create a virtual one
          if(area_it->type =="door")
          {
              ropod_ros_msgs::SubArea sub_area;
              sub_area.name = area_it->name;
              sub_area.id = area_it->id;
              area_it->sub_areas.push_back(sub_area);
          }
          
          for (std::vector<ropod_ros_msgs::SubArea>::iterator sub_area_it = area_it->sub_areas.begin(); sub_area_it != area_it->sub_areas.end(); sub_area_it++)
          {

              path_osm_wayp.push_back(*sub_area_it);
              sub_area_iterators.push_back(area_it);              
          }          
          area_it->sub_areas.clear(); // Clear because later we can insert more waypoints than originally we had
      }

    if (path_osm_wayp.size() < 2 )
    {
        ROS_ERROR("A minimum of two waypoint areas is needed");
        return path_areas;
    }

    ropod_ros_msgs::SubArea area_sub_area_overlap;
    geometry_msgs::Pose sub_area_overlap;

    /* First area is treated in a different way since only one waypoint is needed, not two */
    // Initially request the nodes for first and second area.
    std::vector<ropod_ros_msgs::SubArea>::iterator sub_areas_it = path_osm_wayp.begin();
    std::vector<std::vector<ropod_ros_msgs::Area>::iterator>::iterator sub_areas_areait = sub_area_iterators.begin();
    bool success = request_nodes_and_tags(sub_areas_it,sub_areas_areait, curr_area_nodes,curr_area_type);
    if (!success)
    {
        ROS_ERROR_STREAM("Error requesting nodes of first area");
        return path_areas;
    }
    success = request_nodes_and_tags(sub_areas_it+1, sub_areas_areait+1,next_area_nodes,next_area_type);
    if (!success)
    {
        ROS_ERROR_STREAM("Error requesting nodes of second area");
        return path_areas;
    }

    bool is_curr_area_junction = is_junction(curr_area_type);
    bool is_next_area_junction = is_junction(next_area_type);
    if(is_curr_area_junction && is_next_area_junction)
        ROS_ERROR("Two adjacent junctions are not allowed");
    
    printf("\ncurr and next areas types> %s, %s", curr_area_type.c_str(),  next_area_type.c_str());

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

    sub_area_overlap = compute_sub_areas_overlap(corridor_nodes_ptr,corridor_junction_nodes_ptr,curr_area_is_corridor);
    area_sub_area_overlap = *(sub_areas_it);
    area_sub_area_overlap.waypoint_pose = sub_area_overlap;
    (*sub_areas_areait)->sub_areas.push_back(area_sub_area_overlap);

    curr_area_nodes.clear();
    std::copy(next_area_nodes.begin(),next_area_nodes.end(),std::back_inserter(curr_area_nodes));
    next_area_nodes.clear();
    curr_area_type  =  next_area_type;

    /* Next continue for other areas */
    for ( sub_areas_it = path_osm_wayp.begin()+1; sub_areas_it != (path_osm_wayp.end()-1); sub_areas_it++) {
        sub_areas_areait++;

        // each area has a waypoint of the previous intersection. In the planner this is not a problem since intersections will be left out (The outmessage shoudl be different!)
        // Only when there is two adjacent corridors waypoints will be repeated. Later this can be checked.
        // area_sub_area_overlap = *(sub_areas_it);
        // area_sub_area_overlap.waypoint_pose = sub_area_overlap;
        // (*sub_areas_areait)->waypoints.push_back(area_sub_area_overlap);


        // request the nodes of the next area
        success = request_nodes_and_tags(sub_areas_it+1, sub_areas_areait+1, next_area_nodes, next_area_type);
        if (!success)
        {
            ROS_ERROR_STREAM("Error requesting nodes of next area");
            return path_areas;
        }
        
        is_curr_area_junction = is_junction(curr_area_type);
        is_next_area_junction = is_junction(next_area_type);
        if(is_curr_area_junction && is_next_area_junction)
            ROS_ERROR("Two adjacent junctions are not allowed");
        
        printf("\ncurr and next areas types> %s, %s", curr_area_type.c_str(),  next_area_type.c_str());

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

        area_sub_area_overlap = *(sub_areas_it);
        sub_area_overlap = compute_sub_areas_overlap(corridor_nodes_ptr,corridor_junction_nodes_ptr, curr_area_is_corridor);
        area_sub_area_overlap.waypoint_pose = sub_area_overlap;
        (*sub_areas_areait)->sub_areas.push_back(area_sub_area_overlap);

        // finally, copy next_area to current_area
        curr_area_nodes.clear();
        std::copy(next_area_nodes.begin(),next_area_nodes.end(),std::back_inserter(curr_area_nodes));
        next_area_nodes.clear();
        curr_area_type  =  next_area_type;
    }

    /* Finally process last area, add last found overlap */
    sub_areas_it = (path_osm_wayp.end()-1);
    sub_areas_areait = (sub_area_iterators.end()-1);
    area_sub_area_overlap = *(sub_areas_it);
    area_sub_area_overlap.waypoint_pose = sub_area_overlap;
    (*sub_areas_areait)->sub_areas.push_back(area_sub_area_overlap);

    // show results
    for (std::vector<ropod_ros_msgs::Area>::iterator area_it = path_areas.begin(); area_it != path_areas.end(); area_it++) {
        int no_of_waypts = 0;
        for (std::vector<ropod_ros_msgs::SubArea>::iterator sub_area_it = area_it->sub_areas.begin(); sub_area_it != area_it->sub_areas.end(); sub_area_it++)
        {
            std::cout << "SubArea" << std::endl;
            std::cout << "Type: " << area_it->type << std::endl;
            std::cout << "ID: " << sub_area_it->id << std::endl;
            std::cout << "pos(" << sub_area_it->waypoint_pose.position.x << "," << sub_area_it->waypoint_pose.position.y << ")" << std::endl;
            std::cout << "quat(" << sub_area_it->waypoint_pose.orientation.w << "," << sub_area_it->waypoint_pose.orientation.x << "," << sub_area_it->waypoint_pose.orientation.y << "," << sub_area_it->waypoint_pose.orientation.z << ") \n" << std::endl;
        }
    }

  return path_areas;
}

std::vector<ropod_ros_msgs::Area> OSMSubAreasRoutePlanner::compute_orientations(std::vector<ropod_ros_msgs::Area> path_areas)
{
    // The other function is already computing orientations
    return path_areas;
}

/*
ASSUMPTIONS
- each top level area in plan contains atleast one sub-area
- each sub area has four vertices
(both the assumptions are followed while creating OSM map) 
- start and destination cannot be a junction 

APPROACH
- Process the area-subarea level plan sent by CCU
- Take current and next sub area
- Compute center of both
- Use center of next sub-area to find 2 closest point of current sub-area to next sub-area
- Now use line connecting 2 sub-area centers to determine left and right point
- Among the remaining 2 points, find the point closest 2 front point. This point will be second point for left and right sides
- Similarly compute for all sub-areas
- For last sub areas, use reverse approach and then flip left and right sides
*/


#include <route_planner/napoleon_driving_planner.hpp>

std::vector<ropod_ros_msgs::Area> NapoleonDrivingPlanner::compute_route(std::vector<ropod_ros_msgs::Area> path_areas)
{
    // we treat doors as junctions
    for (int i = 0; i < path_areas.size(); i++)
    {   
        if (path_areas[i].sub_areas.empty())
        {
            ropod_ros_msgs::SubArea door_sub_area;
            door_sub_area.geometry = RoutePlanner::CallGetShapeAction(std::stoi(path_areas[i].id), "door"); 
            door_sub_area.type = "junction";
            // door_sub_area.id = path_areas[i].id;
            path_areas[i].type = "junction";
            path_areas[i].sub_areas.push_back(door_sub_area);  
        }
    }

    for (int i = 0; i < path_areas.size(); i++)
    {
        for (int j = 0; j < path_areas[i].sub_areas.size(); j++)
        {
            ropod_ros_msgs::Side right, left;

            // get area geometries from world model mediator only if it doesn't already exist
            if (path_areas[i].sub_areas[j].geometry.vertices.size() == 0)
                path_areas[i].sub_areas[j].geometry = RoutePlanner::CallGetShapeAction(std::stoi(path_areas[i].sub_areas[j].id), "local_area");

            // get area left and right sides and add intermediate sub-areas
            if (j < path_areas[i].sub_areas.size()-1 && i != path_areas.size()-1)
            {
                if (path_areas[i].sub_areas[j+1].geometry.vertices.size() == 0)
                    path_areas[i].sub_areas[j+1].geometry = RoutePlanner::CallGetShapeAction(std::stoi(path_areas[i].sub_areas[j+1].id), "local_area");
                get_area_sides(path_areas[i].sub_areas[j], path_areas[i].sub_areas[j+1], right, left);
            }
            else if (j == path_areas[i].sub_areas.size()-1 && i != path_areas.size()-1)
            {
                if (path_areas[i+1].sub_areas[0].geometry.vertices.size() == 0)
                    path_areas[i+1].sub_areas[0].geometry = RoutePlanner::CallGetShapeAction(std::stoi(path_areas[i+1].sub_areas[0].id), "local_area");
                get_area_sides(path_areas[i].sub_areas[j], path_areas[i+1].sub_areas[0], right, left);
            }
            else if (j == path_areas[i].sub_areas.size()-1 && i == path_areas.size()-1)
            {
                if (path_areas[i].sub_areas.size() == 1)
                    get_area_sides(path_areas[i].sub_areas[j], path_areas[i-1].sub_areas[path_areas[i-1].sub_areas.size()-1], right, left);
                else
                    get_area_sides(path_areas[i].sub_areas[j], path_areas[i].sub_areas[j-1], right, left);

                if (std::string(path_areas[i].sub_areas[j].type) == "junction")
                {
                    path_areas[i+1].sub_areas[j].turn_point = right.point2;
                }

                path_areas[i].sub_areas[j].right.point1 = left.point2; 
                path_areas[i].sub_areas[j].right.point2 = left.point1; 
                path_areas[i].sub_areas[j].left.point2 = right.point1; 
                path_areas[i].sub_areas[j].left.point1 = right.point2; 

                path_areas[i].sub_areas[j].geometry.vertices = {path_areas[i].sub_areas[j].left.point2, path_areas[i].sub_areas[j].left.point1, 
                                                       path_areas[i].sub_areas[j].right.point1, path_areas[i].sub_areas[j].right.point2};
                return add_intermediate_sub_areas(path_areas);
            }

            // Uncomment for debugging & analysing the generated plan
            /*
            std::cout << "Left 1," << left.point1.x << "," << left.point1.y << std::endl;
            std::cout << "Left 2," << left.point2.x << "," << left.point2.y << std::endl;
            std::cout << "Right 1," << right.point1.x << "," << right.point1.y << std::endl;
            std::cout << "Right 2," << right.point2.x << "," << right.point2.y << std::endl;
            */
            
            if (std::string(path_areas[i].sub_areas[j].type) == "junction")
            {
                path_areas[i].sub_areas[j].turn_point = left.point1;
            }

            path_areas[i].sub_areas[j].right = right; 
            path_areas[i].sub_areas[j].left = left;

            path_areas[i].sub_areas[j].geometry.vertices = {path_areas[i].sub_areas[j].left.point2, path_areas[i].sub_areas[j].left.point1, 
                                                   path_areas[i].sub_areas[j].right.point1, path_areas[i].sub_areas[j].right.point2};

        }
    }
    return add_intermediate_sub_areas(path_areas);
}

std::vector<ropod_ros_msgs::Area> NapoleonDrivingPlanner::add_intermediate_sub_areas(std::vector<ropod_ros_msgs::Area> path_areas)
{
    for (int i = 0; i < path_areas.size(); i++)
    {
        std::vector<ropod_ros_msgs::SubArea> new_sub_areas;

        for (int j = 0; j < path_areas[i].sub_areas.size(); j++)
        {
            new_sub_areas.push_back(path_areas[i].sub_areas[j]);
            if (j+1 == path_areas[i].sub_areas.size() && i+1 != path_areas.size())
            {
                ropod_ros_msgs::SubArea temp;

                if (path_areas[i+1].type == "junction")
                {
                    // find relative turn direction at the junction
                    std::string direction = get_turn_direction_at_junction(path_areas[i], path_areas[i+1], path_areas[i+2]);

                    if (direction == "left")
                    {
                        temp.geometry.vertices = {path_areas[i].sub_areas[j].geometry.vertices[1],
                                                  path_areas[i+1].sub_areas[0].geometry.vertices[3],
                                                  path_areas[i+1].sub_areas[0].geometry.vertices[2],
                                                  path_areas[i].sub_areas[j].geometry.vertices[2]};
                    }
                    else if (direction == "right")
                    {
                        temp.geometry.vertices = {path_areas[i].sub_areas[j].geometry.vertices[1],
                                                  path_areas[i+1].sub_areas[0].geometry.vertices[1],
                                                  path_areas[i+1].sub_areas[0].geometry.vertices[0],
                                                  path_areas[i].sub_areas[j].geometry.vertices[2]};
                    }
                    else if (direction == "straight")
                    {
                        temp.geometry.vertices = {path_areas[i].sub_areas[j].geometry.vertices[1],
                                                  path_areas[i+1].sub_areas[0].geometry.vertices[0],
                                                  path_areas[i+1].sub_areas[0].geometry.vertices[3],
                                                  path_areas[i].sub_areas[j].geometry.vertices[2]};
                    }
                    else
                    {
                        temp.geometry.vertices = {path_areas[i].sub_areas[j].geometry.vertices[1],
                                                  path_areas[i+1].sub_areas[0].geometry.vertices[2],
                                                  path_areas[i+1].sub_areas[0].geometry.vertices[1],
                                                  path_areas[i].sub_areas[j].geometry.vertices[2]};
                    }
                }
                else
                {
                    temp.geometry.vertices = {path_areas[i].sub_areas[j].geometry.vertices[1],
                                     path_areas[i+1].sub_areas[0].geometry.vertices[0],
                                     path_areas[i+1].sub_areas[0].geometry.vertices[3],
                                     path_areas[i].sub_areas[j].geometry.vertices[2]};
                }


                new_sub_areas.push_back(temp);
            }
            else if (j+1 < path_areas[i].sub_areas.size())
            {
                ropod_ros_msgs::SubArea temp;
                temp.geometry.vertices = {path_areas[i].sub_areas[j].geometry.vertices[1],
                                 path_areas[i].sub_areas[j+1].geometry.vertices[0],
                                 path_areas[i].sub_areas[j+1].geometry.vertices[3],
                                 path_areas[i].sub_areas[j].geometry.vertices[2]};
                new_sub_areas.push_back(temp);
            }
        }

        path_areas[i].sub_areas = new_sub_areas;

        // uncomment for debugging
        /*
        for (int k = 0; k < path_areas[i].sub_areas.size(); k++)
        {
            std::cout << path_areas[i].sub_areas[k].geometry.vertices[0].x << "," << path_areas[i].sub_areas[k].geometry.vertices[0].y << ","
                      << path_areas[i].sub_areas[k].geometry.vertices[1].x << "," << path_areas[i].sub_areas[k].geometry.vertices[1].y << ","
                      << path_areas[i].sub_areas[k].geometry.vertices[2].x << "," << path_areas[i].sub_areas[k].geometry.vertices[2].y << ","
                      << path_areas[i].sub_areas[k].geometry.vertices[3].x << "," << path_areas[i].sub_areas[k].geometry.vertices[3].y
                      << std::endl;
        }
        */
    }

    /**
    ensuring each junction has only 1 sub area
    following code is based on following assumptions 
    1. junction has only 1 sub-area in OSM
    2. there are no 2 continuous junctions
    3. there is never a junction at the end of the plan
    **/

    for (int i = 0; i < path_areas.size(); i++)
    {
        if(path_areas[i].type == "junction" && path_areas[i].sub_areas.size() == 2)
        {
            // move sub-area to next area
            path_areas[i+1].sub_areas.insert(path_areas[i+1].sub_areas.begin(), path_areas[i].sub_areas[1]);

            // delete sub-area from junction
            path_areas[i].sub_areas.erase(path_areas[i].sub_areas.end());
        }
    }

    return path_areas;
}

double NapoleonDrivingPlanner::wrap_to_pi(double angle)
{
    angle = fmod(angle, 2 * M_PI);
    if (angle >= M_PI)
        angle -= 2 * M_PI;
    else if (angle <= -M_PI)
        angle += 2 * M_PI;
    return angle;
}

std::string NapoleonDrivingPlanner::get_turn_direction_at_junction(ropod_ros_msgs::Area prev_area, ropod_ros_msgs::Area junction, ropod_ros_msgs::Area next_area)
{
    ropod_ros_msgs::Position prev_area_center = compute_center(prev_area.sub_areas[prev_area.sub_areas.size()-1].geometry);
    ropod_ros_msgs::Position junction_center = compute_center(junction.sub_areas[0].geometry);
    ropod_ros_msgs::Position next_area_center = compute_center(next_area.sub_areas[0].geometry);

    double junction_prev_area_angle = atan2(junction_center.y - prev_area_center.y, junction_center.x - prev_area_center.x); 
    double next_area_junction_angle = atan2(next_area_center.y - junction_center.y, next_area_center.x - junction_center.x);

    double approx_turn_angle = wrap_to_pi(next_area_junction_angle - junction_prev_area_angle);

    if (approx_turn_angle < M_PI/4 && approx_turn_angle > -M_PI/4)
        return "straight";
    else if (approx_turn_angle > M_PI/4 && approx_turn_angle < 3*M_PI/4)
        return "left";
    else if (approx_turn_angle < -M_PI/4 && approx_turn_angle > -3*M_PI/4)
        return "right";
    else
        return "back";
}


bool NapoleonDrivingPlanner::get_area_sides(ropod_ros_msgs::SubArea curr_area, ropod_ros_msgs::SubArea next_area, ropod_ros_msgs::Side &right, ropod_ros_msgs::Side &left)
{
    ropod_ros_msgs::Position curr_area_center = compute_center(curr_area.geometry);
    ropod_ros_msgs::Position next_area_center = compute_center(next_area.geometry);

    std::vector<ropod_ros_msgs::Position> front_points, rear_points;
    get_area_points(curr_area, next_area_center, front_points, rear_points);

    // std::cout << front_points[0] << std::endl;
    // std::cout << front_points[1] << std::endl;

    ropod_ros_msgs::Position front_mid_point;
    front_mid_point.x = (front_points[0].x + front_points[1].x)/2.0;
    front_mid_point.y = (front_points[0].y + front_points[1].y)/2.0;

    ropod_ros_msgs::Position rear_mid_point;
    rear_mid_point.x = (rear_points[0].x + rear_points[1].x)/2.0;
    rear_mid_point.y = (rear_points[0].y + rear_points[1].y)/2.0;

    Line center_line;
    center_line.point1 = rear_mid_point;
    center_line.point2 = front_mid_point;

    if (determine_point_side(center_line, front_points[0]) == RIGHT)
    {
        right.point1 = front_points[0];
        left.point1 = front_points[1];
    }
    else
    {
        left.point1 = front_points[0];
        right.point1 = front_points[1];
    }

    if (determine_point_side(center_line, rear_points[0]) == RIGHT)
    {
        right.point2 = rear_points[0];
        left.point2 = rear_points[1];
    }
    else
    {
        left.point2 = rear_points[0];
        right.point2 = rear_points[1];
    }

    return true;
}

void NapoleonDrivingPlanner::get_area_points(ropod_ros_msgs::SubArea area, ropod_ros_msgs::Position pt, std::vector<ropod_ros_msgs::Position> &front_points, std::vector<ropod_ros_msgs::Position> &rear_points)
{
    std::vector<std::pair<ropod_ros_msgs::Position, double>> distances;

    std::vector<ropod_ros_msgs::Position> neighbouring_points;

    // NOTE: first point is repeated in returned geometries
    for (int i = 0; i < area.geometry.vertices.size() - 1; i++)
    {
        std::pair<ropod_ros_msgs::Position, double> temp;
        temp = std::make_pair(area.geometry.vertices[i], get_euclidean_distance(area.geometry.vertices[i], pt));
        distances.push_back(temp);
    }

    sort(distances.begin(), distances.end(), [ ]( const std::pair<ropod_ros_msgs::Position, double>& lhs, const std::pair<ropod_ros_msgs::Position, double>& rhs )
    {
        return lhs.second < rhs.second;
    });

    // NOTE: based on assumptions that each area has only four points
    if (get_euclidean_distance(distances[0].first, distances[1].first) < 0.3)
    {
        /*
        In case of door its not necessary that 2 farthest points are front points, so we check
        if distance between farthest points is greater then 0.3 m (assumnng width of sub-area will be always
        greater then 0.3m since ROPOD size is 0.6m)
        */
        front_points = {distances[0].first, distances[2].first};
        rear_points = {distances[1].first, distances[3].first}; 
    }
    else
    {
        front_points = {distances[0].first, distances[1].first};
        rear_points = {distances[2].first, distances[3].first};
    }
}

double NapoleonDrivingPlanner::get_euclidean_distance(ropod_ros_msgs::Position pt1, ropod_ros_msgs::Position pt2)
{
    return pow(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2), 0.5);
}

Side NapoleonDrivingPlanner::determine_point_side(Line line, ropod_ros_msgs::Position pt)
{
    double d = (pt.x - line.point1.x)*(line.point2.y - line.point1.y) - (pt.y - line.point1.y)*(line.point2.x - line.point1.x);
    if (d > 0)
        return LEFT;
    else 
        return RIGHT;
}

ropod_ros_msgs::Position NapoleonDrivingPlanner::compute_center(ropod_ros_msgs::Shape geometry)
{
    int no_of_points = geometry.vertices.size();
    ropod_ros_msgs::Position center;
    for (int i = 0; i < no_of_points; i++)
    {
        center.x = center.x + geometry.vertices[i].x;
        center.y = center.y + geometry.vertices[i].y;
    }
    center.x = center.x/no_of_points;
    center.y = center.y/no_of_points;
    return center;
}

std::vector<ropod_ros_msgs::Area> NapoleonDrivingPlanner::compute_orientations(std::vector<ropod_ros_msgs::Area> path_areas)
{
    return path_areas;
}


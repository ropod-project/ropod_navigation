/*
ASSUMPTIONS
- each top level area in plan contains atleast one sub-area
- each sub area has four vertices
(both the assumptions are followed while creating OSM map) 

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
    for (int i = 0; i < path_areas.size(); i++)
    {
        for (int j = 0; j < path_areas[i].sub_areas.size(); j++)
        {
            ropod_ros_msgs::Side right, left;
            if (path_areas[i].sub_areas[j].geometry.vertices.size() == 0)
                    path_areas[i].sub_areas[j].geometry = RoutePlanner::CallGetShapeAction(std::stoi(path_areas[i].sub_areas[j].id), "local_area");

            if (j < path_areas[i].sub_areas.size()-1 && i != path_areas.size()-1)
            {
                path_areas[i].sub_areas[j+1].geometry = RoutePlanner::CallGetShapeAction(std::stoi(path_areas[i].sub_areas[j+1].id), "local_area");
                get_area_sides(path_areas[i].sub_areas[j], path_areas[i].sub_areas[j+1], right, left);
            }
            else if (j == path_areas[i].sub_areas.size()-1 && i != path_areas.size()-1)
            {
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

            /*
            // Uncomment for debugging & analysing the generated plan
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
        // std::cout << "Initial size:" << path_areas[i].sub_areas.size() << std::endl;
        for (int j = 0; j < path_areas[i].sub_areas.size(); j++)
        {
            new_sub_areas.push_back(path_areas[i].sub_areas[j]);
            if (j+1 == path_areas[i].sub_areas.size() && i+1 != path_areas.size())
            {
                ropod_ros_msgs::SubArea temp;
                temp.geometry.vertices = {path_areas[i].sub_areas[j].geometry.vertices[1],
                                 path_areas[i+1].sub_areas[0].geometry.vertices[0],
                                 path_areas[i+1].sub_areas[0].geometry.vertices[3],
                                 path_areas[i].sub_areas[j].geometry.vertices[2]};
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

        for (int k = 0; k < path_areas[i].sub_areas.size(); k++)
        {
            std::cout << path_areas[i].sub_areas[k].geometry.vertices[0].x << "," << path_areas[i].sub_areas[k].geometry.vertices[0].y << ","
                      << path_areas[i].sub_areas[k].geometry.vertices[1].x << "," << path_areas[i].sub_areas[k].geometry.vertices[1].y << ","
                      << path_areas[i].sub_areas[k].geometry.vertices[2].x << "," << path_areas[i].sub_areas[k].geometry.vertices[2].y << ","
                      << path_areas[i].sub_areas[k].geometry.vertices[3].x << "," << path_areas[i].sub_areas[k].geometry.vertices[3].y
                      << std::endl;
        }
        // std::cout << "New size:" << path_areas[i].sub_areas.size() << std::endl;
        // std::cout << "----------------------" << std::endl;
    }

    return path_areas;
}


bool NapoleonDrivingPlanner::get_area_sides(ropod_ros_msgs::SubArea curr_area, ropod_ros_msgs::SubArea next_area, ropod_ros_msgs::Side &right, ropod_ros_msgs::Side &left)
{
    ropod_ros_msgs::Position curr_area_center = compute_center(curr_area.geometry);
    ropod_ros_msgs::Position next_area_center = compute_center(next_area.geometry);

    std::vector<ropod_ros_msgs::Position> front_points, rear_points;
    get_area_points(curr_area, next_area_center, front_points, rear_points);

    // std::cout << front_points[0] << std::endl;
    // std::cout << front_points[1] << std::endl;

    Line center_line;
    center_line.point1 = curr_area_center;
    center_line.point2 = next_area_center;

    // std::cout << determine_point_side(center_line, front_points[0]) << std::endl;
    // std::cout << determine_point_side(center_line, front_points[1]) << std::endl;

    if (determine_point_side(center_line, front_points[0]) == 0)
        right.point1 = front_points[0];
    else
        left.point1 = front_points[0];

    if (determine_point_side(center_line, front_points[1]) == 0)
        right.point1 = front_points[1];
    else
        left.point1 = front_points[1];

    if (get_euclidean_distance(right.point1, rear_points[0]) > get_euclidean_distance(right.point1, rear_points[1]))
        right.point2 = rear_points[1];
    else
        right.point2 = rear_points[0];

    if (get_euclidean_distance(left.point1, rear_points[0]) > get_euclidean_distance(left.point1, rear_points[1]))
        left.point2 = rear_points[1];
    else
        left.point2 = rear_points[0];

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

    // NOTE: its based on assumptions that each area has only four points
    front_points = {distances[0].first, distances[1].first};
    rear_points = {distances[2].first, distances[3].first};
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


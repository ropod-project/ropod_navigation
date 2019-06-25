#include <route_planner/napoleon_driving_planner.hpp>

std::vector<ropod_ros_msgs::Area> NapoleonDrivingPlanner::compute_route(std::vector<ropod_ros_msgs::Area> path_areas)
{
    std::vector<ropod_ros_msgs::SubArea> sub_area_level_plan;

    for (int i = 0; i < path_areas.size(); i++)
    {
        for (int j = 0; j < path_areas[i].sub_areas.size(); j++)
        {
            ropod_ros_msgs::SubArea temp;
            temp.id = path_areas[i].sub_areas[j].id;
            temp.name = path_areas[i].sub_areas[j].name;
            temp.floor_number = path_areas[i].floor_number;
            temp.type = path_areas[i].type;
            temp.geometry = RoutePlanner::CallGetShapeAction(std::stoi(temp.id),"local_area");
            // std::cout << temp << std::endl;
            sub_area_level_plan.push_back(temp);
        }
    }

    for (int i = 0; i < sub_area_level_plan.size(); i++)
    {
        if (i != sub_area_level_plan.size())
        {

        }
    }

    // Line line;
    // line.point1.x = 0; 
    // line.point1.y = 0;

    // line.point2.x = 4; 
    // line.point2.y = 0;

    // std::cout << "cool" << std::endl;

    // ropod_ros_msgs::Position pt;
    // pt.x = 2;
    // pt.y = -2;
    // std::cout << determine_point_side(line, pt) << std::endl;

    // get_two_nearest_points(sub_area_level_plan[0], sub_area_level_plan[0].geometry.vertices[0]);

    return path_areas;
}

std::vector<ropod_ros_msgs::Position> NapoleonDrivingPlanner::get_two_nearest_points(ropod_ros_msgs::SubArea area, ropod_ros_msgs::Position pt)
{
    std::vector<std::pair<ropod_ros_msgs::Position, double>> distances;

    std::vector<ropod_ros_msgs::Position> neighbouring_points;
    for (int i = 0; i < area.geometry.vertices.size(); i++)
    {
        std::pair<ropod_ros_msgs::Position, double> temp;
        temp = std::make_pair(area.geometry.vertices[i], get_euclidean_distance(area.geometry.vertices[i], pt));
        distances.push_back(temp);
    }

    sort(distances.begin(), distances.end(), [ ]( const std::pair<ropod_ros_msgs::Position, double>& lhs, const std::pair<ropod_ros_msgs::Position, double>& rhs )
    {
        return lhs.second < rhs.second;
    });

    neighbouring_points = {distances[0].first, distances[1].first};
    return neighbouring_points;
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

    std::cout << center << std::endl;

    return center;
}

std::vector<ropod_ros_msgs::Area> NapoleonDrivingPlanner::compute_orientations(std::vector<ropod_ros_msgs::Area> path_areas)
{
    return path_areas;
}


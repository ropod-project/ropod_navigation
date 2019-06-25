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
        }
    }
    return path_areas;
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


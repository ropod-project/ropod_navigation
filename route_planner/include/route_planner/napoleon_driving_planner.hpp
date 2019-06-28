#ifndef NAPOLEON_DRIVING_PLANNER_HPP
#define NAPOLEON_DRIVING_PLANNER_HPP

#include <route_planner/route_planner.hpp>

struct Line
{
    ropod_ros_msgs::Position point1;
    ropod_ros_msgs::Position point2;
};

enum Side
{
    RIGHT = 0,
    LEFT = 1
};

class NapoleonDrivingPlanner: public RoutePlanner
{
public:
    virtual std::vector<ropod_ros_msgs::Area> compute_route(std::vector<ropod_ros_msgs::Area>);
    virtual std::vector<ropod_ros_msgs::Area> compute_orientations(std::vector<ropod_ros_msgs::Area>); 

private:
    ropod_ros_msgs::Position compute_center(ropod_ros_msgs::Shape geometry);
    Side determine_point_side(Line, ropod_ros_msgs::Position pt);
    double get_euclidean_distance(ropod_ros_msgs::Position pt1, ropod_ros_msgs::Position pt2);
    void get_area_points(ropod_ros_msgs::SubArea area, ropod_ros_msgs::Position pt, std::vector<ropod_ros_msgs::Position> &front_points, std::vector<ropod_ros_msgs::Position> &rear_points);
    bool get_area_sides(ropod_ros_msgs::SubArea curr_area, ropod_ros_msgs::SubArea next_area, ropod_ros_msgs::Side &right, ropod_ros_msgs::Side &left);
};

#endif /* NAPOLEON_DRIVING_PLANNER */

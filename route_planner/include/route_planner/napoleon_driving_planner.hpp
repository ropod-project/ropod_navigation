#ifndef NAPOLEON_DRIVING_PLANNER_HPP
#define NAPOLEON_DRIVING_PLANNER_HPP

#include <route_planner/route_planner.hpp>

class NapoleonDrivingPlanner: public RoutePlanner
{
public:
    virtual std::vector<ropod_ros_msgs::Area> compute_route(std::vector<ropod_ros_msgs::Area>);
    virtual std::vector<ropod_ros_msgs::Area> compute_orientations(std::vector<ropod_ros_msgs::Area>); 

private:
    ropod_ros_msgs::Position compute_center(ropod_ros_msgs::Shape geometry);
};

#endif /* NAPOLEON_DRIVING_PLANNER */

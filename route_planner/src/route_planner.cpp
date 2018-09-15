/*
An abstract class for route planning. Provides 'route_planner' action server which takes list of areas sent by CCU as inputs 
and returns list of area with waypoint (poses) in local co-ordinate system 

'compute_route' and 'compute_orientation' are pure virtual method which needs to be implemented in derived class. 
*/
#include <route_planner/route_planner.hpp>

RoutePlanner::RoutePlanner(): nh("~"),route_planner_server(nh,"/route_planner",
  boost::bind(&RoutePlanner::RoutePlannerExecute, this, _1),false),get_waypt_position_action_client("/get_waypt_position", true), 
  get_waypt_shape_action_client("/get_waypt_shape", true), waypt_position_result(), waypt_shape_result()
{
    route_planner_server.start();
    get_waypt_position_action_client.waitForServer();
    get_waypt_shape_action_client.waitForServer();
}

RoutePlanner::~RoutePlanner()
{
}

void RoutePlanner::RoutePlannerExecute(const ropod_ros_msgs::RoutePlannerGoalConstPtr& req)
{
    std::vector<ropod_ros_msgs::Area> path_areas;
    ropod_ros_msgs::RoutePlannerResult route_planner_result;
    for(std::vector<ropod_ros_msgs::Area>::const_iterator curr_area = req->areas.begin(); curr_area != req->areas.end(); ++curr_area)
    {
      path_areas.push_back(*curr_area);
    }
    std::vector<ropod_ros_msgs::Area> path_areas2 = this->compute_route(path_areas);  // implemented in derived classes
    route_planner_result.areas = compute_orientations(path_areas2);
    route_planner_server.setSucceeded(route_planner_result);
}


void RoutePlanner::GetWayptPositionResultCb(const actionlib::SimpleClientGoalState& state, const ropod_ros_msgs::GetWayptPositionResultConstPtr& result)
{
    waypt_position_result = *result;
}

void RoutePlanner::GetWayptShapeResultCb(const actionlib::SimpleClientGoalState& state, const ropod_ros_msgs::GetWayptShapeResultConstPtr& result)
{
    waypt_shape_result = *result;
}

ropod_ros_msgs::Position RoutePlanner::CallGetWayptPositionAction(int id)
{
    ropod_ros_msgs::GetWayptPositionGoal req;
    req.ids = {id};
    ropod_ros_msgs::Position pos;
    get_waypt_position_action_client.sendGoal(req, boost::bind(&RoutePlanner::GetWayptPositionResultCb, this, _1, _2));
    bool finished_before_timeout = get_waypt_position_action_client.waitForResult(ros::Duration(5.0));
    if (get_waypt_position_action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        for (auto it = waypt_position_result.positions.begin(); it != waypt_position_result.positions.end(); it++)
        {
            pos = *it;
        }
    }
    return pos;
}

ropod_ros_msgs::Shape RoutePlanner::CallGetWayptShapeAction(int id)
{
    ropod_ros_msgs::GetWayptShapeGoal req;
    req.ids = {id};
    ropod_ros_msgs::Shape shape;
    get_waypt_shape_action_client.sendGoal(req, boost::bind(&RoutePlanner::GetWayptShapeResultCb, this, _1, _2));
    bool finished_before_timeout = get_waypt_shape_action_client.waitForResult(ros::Duration(5.0));
    if (get_waypt_shape_action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        for (auto it = waypt_shape_result.shapes.begin(); it != waypt_shape_result.shapes.end(); it++)
        {
            shape = *it;
        }
    }
    return shape;
}

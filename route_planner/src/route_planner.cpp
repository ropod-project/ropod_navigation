/*
An abstract class for route planning. Provides 'route_planner' action server which takes list of areas sent by CCU as inputs 
and returns list of area with waypoint (poses) in local co-ordinate system 

'compute_route' and 'compute_orientation' are pure virtual method which needs to be implemented in derived class. 
*/
#include <route_planner/route_planner.hpp>

RoutePlanner::RoutePlanner(): nh("~"),route_planner_server(nh,"/route_planner",
  boost::bind(&RoutePlanner::RoutePlannerExecute, this, _1),false),get_topology_node_action_client("/get_topology_node", true), 
  get_shape_action_client("/get_shape", true), topology_node_result(), shape_result()
{
    route_planner_server.start();
    get_topology_node_action_client.waitForServer();
    get_shape_action_client.waitForServer();
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


void RoutePlanner::GetTopologyNodeResultCb(const actionlib::SimpleClientGoalState& state, const ropod_ros_msgs::GetTopologyNodeResultConstPtr& result)
{
    topology_node_result = *result;
}

void RoutePlanner::GetShapeResultCb(const actionlib::SimpleClientGoalState& state, const ropod_ros_msgs::GetShapeResultConstPtr& result)
{
    shape_result = *result;
}

ropod_ros_msgs::Position RoutePlanner::CallGetTopologyNodeAction(int id, std::string entity_type)
{
    ropod_ros_msgs::GetTopologyNodeGoal req;
    req.id = id;
    req.type = entity_type;
    ropod_ros_msgs::Position pos;
    get_topology_node_action_client.sendGoal(req, boost::bind(&RoutePlanner::GetTopologyNodeResultCb, this, _1, _2));
    bool finished_before_timeout = get_topology_node_action_client.waitForResult(ros::Duration(5.0));
    if (get_topology_node_action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        pos = topology_node_result.position;
    }
    return pos;
}

ropod_ros_msgs::Shape RoutePlanner::CallGetShapeAction(int id, std::string entity_type)
{
    ropod_ros_msgs::GetShapeGoal req;
    req.id = id;
    req.type = entity_type;
    ropod_ros_msgs::Shape shape;
    get_shape_action_client.sendGoal(req, boost::bind(&RoutePlanner::GetShapeResultCb, this, _1, _2));
    bool finished_before_timeout = get_shape_action_client.waitForResult(ros::Duration(5.0));
    if (get_shape_action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        shape = shape_result.shape;
    }
    return shape;
}

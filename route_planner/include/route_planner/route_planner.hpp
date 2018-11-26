#ifndef ROUTE_PLANNER_HPP
#define ROUTE_PLANNER_HPP

/* ROS includes */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

/* C++ */
#include <iostream>
#include <queue>
#include <string>


/* ROPOD messages */
#include <ropod_ros_msgs/Area.h>
#include <ropod_ros_msgs/SubArea.h>
#include <ropod_ros_msgs/Position.h>
#include <ropod_ros_msgs/Shape.h>

#include <ropod_ros_msgs/RoutePlannerAction.h>
#include <ropod_ros_msgs/GetTopologyNodeAction.h> 
#include <ropod_ros_msgs/GetShapeAction.h> 


class RoutePlanner
{
protected:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<ropod_ros_msgs::RoutePlannerAction> route_planner_server;
    actionlib::SimpleActionClient<ropod_ros_msgs::GetTopologyNodeAction> get_topology_node_action_client;
    actionlib::SimpleActionClient<ropod_ros_msgs::GetShapeAction> get_shape_action_client;
    ropod_ros_msgs::GetTopologyNodeResult topology_node_result;
    ropod_ros_msgs::GetShapeResult shape_result;
    
    void GetTopologyNodeResultCb(const actionlib::SimpleClientGoalState& state,const ropod_ros_msgs::GetTopologyNodeResultConstPtr& result);
    void GetShapeResultCb(const actionlib::SimpleClientGoalState& state,const ropod_ros_msgs::GetShapeResultConstPtr& result);
    void RoutePlannerExecute(const ropod_ros_msgs::RoutePlannerGoalConstPtr& goal);
    ropod_ros_msgs::Position CallGetTopologyNodeAction(int id, std::string entity_type);
    ropod_ros_msgs::Shape CallGetShapeAction(int id, std::string entity_type);

public:
    RoutePlanner();
    virtual ~RoutePlanner();
    virtual std::vector<ropod_ros_msgs::Area> compute_route(std::vector<ropod_ros_msgs::Area>) = 0;
    virtual std::vector<ropod_ros_msgs::Area> compute_orientations(std::vector<ropod_ros_msgs::Area>) = 0;
};

#endif /* ROUTE_PLANNER_HPP */

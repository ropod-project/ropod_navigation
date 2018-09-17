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


/* ROPOD messages */
#include <ropod_ros_msgs/Area.h>
#include <ropod_ros_msgs/Waypoint.h>
#include <ropod_ros_msgs/Position.h>
#include <ropod_ros_msgs/Shape.h>

#include <ropod_ros_msgs/RoutePlannerAction.h>
#include <ropod_ros_msgs/GetWayptPositionAction.h> 
#include <ropod_ros_msgs/GetWayptShapeAction.h> 


class RoutePlanner
{
protected:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<ropod_ros_msgs::RoutePlannerAction> route_planner_server;
    actionlib::SimpleActionClient<ropod_ros_msgs::GetWayptPositionAction> get_waypt_position_action_client;
    actionlib::SimpleActionClient<ropod_ros_msgs::GetWayptShapeAction> get_waypt_shape_action_client;
    ropod_ros_msgs::GetWayptPositionResult waypt_position_result;
    ropod_ros_msgs::GetWayptShapeResult waypt_shape_result;
    
    void GetWayptPositionResultCb(const actionlib::SimpleClientGoalState& state,const ropod_ros_msgs::GetWayptPositionResultConstPtr& result);
    void GetWayptShapeResultCb(const actionlib::SimpleClientGoalState& state,const ropod_ros_msgs::GetWayptShapeResultConstPtr& result);
    void RoutePlannerExecute(const ropod_ros_msgs::RoutePlannerGoalConstPtr& goal);
    ropod_ros_msgs::Position CallGetWayptPositionAction(int id);
    ropod_ros_msgs::Shape CallGetWayptShapeAction(int id);

public:
    RoutePlanner();
    virtual ~RoutePlanner();
    virtual std::vector<ropod_ros_msgs::Area> compute_route(std::vector<ropod_ros_msgs::Area>) = 0;
    virtual std::vector<ropod_ros_msgs::Area> compute_orientations(std::vector<ropod_ros_msgs::Area>) = 0;
};

#endif /* ROUTE_PLANNER_HPP */

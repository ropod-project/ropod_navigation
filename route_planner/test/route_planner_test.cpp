/*
Provides simple example path for independent testing of this package
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <array>
#include <actionlib/client/simple_action_client.h>
#include <ropod_ros_msgs/Area.h>
#include <ropod_ros_msgs/Waypoint.h>
#include <ropod_ros_msgs/RoutePlannerAction.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "route_planner_test");
    ros::NodeHandle nh("~");

    ROS_INFO("Route planner test");

    std::vector<ropod_ros_msgs::Area>path_areas;

    ropod_ros_msgs::Area temp1;
    temp1.area_id = std::to_string(3122);
    temp1.name = "AMK_B_L-1_C30";
    ropod_ros_msgs::Waypoint wtemp1;
    wtemp1.semantic_id = "AMK_B_L-1_C30_LA1";
    wtemp1.area_id = "4857";
    temp1.waypoints.push_back(wtemp1);
    path_areas.push_back(temp1);

    ropod_ros_msgs::Area temp2;
    temp2.area_id = std::to_string(3113);
    temp2.name = "AMK_B_L-1_C29_Door1";
    // ropod_ros_msgs::Waypoint wtemp2;
    // wtemp2.semantic_id = "AMK_B_L-1_C29_Door1";
    // wtemp2.area_id = "3113";
    // temp2.waypoints.push_back(wtemp2);
    path_areas.push_back(temp2);

    ropod_ros_msgs::Area temp3;
    temp3.area_id = std::to_string(4970);
    temp3.name = "AMK_B_L-1_C29";
    ropod_ros_msgs::Waypoint wtemp3;
    wtemp3.semantic_id = "AMK_B_L-1_C29_LA1";
    wtemp3.area_id = "4856";
    temp3.waypoints.push_back(wtemp3);  
    path_areas.push_back(temp3);

    ropod_ros_msgs::Area temp4;
    temp4.area_id = std::to_string(4971);
    temp4.name = "AMK_B_L-1_C28";
    ropod_ros_msgs::Waypoint wtemp4;
    wtemp4.semantic_id = "AMK_B_L-1_C28_LA1";
    wtemp4.area_id = "4855";
    temp4.waypoints.push_back(wtemp4);  
    path_areas.push_back(temp4);

    ropod_ros_msgs::Area temp5;
    temp5.area_id = std::to_string(3123);
    temp5.name = "AMK_B_L-1_C27";
    ropod_ros_msgs::Waypoint wtemp5;
    wtemp5.semantic_id = "AMK_B_L-1_C27_LA1";
    wtemp5.area_id = "4866";
    temp5.waypoints.push_back(wtemp5);  
    path_areas.push_back(temp5);


    actionlib::SimpleActionClient<ropod_ros_msgs::RoutePlannerAction> route_planner_action_client("/route_planner",true);

    route_planner_action_client.waitForServer();

    ropod_ros_msgs::RoutePlannerGoal req;
    req.areas = path_areas;
    route_planner_action_client.sendGoal(req);

    if (route_planner_action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Conenction successful");
    }


    return 0;
}
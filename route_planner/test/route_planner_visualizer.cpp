/*
Visulises planned route
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <array>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseArray.h>
#include <ropod_ros_msgs/Area.h>
#include <ropod_ros_msgs/SubArea.h>
#include <ropod_ros_msgs/RoutePlannerAction.h>

ros::Subscriber route_planner_result_subscriber;
ros::Publisher pose_array_publisher;

void routePlannerResultCallback(const ropod_ros_msgs::RoutePlannerActionResult::ConstPtr& msg)
{
    ROS_INFO("Route planner result received");
    geometry_msgs::PoseArray calculated_poses;
    calculated_poses.header.frame_id = "map";

    for (int i = 0; i < msg->result.areas.size(); i++)
    {
        for (int j = 0; j < msg->result.areas[i].sub_areas.size(); j++)
        {
            calculated_poses.poses.push_back(msg->result.areas[i].sub_areas[j].waypoint_pose);
        }
    }
    pose_array_publisher.publish(calculated_poses);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "route_planner_visualizer");
    ros::NodeHandle nh("~");

    ROS_INFO("Route planner result visualizer");

    route_planner_result_subscriber = nh.subscribe("/route_planner/result", 1, routePlannerResultCallback);
    pose_array_publisher = nh.advertise<geometry_msgs::PoseArray>("/route_planner_visualizer", 1);

    ros::spin(); 
    return 0;
}
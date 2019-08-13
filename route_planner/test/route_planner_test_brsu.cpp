/*
Provides simple example path for independent testing of this package
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <array>
#include <actionlib/client/simple_action_client.h>
#include <ropod_ros_msgs/Area.h>
#include <ropod_ros_msgs/SubArea.h>
#include <ropod_ros_msgs/RoutePlannerAction.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "route_planner_test_brsu");
    ros::NodeHandle nh("~");

    ROS_INFO("Route planner test brsu");

    std::vector<ropod_ros_msgs::Area>path_areas;
    
    ropod_ros_msgs::Area temp1;
    temp1.id = std::to_string(5);
    temp1.name = "BRSU_C_L0_C9";
    temp1.type = "corridor";
    ropod_ros_msgs::SubArea stemp1;
    stemp1.name = "BRSU_C_L0_C9_LA1";
    stemp1.id = std::to_string(61);
    temp1.sub_areas.push_back(stemp1);
    path_areas.push_back(temp1);

    ropod_ros_msgs::Area temp2;
    temp2.id = std::to_string(7);
    temp2.name = "BRSU_C_L0_C8";
    temp2.type = "junction";
    ropod_ros_msgs::SubArea stemp2;
    stemp2.name = "BRSU_C_L0_C8_LA1";
    stemp2.id = std::to_string(64);
    temp2.sub_areas.push_back(stemp2);
    path_areas.push_back(temp2);

    // For right turn at junction
    // ropod_ros_msgs::Area temp3;
    // ropod_ros_msgs::SubArea stemp3;
    // temp3.id = std::to_string(2);
    // temp3.name = "BRSU_C_L0_C11";
    // temp3.type = "corridor";
    // stemp3.name = "BRSU_C_L0_C11_LA2";
    // stemp3.id = std::to_string(57);
    // temp3.sub_areas.push_back(stemp3);
    // path_areas.push_back(temp3);

    ropod_ros_msgs::Area temp3;
    ropod_ros_msgs::SubArea stemp3;
    temp3.id = std::to_string(92);
    temp3.name = "BRSU_C_L0_C7";
    temp3.type = "corridor";
    stemp3.name = "BRSU_C_L0_C7_LA1";
    stemp3.id = std::to_string(66);
    temp3.sub_areas.push_back(stemp3);
    path_areas.push_back(temp3);    

    ropod_ros_msgs::Area temp4;
    ropod_ros_msgs::SubArea stemp4;
    temp4.id = std::to_string(8);
    temp4.name = "BRSU_C_L0_C6";
    temp4.type = "corridor";
    stemp4.name = "BRSU_C_L0_C6_LA1";
    stemp4.id = std::to_string(68);
    temp4.sub_areas.push_back(stemp4);
    path_areas.push_back(temp4);

    ropod_ros_msgs::Area temp5;
    ropod_ros_msgs::SubArea stemp5;
    temp5.id = std::to_string(9);
    temp5.name = "BRSU_C_L0_C5";
    temp5.type = "corridor";
    stemp5.name = "BRSU_C_L0_C5_LA1";
    stemp5.id = std::to_string(70);
    temp5.sub_areas.push_back(stemp5);
    path_areas.push_back(temp5);

    ropod_ros_msgs::Area temp6;
    ropod_ros_msgs::SubArea stemp6;
    temp6.id = std::to_string(10);
    temp6.name = "BRSU_C_L0_C4";
    temp6.type = "corridor";
    stemp6.name = "BRSU_C_L0_C4_LA1";
    stemp6.id = std::to_string(72);
    temp6.sub_areas.push_back(stemp6);
    path_areas.push_back(temp6);

    ropod_ros_msgs::Area temp7;
    ropod_ros_msgs::SubArea stemp7;
    temp7.id = std::to_string(12);
    temp7.name = "BRSU_C_L0_C2";
    temp7.type = "junction";
    stemp7.name = "BRSU_C_L0_C2_LA1";
    stemp7.id = std::to_string(75);
    temp7.sub_areas.push_back(stemp7);
    path_areas.push_back(temp7);

    ropod_ros_msgs::Area temp8;
    ropod_ros_msgs::SubArea stemp8;    
    temp8.id = std::to_string(13);
    temp8.name = "BRSU_C_L0_C2";
    temp8.type = "corridor";
    stemp8.name = "BRSU_C_L0_C2_LA2";
    stemp8.id = std::to_string(77);
    temp8.sub_areas.push_back(stemp8);
    path_areas.push_back(temp8);

    ropod_ros_msgs::Area temp9;
    ropod_ros_msgs::SubArea stemp9;
    temp9.id = std::to_string(123);
    temp9.name = "";
    temp9.type = "area";
    stemp9.name = "";
    stemp9.id = std::to_string(117);
    temp9.sub_areas.push_back(stemp9);
    path_areas.push_back(temp9);

    ropod_ros_msgs::Area temp10;
    ropod_ros_msgs::SubArea stemp10;
    temp10.id = std::to_string(124);
    temp10.name = "";
    temp10.type = "area";
    stemp10.name = "";
    stemp10.id = std::to_string(118);
    temp10.sub_areas.push_back(stemp10);
    path_areas.push_back(temp10);

    ropod_ros_msgs::Area temp11;
    ropod_ros_msgs::SubArea stemp11;
    temp11.id = std::to_string(126);
    temp11.name = "";
    temp11.type = "area";
    stemp11.name = "";
    stemp11.id = std::to_string(119);
    temp11.sub_areas.push_back(stemp11);
    path_areas.push_back(temp11);

    ropod_ros_msgs::Area temp12;
    ropod_ros_msgs::SubArea stemp12;
    temp12.id = std::to_string(125);
    temp12.name = "";
    temp12.type = "area";
    stemp12.name = "";
    stemp12.id = std::to_string(120);
    temp12.sub_areas.push_back(stemp12);
    path_areas.push_back(temp12);

    
    actionlib::SimpleActionClient<ropod_ros_msgs::RoutePlannerAction> route_planner_action_client("/route_planner",true);

    route_planner_action_client.waitForServer();

    ropod_ros_msgs::RoutePlannerGoal req;
    req.areas = path_areas;
    route_planner_action_client.sendGoal(req);

    //wait for the action to return
    bool finished_before_timeout = route_planner_action_client.waitForResult(ros::Duration(60.0));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = route_planner_action_client.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    return 0;
}
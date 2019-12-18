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
    ros::init(argc, argv, "route_planner_test_hochschulestrasse_return");
    ros::NodeHandle nh("~");

    ROS_INFO("Route planner test hochschulestrasse return");

    std::vector<ropod_ros_msgs::Area>path_areas;
    
    ropod_ros_msgs::Area temp1;
    temp1.id = std::to_string(281);
    temp1.name = "BRSU_A_L0_C8";
    temp1.type = "corridor";
    ropod_ros_msgs::SubArea stemp1;
    stemp1.name = "BRSU_A_L0_C8_LA2";
    stemp1.id = std::to_string(248);
    temp1.sub_areas.push_back(stemp1);
    path_areas.push_back(temp1);

    ropod_ros_msgs::Area temp2;
    temp2.id = std::to_string(273);
    temp2.name = "BRSU_A_L0_C7";
    temp2.type = "junction";
    ropod_ros_msgs::SubArea stemp2;
    stemp2.name = "BRSU_A_L0_C7_LA2";
    stemp2.id = std::to_string(250);
    temp2.sub_areas.push_back(stemp2);
    path_areas.push_back(temp2);

    ropod_ros_msgs::Area temp3;
    ropod_ros_msgs::SubArea stemp3;
    temp3.id = std::to_string(278);
    temp3.name = "BRSU_A_L0_C6";
    temp3.type = "junction";
    stemp3.name = "BRSU_A_L0_C6_LA1";
    stemp3.id = std::to_string(252);
    temp3.sub_areas.push_back(stemp3);
    path_areas.push_back(temp3);    

    ropod_ros_msgs::Area temp4;
    ropod_ros_msgs::SubArea stemp4;
    temp4.id = std::to_string(267);
    temp4.name = "BRSU_A_L0_C4";
    temp4.type = "corridor";
    stemp4.name = "BRSU_A_L0_C4_LA2";
    stemp4.id = std::to_string(257);
    temp4.sub_areas.push_back(stemp4);
    path_areas.push_back(temp4);

    ropod_ros_msgs::Area temp5;
    ropod_ros_msgs::SubArea stemp5;
    temp5.id = std::to_string(268);
    temp5.name = "BRSU_A_L0_C2";
    temp5.type = "corridor";
    stemp5.name = "BRSU_A_L0_C2_LA1";
    stemp5.id = std::to_string(259);
    temp5.sub_areas.push_back(stemp5);
    path_areas.push_back(temp5);

    ropod_ros_msgs::Area temp6;
    ropod_ros_msgs::SubArea stemp6;
    temp6.id = std::to_string(276);
    temp6.name = "BRSU_A_L0_C1";
    temp6.type = "corridor";
    stemp6.name = "BRSU_A_L0_C1_LA2";
    stemp6.id = std::to_string(260);
    temp6.sub_areas.push_back(stemp6);
    path_areas.push_back(temp6);

    ropod_ros_msgs::Area temp7;
    ropod_ros_msgs::SubArea stemp7;
    temp7.id = std::to_string(274);
    temp7.name = "BRSU_A_L0_A12";
    temp7.type = "junction";
    stemp7.name = "BRSU_A_L0_A12_LA1";
    stemp7.id = std::to_string(262);
    temp7.sub_areas.push_back(stemp7);
    path_areas.push_back(temp7);

    ropod_ros_msgs::Area temp8;
    ropod_ros_msgs::SubArea stemp8;    
    temp8.id = std::to_string(272);
    temp8.name = "BRSU_A_L0_A8";
    temp8.type = "area";
    stemp8.name = "BRSU_A_L0_A8_LA1";
    stemp8.id = std::to_string(263);
    temp8.sub_areas.push_back(stemp8);
    path_areas.push_back(temp8);

    ropod_ros_msgs::Area temp9;
    ropod_ros_msgs::SubArea stemp9;
    temp9.id = std::to_string(126);
    temp9.name = "BRSU_A_L0_A2";
    temp9.type = "junction";
    stemp9.name = "BRSU_A_L0_A2_LA1";
    stemp9.id = std::to_string(119);
    temp9.sub_areas.push_back(stemp9);
    path_areas.push_back(temp9);

    ropod_ros_msgs::Area temp10;
    ropod_ros_msgs::SubArea stemp10;
    temp10.id = std::to_string(124);
    temp10.name = "BRSU_A_L0_A3";
    temp10.type = "area";
    stemp10.name = "BRSU_A_L0_A3_LA1";
    stemp10.id = std::to_string(118);
    temp10.sub_areas.push_back(stemp10);
    path_areas.push_back(temp10);

    ropod_ros_msgs::Area temp11;
    ropod_ros_msgs::SubArea stemp11;
    temp11.id = std::to_string(123);
    temp11.name = "BRSU_A_L0_A4";
    temp11.type = "junction";
    stemp11.name = "BRSU_A_L0_A4_LA1";
    stemp11.id = std::to_string(117);
    temp11.sub_areas.push_back(stemp11);
    path_areas.push_back(temp11);

    ropod_ros_msgs::Area temp12;
    ropod_ros_msgs::SubArea stemp12;
    temp12.id = std::to_string(13);
    temp12.name = "BRSU_C_L0_C1";
    temp12.type = "corridor";
    stemp12.name = "BRSU_C_L0_C1_LA2";
    stemp12.id = std::to_string(76);
    temp12.sub_areas.push_back(stemp12);
    path_areas.push_back(temp12);

    ropod_ros_msgs::Area temp13;
    ropod_ros_msgs::SubArea stemp13;
    temp13.id = std::to_string(12);
    temp13.name = "BRSU_C_L0_C2";
    temp13.type = "junction";
    stemp13.name = "BRSU_C_L0_C2_LA1";
    stemp13.id = std::to_string(75);
    temp13.sub_areas.push_back(stemp13);
    path_areas.push_back(temp13);

    ropod_ros_msgs::Area temp14;
    ropod_ros_msgs::SubArea stemp14;
    temp14.id = std::to_string(10);
    temp14.name = "BRSU_C_L0_C2";
    temp14.type = "corridor";
    stemp14.name = "BRSU_C_L0_C2_LA1";
    stemp14.id = std::to_string(71);
    temp14.sub_areas.push_back(stemp14);
    path_areas.push_back(temp14);

    ropod_ros_msgs::Area temp15;
    ropod_ros_msgs::SubArea stemp15;
    temp15.id = std::to_string(9);
    temp15.name = "BRSU_C_L0_C2";
    temp15.type = "corridor";
    stemp15.name = "BRSU_C_L0_C2_LA1";
    stemp15.id = std::to_string(69);
    temp15.sub_areas.push_back(stemp15);
    path_areas.push_back(temp15);

    ropod_ros_msgs::Area temp16;
    ropod_ros_msgs::SubArea stemp16;
    temp16.id = std::to_string(92);
    temp16.name = "BRSU_C_L0_C2";
    temp16.type = "corridor";
    stemp16.name = "BRSU_C_L0_C2_LA1";
    stemp16.id = std::to_string(65);
    temp16.sub_areas.push_back(stemp16);
    path_areas.push_back(temp16);

    ropod_ros_msgs::Area temp17;
    ropod_ros_msgs::SubArea stemp17;
    temp17.id = std::to_string(7);
    temp17.name = "BRSU_C_L0_C2";
    temp17.type = "junction";
    stemp17.name = "BRSU_C_L0_C2_LA1";
    stemp17.id = std::to_string(64);
    temp17.sub_areas.push_back(stemp17);
    path_areas.push_back(temp17);

    ropod_ros_msgs::Area temp18;
    ropod_ros_msgs::SubArea stemp18;
    temp18.id = std::to_string(5);
    temp18.name = "BRSU_C_L0_C2";
    temp18.type = "corridor";
    stemp18.name = "BRSU_C_L0_C2_LA1";
    stemp18.id = std::to_string(60);
    temp18.sub_areas.push_back(stemp18);
    path_areas.push_back(temp18);

    
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
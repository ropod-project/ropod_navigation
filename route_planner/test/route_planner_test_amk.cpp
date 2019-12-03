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
    ros::init(argc, argv, "route_planner_test_amk");
    ros::NodeHandle nh("~");

    ROS_INFO("Route planner test amk");

    std::vector<ropod_ros_msgs::Area>path_areas;


    ropod_ros_msgs::Area temp1;
    temp1.id = std::to_string(118);
    temp1.name = "AMK_D_L-1_C41";
    temp1.type = "corridor";            // TODO: this is junction in actual map, should be corridor
    ropod_ros_msgs::SubArea wtemp1;
    wtemp1.name = "AMK_D_L-1_C41_LA1";
    wtemp1.id = "92";
    temp1.sub_areas.push_back(wtemp1);  
    path_areas.push_back(temp1);

    ropod_ros_msgs::Area temp2;
    temp2.id = std::to_string(119);
    temp2.name = "AMK_D_L-1_C40";
    temp2.type = "corridor"; 
    ropod_ros_msgs::SubArea wtemp2;
    wtemp2.name = "AMK_D_L-1_C40_LA2";
    wtemp2.id = "93";
    temp2.sub_areas.push_back(wtemp2);  
    path_areas.push_back(temp2);

    ropod_ros_msgs::Area temp3;
    temp3.id = std::to_string(120);
    temp3.name = "AMK_D_L-1_C39";
    temp3.type = "junction";
    ropod_ros_msgs::SubArea wtemp3;
    wtemp3.name = "AMK_D_L-1_C39_LA2";
    wtemp3.id = "95";
    temp3.sub_areas.push_back(wtemp3);  
    path_areas.push_back(temp3);

    ropod_ros_msgs::Area temp4;
    temp4.id = std::to_string(121);
    temp4.name = "AMK_C_L-1_C38";
    temp4.type = "corridor";
    ropod_ros_msgs::SubArea wtemp4;
    wtemp4.name = "AMK_C_L-1_C38_LA2";
    wtemp4.id = "96";
    temp4.sub_areas.push_back(wtemp4);  
    path_areas.push_back(temp4);

    ropod_ros_msgs::Area temp5;
    temp5.id = std::to_string(122);
    temp5.name = "AMK_C_L-1_C37";
    temp5.type = "corridor";
    ropod_ros_msgs::SubArea wtemp5;
    wtemp5.name = "AMK_C_L-1_C37_LA2";
    wtemp5.id = "98";
    temp5.sub_areas.push_back(wtemp5);  
    path_areas.push_back(temp5);

    ropod_ros_msgs::Area temp6;
    temp6.id = std::to_string(123);
    temp6.name = "AMK_C_L-1_C36";
    temp6.type = "corridor";
    ropod_ros_msgs::SubArea wtemp6;
    wtemp6.name = "AMK_C_L-1_C36_LA2";
    wtemp6.id = "100";
    temp6.sub_areas.push_back(wtemp6);  
    path_areas.push_back(temp6);

    ropod_ros_msgs::Area temp7;
    temp7.id = std::to_string(124);
    temp7.name = "AMK_C_L-1_C35";
    temp7.type = "corridor";
    ropod_ros_msgs::SubArea wtemp7;
    wtemp7.name = "AMK_C_L-1_C35_LA2";
    wtemp7.id = "102";
    temp7.sub_areas.push_back(wtemp7);  
    path_areas.push_back(temp7);

    ropod_ros_msgs::Area temp8;
    temp8.id = std::to_string(125);
    temp8.name = "AMK_C_L-1_C34";
    temp8.type = "corridor";
    ropod_ros_msgs::SubArea wtemp8;
    wtemp8.name = "AMK_C_L-1_C34_LA2";
    wtemp8.id = "104";
    temp8.sub_areas.push_back(wtemp8);  
    path_areas.push_back(temp8);

    ropod_ros_msgs::Area temp9;
    temp9.id = std::to_string(126);
    temp9.name = "AMK_B_L-1_C33";
    temp9.type = "corridor";
    ropod_ros_msgs::SubArea wtemp9;
    wtemp9.name = "AMK_B_L-1_C33_LA2";
    wtemp9.id = "71";
    temp9.sub_areas.push_back(wtemp9);  
    path_areas.push_back(temp9);

    ropod_ros_msgs::Area temp10;
    temp10.id = std::to_string(127);
    temp10.name = "AMK_B_L-1_C32";
    temp10.type = "corridor";
    ropod_ros_msgs::SubArea wtemp10;
    wtemp10.name = "AMK_B_L-1_C32_LA2";
    wtemp10.id = "73";
    temp10.sub_areas.push_back(wtemp10);  
    path_areas.push_back(temp10);

    ropod_ros_msgs::Area temp11;
    temp11.id = std::to_string(128);
    temp11.name = "AMK_B_L-1_C31";
    temp11.type = "corridor";
    ropod_ros_msgs::SubArea wtemp11;
    wtemp11.name = "AMK_B_L-1_C31_LA2";
    wtemp11.id = "75";
    temp11.sub_areas.push_back(wtemp11);  
    path_areas.push_back(temp11);

    ropod_ros_msgs::Area temp12;
    temp12.id = std::to_string(129);
    temp12.name = "AMK_B_L-1_C30";
    temp12.type = "corridor";
    ropod_ros_msgs::SubArea wtemp12;
    wtemp12.name = "AMK_B_L-1_C30_LA2";
    wtemp12.id = "77";
    temp12.sub_areas.push_back(wtemp12);  
    path_areas.push_back(temp12);

    ropod_ros_msgs::Area temp13;
    temp13.id = std::to_string(130);
    temp13.name = "AMK_B_L-1_C29";
    temp13.type = "corridor";
    ropod_ros_msgs::SubArea wtemp13;
    wtemp13.name = "AMK_B_L-1_C29_LA2";
    wtemp13.id = "82";
    temp13.sub_areas.push_back(wtemp13);  
    path_areas.push_back(temp13);

    // TODO: block connections in areas with localization issue

    ropod_ros_msgs::Area temp14;
    temp14.id = std::to_string(132);
    temp14.name = "AMK_B_L-1_C27";
    temp14.type = "junction";
    ropod_ros_msgs::SubArea wtemp14;
    wtemp14.name = "AMK_B_L-1_C27_LA1";  // TODO: make this 4 point area (Currently 5)
    wtemp14.id = "81";
    temp14.sub_areas.push_back(wtemp14);  
    path_areas.push_back(temp14);

    ropod_ros_msgs::Area temp15;
    temp15.id = std::to_string(134);
    temp15.name = "AMK_B_L-1_C26";
    temp15.type = "corridor";
    ropod_ros_msgs::SubArea wtemp15;
    wtemp15.name = "AMK_B_L-1_C26_LA2";
    wtemp15.id = "84";
    temp15.sub_areas.push_back(wtemp15);  
    path_areas.push_back(temp15);

    ropod_ros_msgs::Area temp16;
    temp16.id = std::to_string(135);
    temp16.name = "AMK_B_L-1_C24";
    temp16.type = "corridor";
    ropod_ros_msgs::SubArea wtemp16;
    wtemp16.name = "AMK_B_L-1_C24_LA2";
    wtemp16.id = "88";
    temp16.sub_areas.push_back(wtemp16);  
    path_areas.push_back(temp16);

    ropod_ros_msgs::Area temp17;
    temp17.id = std::to_string(139);
    temp17.name = "AMK_B_L-1_C15";
    temp17.type = "junction";
    ropod_ros_msgs::SubArea wtemp17;
    wtemp17.name = "AMK_B_L-1_C15_LA1";
    wtemp17.id = "36";
    temp17.sub_areas.push_back(wtemp17);  
    path_areas.push_back(temp17);

    ropod_ros_msgs::Area temp18;
    temp18.id = std::to_string(172);
    temp18.name = "AMK_B_L-1_C16";
    temp18.type = "corridor";
    ropod_ros_msgs::SubArea wtemp18;
    wtemp18.name = "AMK_B_L-1_C16_LA1";
    wtemp18.id = "35";
    temp18.sub_areas.push_back(wtemp18);  
    path_areas.push_back(temp18);

    ropod_ros_msgs::Area temp19;
    temp19.id = std::to_string(137);
    temp19.name = "AMK_B_L-1_C17";
    temp19.type = "corridor";   // TODO: make this corridor in the map
    ropod_ros_msgs::SubArea wtemp19;
    wtemp19.name = "AMK_B_L-1_C17_LA1";   // TODO: make this 4 point area
    wtemp19.id = "33";
    temp19.sub_areas.push_back(wtemp19);  
    path_areas.push_back(temp19);

    ropod_ros_msgs::Area temp20;
    temp20.id = std::to_string(111);
    temp20.name = "AMK_B_L-1_C5";
    temp20.type = "junction";   
    ropod_ros_msgs::SubArea wtemp20;
    wtemp20.name = "AMK_B_L-1_C5_LA1";   // TODO: make this 4 point area
    wtemp20.id = "62";
    temp20.sub_areas.push_back(wtemp20);  
    path_areas.push_back(temp20);

    ropod_ros_msgs::Area temp21;
    temp21.id = std::to_string(112);
    temp21.name = "AMK_B_L-1_C4";
    temp21.type = "corridor";   
    ropod_ros_msgs::SubArea wtemp21;
    wtemp21.name = "AMK_B_L-1_C4_LA1";
    wtemp21.id = "64";
    temp21.sub_areas.push_back(wtemp21);  
    path_areas.push_back(temp21);

    ropod_ros_msgs::Area temp22;
    temp22.id = std::to_string(112);
    temp22.name = "AMK_B_L-1_C4";
    temp22.type = "corridor";   
    ropod_ros_msgs::SubArea wtemp22;
    wtemp22.name = "AMK_B_L-1_C4_LA1";
    wtemp22.id = "64";
    temp22.sub_areas.push_back(wtemp22);  
    path_areas.push_back(temp22);

    ropod_ros_msgs::Area temp23;
    temp23.id = std::to_string(112);
    temp23.name = "AMK_B_L-1_C3";
    temp23.type = "junction"; // TODO: convert from  corridor to junction   
    ropod_ros_msgs::SubArea wtemp23;
    wtemp23.name = "AMK_B_L-1_C3_LA1"; // TODO: delete other sub-area at junction
    wtemp23.id = "66";
    temp23.sub_areas.push_back(wtemp23);  
    path_areas.push_back(temp23);

    ropod_ros_msgs::Area temp24;
    temp24.id = std::to_string(114);
    temp24.name = "AMK_B_L-1_C2";
    temp24.type = "corridor"; 
    ropod_ros_msgs::SubArea wtemp24;
    wtemp24.name = "AMK_B_L-1_C2_LA1"; // TODO: make this 4 point area in the map
    wtemp24.id = "68";
    temp24.sub_areas.push_back(wtemp24);  
    path_areas.push_back(temp24);
    
    
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
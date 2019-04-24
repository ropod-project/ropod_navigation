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
    temp1.id = std::to_string(4991);
    temp1.name = " AMK_A_L-1_C42";
    temp1.type = "corridor";
    // ropod_ros_msgs::SubArea wtemp1;
    // wtemp1.name = " AMK_A_L-1_C42_LA2";
    // wtemp1.id = "4989";
    // temp1.sub_areas.push_back(wtemp1);
    ropod_ros_msgs::SubArea wtemp1_1;
    wtemp1_1.name = " AMK_A_L-1_C42_LA1";
    wtemp1_1.id = "152";
    temp1.sub_areas.push_back(wtemp1_1);    
    path_areas.push_back(temp1);

    ropod_ros_msgs::Area temp3;
    temp3.id = std::to_string(3128);
    temp3.name = "AMK_A_L-1_C12";
    temp3.type = "junction";
    ropod_ros_msgs::SubArea wtemp3;
    wtemp3.name = "AMK_A_L-1_C12_LA1";
    wtemp3.id = "49";
    temp3.sub_areas.push_back(wtemp3);  
    path_areas.push_back(temp3);

    ropod_ros_msgs::Area temp4;
    temp4.id = std::to_string(3130);
    temp4.name = "AMK_B_L-1_C13";
    temp4.type = "corridor";
    ropod_ros_msgs::SubArea wtemp4;
    wtemp4.name = "AMK_B_L-1_C13_LA2";
    wtemp4.id = "39";
    temp4.sub_areas.push_back(wtemp4);  
    path_areas.push_back(temp4);

    ropod_ros_msgs::Area temp2;
    temp2.id = std::to_string(157);
    temp2.name = "AMK_A_L-1_C13_Door1";
    temp2.type = "door";
    path_areas.push_back(temp2);    
    
    ropod_ros_msgs::Area temp5;
    temp5.id = std::to_string(3131);
    temp5.name = "AMK_B_L-1_C14";
    temp5.type = "corridor";
    ropod_ros_msgs::SubArea wtemp5;
    wtemp5.name = "AMK_B_L-1_C14_LA1";
    wtemp5.id = "38";
    temp5.sub_areas.push_back(wtemp5);  
    path_areas.push_back(temp5);
    
    ropod_ros_msgs::Area temp6;
    temp6.id = std::to_string(3133);
    temp6.name = "AMK_B_L-1_C15";
    temp6.type = "junction";
    ropod_ros_msgs::SubArea wtemp6;
    wtemp6.name = "AMK_B_L-1_C15_LA1";
    wtemp6.id = "36";
    temp6.sub_areas.push_back(wtemp6);  
    path_areas.push_back(temp6);    

    ropod_ros_msgs::Area temp7;
    temp7.id = std::to_string(3134);
    temp7.name = "AMK_B_L-1_C16";
    temp7.type = "corridor";
    ropod_ros_msgs::SubArea wtemp7;
    wtemp7.name = "AMK_B_L-1_C16_LA1";
    wtemp7.id = "35";
    temp7.sub_areas.push_back(wtemp7);  
    path_areas.push_back(temp7);    
    
    actionlib::SimpleActionClient<ropod_ros_msgs::RoutePlannerAction> route_planner_action_client("/route_planner",true);

    route_planner_action_client.waitForServer();

    ropod_ros_msgs::RoutePlannerGoal req;
    req.areas = path_areas;
    route_planner_action_client.sendGoal(req);

    if (route_planner_action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Conenction successful");
    }
    else
    {
      ROS_WARN("Conenction failed");
    }


    return 0;
}
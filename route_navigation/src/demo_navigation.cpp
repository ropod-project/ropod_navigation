#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <string>


/* ROPOD ROS messages */
#include <ropod_ros_msgs/sem_waypoint_cmd.h>

/* Simple world model */
#include "waypoint_navigation.h"
#include "elevator_navigation.h"
#include "simplified_world_model.h"
#include <ropod_demo_dec_2017/doorDetection.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

wm::Simplified_WorldModel simple_wm;
Waypoint_navigation waypoint_navigation;
Elevator_navigation elevator_navigation;
ropod_demo_dec_2017::doorDetection doorStatus;

enum {
    NAV_WAYPOINT = 0,
    NAV_ELEVATOR,
    NAV_NONE
};

int active_nav = NAV_NONE;

void move_base_fbCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg)
{
    waypoint_navigation.base_position_ = msg;
    elevator_navigation.base_position_ = msg;
}


void routetopicCallback(const nav_msgs::Path::ConstPtr& Pathmsg)
{




    if(Pathmsg->header.frame_id == "PAUSE") {
        waypoint_navigation.pause_navigation();
        elevator_navigation.pause_navigation();
    } else if(Pathmsg->header.frame_id == "RESUME") {
        waypoint_navigation.resume_navigation();
        elevator_navigation.resume_navigation();
    } else if(Pathmsg->header.frame_id == "TAKE_ELEVATOR") {
        elevator_navigation.start_navigation();
        active_nav = NAV_ELEVATOR;
    } else {
        waypoint_navigation.start_navigation(*Pathmsg);
        active_nav = NAV_WAYPOINT;
    }


}

void CCUcommandCallback(const ropod_ros_msgs::sem_waypoint_cmd::ConstPtr& Cmdmsg)
{

    ROS_INFO("CCU Command received. Behavior %s, %d primitives", Cmdmsg->primitive[0].behaviour.c_str(), (int) Cmdmsg->primitive.size());

    nav_msgs::Path Pathmsg;


    // For now only one primitive is processed. This will be changed once the messages are fixed
    Pathmsg.poses = Cmdmsg->primitive[0].poses;

    if(Cmdmsg->primitive[0].behaviour == "PAUSE") {
        waypoint_navigation.pause_navigation();
        elevator_navigation.pause_navigation();
    } else if(Cmdmsg->primitive[0].behaviour == "RESUME") {
        waypoint_navigation.resume_navigation();
        elevator_navigation.resume_navigation();
    } else if(Cmdmsg->primitive[0].behaviour == "TAKE_ELEVATOR") {
        elevator_navigation.start_navigation();
        active_nav = NAV_ELEVATOR;
    } else if(Cmdmsg->primitive[0].behaviour == "GOTO") {
        waypoint_navigation.start_navigation(Pathmsg);
        active_nav = NAV_WAYPOINT;
    }


}
void doorDetectCallback(const ropod_demo_dec_2017::doorDetection::ConstPtr& DoorStmsg)
{
    doorStatus = *DoorStmsg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "route_navigation");
    ros::NodeHandle n;
    ros::Rate rate(5.0);

    ros::Subscriber sub 		= 	n.subscribe<nav_msgs::Path>("/planned_route", 10, routetopicCallback);
    ros::Subscriber submvfb = 	n.subscribe<move_base_msgs::MoveBaseActionFeedback>("/move_base/feedback", 10, move_base_fbCallback);
    ros::Subscriber subCCUCommands = n.subscribe<ropod_ros_msgs::sem_waypoint_cmd>("waypoint_cmd", 10,CCUcommandCallback);

    ros::Subscriber subdoorStatus = n.subscribe<ropod_demo_dec_2017::doorDetection>("door", 10, doorDetectCallback);
    doorStatus.closed = false;
    doorStatus.open = false;
    doorStatus.undetectable = true;

    ros::Publisher movbase_cancel_pub = n.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);




    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
    
    // wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    bool sendgoal = false;
    // Wait fo route to be published
    ROS_INFO("Wait for route");
    while(n.ok()) {

        // Select teh corresponding navigation
        switch(active_nav) {
	  
        case NAV_WAYPOINT:
            waypoint_navigation.navigation_state_machine(movbase_cancel_pub, &goal, sendgoal);
            break;
	    
        case NAV_ELEVATOR:
            elevator_navigation.navigation_state_machine(simple_wm.elevator1,movbase_cancel_pub, &goal, sendgoal, doorStatus);
            break;
	    
        default:
            break;
        }

        if (sendgoal)
            ac.sendGoal(goal);


        ros::spinOnce();
        rate.sleep();
    }



    return 0;
}
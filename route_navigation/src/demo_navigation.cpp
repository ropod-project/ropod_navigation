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
#include <ropod_ros_msgs/ropod_demo_plan.h>
#include <ropod_ros_msgs/ropod_demo_location.h>
#include <ropod_ros_msgs/ropod_demo_area.h>
#include <ropod_ros_msgs/ropod_demo_waypoint.h>
#include <ropod_ros_msgs/ropod_demo_status.h>
#include <ropod_ros_msgs/ropod_demo_status_update.h>
#include <ropod_ros_msgs/ropod_door_detection.h>

#include "waypoint_navigation.h"
#include "elevator_navigation.h"
#include "route_navigation_defines.h"
/* Simple world model */
#include "simplified_world_model.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

wm::SimplifiedWorldModel simple_wm;
WaypointNavigation waypoint_navigation;
ElevatorNavigation elevator_navigation;
ropod_ros_msgs::ropod_door_detection door_status;
ropod_ros_msgs::ropod_demo_status_update ropod_fb_msg;

enum {
    NAVTYPE_WAYPOINT = 0,
    NAVTYPE_ELEVATOR,
    NAVTYPE_NONE
};

int active_nav = NAVTYPE_NONE;
bool path_msg_received = false;
bool prepare_next_loc = false;
ropod_ros_msgs::ropod_demo_plan plan_msg;
ropod_ros_msgs::ropod_demo_plan plan_msg_rec;

void move_base_fbCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg)
{
    waypoint_navigation.base_position = msg;
    elevator_navigation.base_position = msg;
}

void CCUPathCommandCallback(const ropod_ros_msgs::ropod_demo_plan::ConstPtr& plan_msg_)
{
    ROS_INFO("CCU Command received. planID %s, %d locations", plan_msg_->planID.c_str(), (int) plan_msg_->locations.size());
    plan_msg_rec = *plan_msg_;
    path_msg_received = true;
}

void doorDetectCallback(const ropod_ros_msgs::ropod_door_detection::ConstPtr& DoorStmsg)
{
    door_status = *DoorStmsg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "route_navigation");
    ros::NodeHandle n("~");
    ros::Rate rate(5.0);

    std::string moveBaseServerName;
    std::string moveBaseFeedbackTopic;
    std::string moveBaseCancelTopic;

    n.param<std::string>("move_base_server", moveBaseServerName, "/move_base");
    n.param<std::string>("move_base_feedback_topic", moveBaseFeedbackTopic, "/move_base/feedback");
    n.param<std::string>("move_base_cancel_topic", moveBaseCancelTopic, "/move_base/cancel");

    ros::Subscriber sub_movebase_fb =   n.subscribe<move_base_msgs::MoveBaseActionFeedback>(moveBaseFeedbackTopic, 10, move_base_fbCallback);
    ros::Subscriber sub_ccu_commands = n.subscribe<ropod_ros_msgs::ropod_demo_plan>("/ropod_demo_plan", 10, CCUPathCommandCallback);
    ros::Subscriber subdoor_status = n.subscribe<ropod_ros_msgs::ropod_door_detection>("/door", 10, doorDetectCallback);

    door_status.closed = false;
    door_status.open = false;
    door_status.undetectable = true;

    ros::Publisher movbase_cancel_pub = n.advertise<actionlib_msgs::GoalID>(moveBaseCancelTopic, 1);
    ros::Publisher ropod_task_fb_pub = n.advertise<ropod_ros_msgs::ropod_demo_status_update>("/ropod_task_feedback", 1);

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac(moveBaseServerName, true);

    nav_msgs::Path path_msg;
    std::vector<std::basic_string< char > > waypoint_ids;

    // wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    bool send_goal = false;
    // Wait fo route to be published
    ROS_INFO("Wait for route");
    while(n.ok())
    {
        int curr_loc;
        std::vector<ropod_ros_msgs::ropod_demo_area>::const_iterator curr_area;
        std::vector<ropod_ros_msgs::ropod_demo_waypoint>::const_iterator curr_wp;
        int it_idwp;

        // Process the received Plan message. Point to first location
        if(path_msg_received) { // checks need to be done if a current navigation is taking place

            path_msg_received = false;

            // If first location is a PAUSE OR RESUME, then do not modify existing path
            if (plan_msg_rec.locations[0].command == "PAUSE")
            {
                waypoint_navigation.pauseNavigation();
                elevator_navigation.pauseNavigation();
            }
            else if (plan_msg_rec.locations[0].command == "RESUME")
            {
                if(active_nav ==  NAVTYPE_NONE)
                {   // in case the pause is received in between locations, force to go to next location
                    prepare_next_loc = true;
                }
                else
                {
                    waypoint_navigation.resumeNavigation();
                    elevator_navigation.resumeNavigation();
                }
            }
            else
            {
                curr_loc = 0;
                plan_msg = plan_msg_rec;
                prepare_next_loc = true;
            }

        }

        // Process locations, read areas and stack the waypoins

        if(prepare_next_loc)
        {
            // Clear previous waypoints and locations
            waypoint_ids.clear();
            path_msg.poses.clear();
            // Load waypoints of new location
            if(curr_loc < plan_msg.locations.size())
            {
                // Extract all areas and waypoints to go the corresponding location
                for(std::vector<ropod_ros_msgs::ropod_demo_area>::const_iterator curr_area = plan_msg.locations[curr_loc].areas.begin(); curr_area != plan_msg.locations[curr_loc].areas.end(); ++curr_area)
                {
                    for(std::vector<ropod_ros_msgs::ropod_demo_waypoint>::const_iterator curr_wp = curr_area->waypoints.begin(); curr_wp != curr_area->waypoints.end(); ++curr_wp)
                    {
                        geometry_msgs::PoseStamped pose_stamped_temp;
                        pose_stamped_temp.pose = curr_wp->waypointPosition;
                        path_msg.poses.push_back(pose_stamped_temp);
                        waypoint_ids.push_back(curr_wp->id.c_str());
                    }
                }


                // Select the corresponding navigation routine depending con the command
                if (plan_msg.locations[curr_loc].command == "PAUSE")
                {
                    // Notice that a RESUME in a sequence is not valid, once a pause is received, the path is resumed only if received separately from the CCU
                    waypoint_navigation.pauseNavigation();
                    elevator_navigation.pauseNavigation();
                }
                else if(plan_msg.locations[curr_loc].command == "GOTO")
                {
                    waypoint_navigation.startNavigation(path_msg);
                    active_nav = NAVTYPE_WAYPOINT;
                }
                else if (plan_msg.locations[curr_loc].command == "TAKE_ELEVATOR")
                {
//            elevator_navigation.startNavigation(simple_wm.elevator1,path_msg);
                    active_nav = NAVTYPE_ELEVATOR;
                }

            }

            curr_loc++;
            prepare_next_loc = false;

        }


        TaskFeedbackCcu nav_state;

        // Select the corresponding navigation
        switch(active_nav)
        {

        case NAVTYPE_WAYPOINT:
            // ROS_INFO("NAV_WAYPOINT");
            nav_state = waypoint_navigation.callNavigationStateMachine(movbase_cancel_pub, &goal, send_goal);
            break;

        case NAVTYPE_ELEVATOR:
            // ROS_INFO("NAV_ELEVATOR");
            nav_state = elevator_navigation.callNavigationStateMachine(movbase_cancel_pub, &goal, send_goal, simple_wm.elevator1, door_status);
            if(nav_state.fb_nav == NAV_DONE)
            {
                active_nav = NAVTYPE_NONE;
                prepare_next_loc = true;
            }
            break;

        default:
            nav_state.fb_nav = NAV_IDLE;
            break;
        }

        // Send feedback
        // TODO: how to properly give feedback when taking elevator since waypoints are not fixed? e.g. multiple possible waiting areas outside elevator
        if(nav_state.fb_nav == NAV_DONE)
        {
            ROS_INFO("NAV_ELEVATOR DONE!");
            active_nav = NAVTYPE_NONE;
            prepare_next_loc = true;
        }
        else if(nav_state.fb_nav == NAV_WAYPOINT_DONE)
        {
            ROS_INFO("Waypoint done notification received");
            if(nav_state.wayp_n<=waypoint_ids.size())
                ropod_fb_msg.id = waypoint_ids[nav_state.wayp_n-1];
            ropod_fb_msg.status.status = "reached";
            ropod_fb_msg.status.sequenceNumber = nav_state.wayp_n;
            ropod_fb_msg.status.totalNumber = waypoint_ids.size();
            ropod_task_fb_pub.publish(ropod_fb_msg);
            // Update coming waypoint
        }
        else if(nav_state.fb_nav == NAV_GOTOPOINT)
        {
            if(nav_state.wayp_n<=waypoint_ids.size())
                ropod_fb_msg.id = waypoint_ids[nav_state.wayp_n-1];
            ropod_fb_msg.status.status = "reaching";
            ropod_fb_msg.status.sequenceNumber = nav_state.wayp_n;
            ropod_fb_msg.status.totalNumber = waypoint_ids.size();
            ropod_task_fb_pub.publish(ropod_fb_msg);
        }

        // Send navigation command
        if (send_goal)
            ac.sendGoal(goal);


        ros::spinOnce();
        rate.sleep();
    }



    return 0;
}

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <string>

/* ROPOD ROS messages */
#include <ropod_ros_msgs/ropod_door_detection.h>

#include <ropod_ros_msgs/Action.h>
#include <ropod_ros_msgs/TaskProgressGOTO.h>
#include <ropod_ros_msgs/route_planner.h>


#include "waypoint_navigation.h"
#include "elevator_navigation.h"
#include "route_navigation_defines.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

wm::SimplifiedWorldModel simple_wm;
WaypointNavigation waypoint_navigation;
ElevatorNavigation elevator_navigation;
ropod_ros_msgs::ropod_door_detection door_status;
ropod_ros_msgs::TaskProgressGOTO ropod_progress_msg;


enum {
    NAVTYPE_WAYPOINT = 0,
    NAVTYPE_ELEVATOR,
    NAVTYPE_NONE
};

int active_nav = NAVTYPE_NONE;
bool action_msg_received = false;
bool prepare_next_loc = false;
ropod_ros_msgs::Action action_msg;
ropod_ros_msgs::Action action_msg_rec;

void move_base_fbCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg)
{
    waypoint_navigation.base_position = msg;
    elevator_navigation.base_position = msg;
}

void actionCallback(const ropod_ros_msgs::Action::ConstPtr& action_msg_)
{
    ROS_INFO("Action message received. Action ID %s, %d areas", action_msg_->action_id.c_str(), (int) action_msg_->areas.size());
    action_msg_rec = *action_msg_;
    action_msg_received = true;
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
    ros::Subscriber sub_ccu_commands = n.subscribe<ropod_ros_msgs::Action>("goto_action", 10, actionCallback);
    ros::Subscriber subdoor_status = n.subscribe<ropod_ros_msgs::ropod_door_detection>("/door", 10, doorDetectCallback);

    door_status.closed = false;
    door_status.open = false;
    door_status.undetectable = true;

    ros::Publisher movbase_cancel_pub = n.advertise<actionlib_msgs::GoalID>(moveBaseCancelTopic, 1);
    ros::Publisher ropod_task_fb_pub = n.advertise<ropod_ros_msgs::TaskProgressGOTO>("progress", 1);

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
        std::vector<ropod_ros_msgs::Area>::const_iterator curr_area;
        std::vector<ropod_ros_msgs::Waypoint>::const_iterator curr_wp;
        int it_idwp;

        // Process the received Action message. Point to first location
        if(action_msg_received) { // checks need to be done if a current navigation is taking place

            action_msg_received = false;
            curr_loc = 0;
            action_msg = action_msg_rec;
            waypoint_ids.clear();
            path_msg.poses.clear();

            if (action_msg.type == "GOTO")
            {
                ropod_ros_msgs::route_planner route_planner_msg;
                route_planner_msg.request.areas = action_msg.areas;

                ROS_INFO("Now requesting route from route planner");                
                ros::ServiceClient route_planner_client = n.serviceClient<ropod_ros_msgs::route_planner>("/route_planner/route_planner_service");
                if (route_planner_client.call(route_planner_msg))
                {
                    ROS_INFO("Route received from route planner");
                }
                else
                {
                    ROS_ERROR("Failed to call service route_planner_service");
                }

                // Extract all waypoints to go the corresponding location
                for(std::vector<ropod_ros_msgs::Area>::const_iterator curr_area = route_planner_msg.response.areas.begin();
                        curr_area != route_planner_msg.response.areas.end(); ++curr_area)
                {
                    for (std::vector<ropod_ros_msgs::Waypoint>::const_iterator curr_way = curr_area->waypoints.begin(); 
                        curr_way != curr_area->waypoints.end(); ++curr_way)
                    {
                        geometry_msgs::PoseStamped p; 
                        p.pose = curr_way->waypoint_pose;
                        std::string area_type = curr_way->semantic_id;
                        path_msg.poses.push_back(p);
                        waypoint_ids.push_back(area_type);
                    }
                }
                waypoint_navigation.startNavigation(path_msg);
                active_nav = NAVTYPE_WAYPOINT;
            }
            else if (action_msg.type == "ENTER_ELEVATOR")
            {
                // here we need to extract the two navigation poses to enter elevator
                // and pass it to elevator_navigation
                // geometry_msgs::PoseStamped p = getElevatorPoseFromWorldModel(floor id?)
//            elevator_navigation.startNavigation(simple_wm.elevator1,path_msg);
                active_nav = NAVTYPE_ELEVATOR;
            }
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
                ropod_progress_msg.area_name = waypoint_ids[nav_state.wayp_n-1];
            ropod_progress_msg.action_id = action_msg.action_id;
            ropod_progress_msg.action_type = action_msg.type;
            ropod_progress_msg.status = "reached";
            ropod_progress_msg.sequenceNumber = nav_state.wayp_n;
            ropod_progress_msg.totalNumber = waypoint_ids.size();
            ropod_task_fb_pub.publish(ropod_progress_msg);
            // Update coming waypoint
        }
        else if(nav_state.fb_nav == NAV_GOTOPOINT)
        {
            if(nav_state.wayp_n<=waypoint_ids.size())
                ropod_progress_msg.area_name = waypoint_ids[nav_state.wayp_n-1];
            ropod_progress_msg.action_id = action_msg.action_id;
            ropod_progress_msg.action_type = action_msg.type;
            ropod_progress_msg.status = "reaching";
            ropod_progress_msg.sequenceNumber = nav_state.wayp_n;
            ropod_progress_msg.totalNumber = waypoint_ids.size();
            ropod_task_fb_pub.publish(ropod_progress_msg);
        }

        // Send navigation command
        if (send_goal)
            ac.sendGoal(goal);


        ros::spinOnce();
        rate.sleep();
    }



    return 0;
}

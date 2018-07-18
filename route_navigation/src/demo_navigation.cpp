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
#include <ropod_ros_msgs/sem_waypoint_cmd.h>
#include <ropod_ros_msgs/ropod_demo_plan.h>
#include <ropod_ros_msgs/ropod_demo_location.h>
#include <ropod_ros_msgs/ropod_demo_area.h>
#include <ropod_ros_msgs/ropod_demo_waypoint.h>
#include <ropod_ros_msgs/ropod_demo_status.h>
#include <ropod_ros_msgs/ropod_demo_status_update.h>
#include <ropod_ros_msgs/ropod_door_detection.h>

#include <ropod_ros_msgs/Action.h>
#include <ropod_ros_msgs/TaskProgressGOTO.h>

#include "waypoint_navigation.h"
#include "elevator_navigation.h"
#include "route_navigation_defines.h"
/* Simple world model */
#include "simplified_world_model.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

wm::Simplified_WorldModel simple_wm;
Waypoint_navigation waypoint_navigation;
Elevator_navigation elevator_navigation;
ropod_ros_msgs::ropod_door_detection doorStatus;
ropod_ros_msgs::ropod_demo_status_update ropod_fb_msg;
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
    waypoint_navigation.base_position_ = msg;
    elevator_navigation.base_position_ = msg;
}

void CCUPathCommandCallback(const ropod_ros_msgs::Action::ConstPtr &action_msg)
{
    ROS_INFO("go_to command received. Action ID: %s, %d areas: ", action_msg->action_id.c_str(), (int) action_msg->areas.size());
    action_msg_rec = *action_msg;
    action_msg_received = true;
}

void doorDetectCallback(const ropod_ros_msgs::ropod_door_detection::ConstPtr& DoorStmsg)
{
    doorStatus = *DoorStmsg;
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

    ros::Subscriber submvfb = 	n.subscribe<move_base_msgs::MoveBaseActionFeedback>(moveBaseFeedbackTopic, 10, move_base_fbCallback);
    ros::Subscriber subCCUCommands = n.subscribe<ropod_ros_msgs::Action>("go_to_action", 10, CCUPathCommandCallback);
    ros::Subscriber subdoorStatus = n.subscribe<ropod_ros_msgs::ropod_door_detection>("/door", 10, doorDetectCallback);

    doorStatus.closed = false;
    doorStatus.open = false;
    doorStatus.undetectable = true;

    ros::Publisher movbase_cancel_pub = n.advertise<actionlib_msgs::GoalID>(moveBaseCancelTopic, 1);
    ros::Publisher ropod_task_fb_pub = n.advertise<ropod_ros_msgs::TaskProgressGOTO>("progress", 1);

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac(moveBaseServerName, true);

    nav_msgs::Path Pathmsg;
    std::vector<std::basic_string< char > > Wayp_ids;

    // wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    bool sendgoal = false;
    // Wait fo route to be published
    ROS_INFO("Wait for route");
    while(n.ok()) {
      int curr_loc;
      std::vector<ropod_ros_msgs::Area>::const_iterator curr_area;
      std::vector<ropod_ros_msgs::Waypoint>::const_iterator curr_wp;
      int it_idwp;

      // Process the received Plan message. Point to first location
      if(action_msg_received){ // checks need to be done if a current navigation is taking place

            action_msg_received = false;
            curr_loc = 0;
            action_msg = action_msg_rec;


          // Process locations, read areas and stack the waypoins

            // Clear previous waypoints and locations
            Wayp_ids.clear();
            Pathmsg.poses.clear();
            // Load waypoints of new location

            // Extract all areas and waypoints to go the corresponding location
            for(std::vector<ropod_ros_msgs::Area>::const_iterator curr_area = action_msg.areas.begin(); curr_area != action_msg.areas.end(); ++curr_area)
            {
                std::string name = curr_area->name;
                std::string param_name = "/areas/" + curr_area->name;
                std::vector<double> waypoint;
                ros::param::get(param_name, waypoint);
                geometry_msgs::PoseStamped p;
                p.pose.position.x = waypoint[0];
                p.pose.position.y = waypoint[1];
                tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, waypoint[2]);
                q.normalize();
                p.pose.orientation.x = q.x();
                p.pose.orientation.y = q.y();
                p.pose.orientation.z = q.z();
                p.pose.orientation.w = q.w();
                Pathmsg.poses.push_back(p);
                Wayp_ids.push_back(name);
            }

            /*

          // Select the corresponding navigation routine depending con the command
          if (Planmsg.locations[curr_loc].command == "PAUSE") {
            // Notice that a RESUME in a sequence is not valid, once a pause is received, the path is resumed only if received separately from the CCU
            waypoint_navigation.pause_navigation();
            elevator_navigation.pause_navigation();
          } else if(Planmsg.locations[curr_loc].command == "GOTO") {
              waypoint_navigation.start_navigation(Pathmsg);
              active_nav = NAVTYPE_WAYPOINT;
          } else if (Planmsg.locations[curr_loc].command == "TAKE_ELEVATOR") {
              elevator_navigation.start_navigation(simple_wm.elevator1,Pathmsg);
              active_nav = NAVTYPE_ELEVATOR;
          }
          */


          waypoint_navigation.start_navigation(Pathmsg);
          active_nav = NAVTYPE_WAYPOINT;
      }

      task_fb_ccu nav_state;

      // Select the corresponding navigation
      switch(active_nav) {

	case NAVTYPE_WAYPOINT:
	  // ROS_INFO("NAV_WAYPOINT");
	  nav_state = waypoint_navigation.navigation_state_machine(movbase_cancel_pub, &goal, sendgoal);
	  break;

	case NAVTYPE_ELEVATOR:
	  // ROS_INFO("NAV_ELEVATOR");
	  nav_state = elevator_navigation.navigation_state_machine(movbase_cancel_pub, &goal, sendgoal, simple_wm.elevator1, doorStatus);
	  if(nav_state.fb_nav == NAV_DONE){

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
      if(nav_state.fb_nav == NAV_DONE){
        ROS_INFO("NAV_ELEVATOR DONE!");
        active_nav = NAVTYPE_NONE;
        prepare_next_loc = true;
      }else if(nav_state.fb_nav == NAV_WAYPOINT_DONE){
        ROS_INFO("Waypoint done notification received");
        if(nav_state.wayp_n<=Wayp_ids.size())
          ropod_progress_msg.area_name = Wayp_ids[nav_state.wayp_n-1];
        ropod_progress_msg.action_id = action_msg.action_id;
        ropod_progress_msg.action_type = action_msg.type;
        ropod_progress_msg.status = "reached";
        ropod_progress_msg.sequenceNumber = nav_state.wayp_n;
        ropod_progress_msg.totalNumber = Wayp_ids.size();
        ropod_task_fb_pub.publish(ropod_progress_msg);
        // Update coming waypoint
      }else if(nav_state.fb_nav == NAV_GOTOPOINT){
          if(nav_state.wayp_n<=Wayp_ids.size())
            ropod_progress_msg.area_name = Wayp_ids[nav_state.wayp_n-1];
          ropod_progress_msg.action_id = action_msg.action_id;
          ropod_progress_msg.action_type = action_msg.type;
          ropod_progress_msg.status = "reaching";
          ropod_progress_msg.sequenceNumber = nav_state.wayp_n;
          ropod_progress_msg.totalNumber = Wayp_ids.size();
          ropod_task_fb_pub.publish(ropod_progress_msg);
      }

      // Send navigation command
      if (sendgoal)
	  ac.sendGoal(goal);


      ros::spinOnce();
      rate.sleep();
  }



  return 0;
}

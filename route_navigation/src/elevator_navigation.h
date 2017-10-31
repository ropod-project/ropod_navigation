#ifndef ELEVATOR_NAV_HH
#define ELEVATOR_NAV_HH

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <string>

#include "simplified_world_model.h"
#include <ropod_demo_dec_2017/doorDetection.h>

#define WAYP_REACHED_DIST 0.5
#define GOAL_REACHED_DIST 0.2
#define GOAL_REACHED_ANG  20.0*3.141592/180.0


class Elevator_navigation
{

    enum { NAV_HOLD = 0,
           NAV_IDLE,
           NAV_PAUSED,
           NAV_BUSY,
           NAV_CHECKDOOR_IN,
           NAV_CHECKDOOR_OUT,
           NAV_WAIT_FLOOR_CHANGE,
           NAV_GOTOPOINT,
           NAV_WAYPOINT_DONE,
           NAV_DONE
         };

public:

    nav_msgs::Path planned_route;
    int planned_route_size;
    bool route_busy;
    bool nav_paused_req;
    int  waypoint_cnt;
    int nav_state;
    int nav_next_state;
    int nav_state_bpause;
    int nav_next_state_wp;
    move_base_msgs::MoveBaseActionFeedback::ConstPtr base_position_;
    tf::Transform base_position_tf_;
    tf::Transform waypoint_tf_;
    actionlib_msgs::GoalID emptyGoalID;
    move_base_msgs::MoveBaseGoal goal;
    ros::Time stamp_start;
    ros::Duration stamp_wait;


    Elevator_navigation();
    ~Elevator_navigation();

    void start_navigation();
    void pause_navigation();
    void resume_navigation();
    void reset_navigation();
    void stop_navigation();
    bool is_position_valid();
    bool is_waypoint_achieved();
    bool check_door(ropod_demo_dec_2017::doorDetection doorStatus);
    void navigation_state_machine(wm::Elevator elevator,ros::Publisher movbase_cancel_pub, move_base_msgs::MoveBaseGoal* goal_ptr, bool& sendgoal, ropod_demo_dec_2017::doorDetection doorStatus);
};



#endif
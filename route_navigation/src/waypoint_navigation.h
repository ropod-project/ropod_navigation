#ifndef ROUTE_WAYP_NAV_HH
#define ROUTE_WAYP_NAV_HH

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <string>

#include <ropod_ros_msgs/ropod_demo_status_update.h>

#include "route_navigation_defines.h"

#define WAYP_REACHED_DIST 0.5
#define GOAL_REACHED_DIST 0.2
#define GOAL_REACHED_ANG  20.0*3.141592/180.0

class WaypointNavigation {

    enum { WAYP_NAV_HOLD = 0,
           WAYP_NAV_IDLE,
           WAYP_NAV_PAUSED,
           WAYP_NAV_BUSY,
           WAYP_NAV_GETPOINT,
           WAYP_NAV_GOTOPOINT,
           WAYP_NAV_WAYPOINT_DONE,
           WAYP_NAV_DONE
         };

public:
    nav_msgs::Path planned_route;
    int planned_route_size;
    bool route_busy;
    bool nav_paused_req;
    int waypoint_cnt;
    int nav_state;
    int nav_next_state;
    int nav_state_bpause;
    move_base_msgs::MoveBaseActionFeedback::ConstPtr base_position;
    tf::Transform base_positiontf_;
    tf::Transform waypoint_tf_;
    actionlib_msgs::GoalID emptyGoalID;
    move_base_msgs::MoveBaseGoal goal;
    
    ropod_ros_msgs::ropod_demo_status_update ropod_fb_msg;

    WaypointNavigation();
    ~WaypointNavigation();

    void startNavigation(nav_msgs::Path Pathmsg);
    void pauseNavigation();
    void resumeNavigation();
    void resetNavigation();
    void stopNavigation();
    bool isPositionValid();
    bool isWaypointAchieved();
    bool isLastWaypoint();

    geometry_msgs::Pose getNextWaypoint(void);

    TaskFeedbackCcu callNavigationStateMachine(ros::Publisher &movbase_cancel_pub, move_base_msgs::MoveBaseGoal* goal_ptr, bool& sendgoal);
private:
};

#endif

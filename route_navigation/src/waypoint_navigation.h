#ifndef ROUTE_WAYP_NAV_HH
#define ROUTE_WAYP_NAV_HH

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <string>

#include <ropod_ros_msgs/NavigationArea.h>
#include <ropod_ros_msgs/Waypoint.h>
#include <ropod_ros_msgs/DoorDetection.h>

#include <maneuver_navigation/Goal.h>
#include <maneuver_navigation/Configuration.h>
#include <maneuver_navigation/Feedback.h>

#include "route_navigation_defines.h"

#define WAYP_REACHED_DIST 0.5
#define GOAL_REACHED_DIST 0.3
#define GOAL_REACHED_ANG  20.0*3.141592/180.0

class WaypointNavigation
{

    enum
    {
        WAYP_NAV_HOLD = 0,
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
    std::vector<ropod_ros_msgs::NavigationArea> planned_full_route;
    std::vector<ropod_ros_msgs::NavigationArea>::const_iterator curr_nav_area;
    std::vector<ropod_ros_msgs::Waypoint>::const_iterator curr_nav_waypoint;
    std::vector<ropod_ros_msgs::Waypoint>::const_iterator prev_nav_waypoint;
    maneuver_navigation::Configuration nav_configuration;
    bool change_of_area;
    bool last_area_loaded;
    bool last_waypoint_loaded;
    bool perform_initial_rotation;
    bool route_busy;
    bool nav_paused_req;
    int waypoint_cnt;
    int nav_state;
    int nav_next_state;
    int nav_state_bpause;
    geometry_msgs::PoseStamped::ConstPtr base_position;
    tf::Transform base_positiontf_;
    tf::Transform waypoint_tf_;
    std_msgs::Bool true_bool_msg_;


    WaypointNavigation();
    ~WaypointNavigation();
    void startNavigation(std::vector<ropod_ros_msgs::NavigationArea>& navigation_areas_plan_msg);
    void pauseNavigation();
    void resumeNavigation();
    void resetNavigation();
    void stopNavigation();
    bool isPositionValid();
    bool isWaypointAchieved(const geometry_msgs::PoseStamped &goal);
    bool isLastWaypoint();

    bool getNextWaypoint(maneuver_navigation::Goal &mn_goal);

    TaskFeedbackCcu callNavigationStateMachine(ros::Publisher &nav_cancel_pub, maneuver_navigation::Goal &mn_goal, maneuver_navigation::Feedback &mn_feedback, bool& sendgoal);
private:
};

#endif

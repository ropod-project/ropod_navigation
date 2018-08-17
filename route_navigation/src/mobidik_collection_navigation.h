#ifndef MOBIDIK_COLLECTION_NAV_HH
#define MOBIDIK_COLLECTION_NAV_HH

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <string>

#include <ed/world_model.h>
#include "simplified_world_model.h"
#include <ropod_ros_msgs/ropod_door_detection.h>
#include <ropod_ros_msgs/ropod_demo_status_update.h>

#define WAYP_REACHED_DIST 0.5
#define GOAL_REACHED_DIST 0.2
#define GOAL_REACHED_ANG  20.0*3.141592/180.0

#define WAYP_MOBID_COLL_REACHED_DIST 0.1
#define GOAL_MOBID_COLL_REACHED_DIST 0.1
#define GOAL_MOBID_REACHED_ANG  5.0*3.141592/180.0

class MobidikCollection
{

    enum { MOBID_COLL_NAV_HOLD = 0,
           MOBID_COLL_NAV_IDLE,
           MOBID_COLL_NAV_PAUSED,
           MOBID_COLL_NAV_BUSY,
           MOBID_COLL_FIND_MOBIDIK,
           MOBID_COLL_ROTATE,
           MOBID_COLL_NAV_GOTOPOINT,
           MOBID_COLL_NAV_WAYPOINT_DONE,
           MOBID_COLL_NAV_CONNECTING,
           MOBID_COLL_NAV_COUPLING,
           MOBID_COLL_NAV_DONE           
         };

    MobidikCollection();
    ~MobidikCollection();

    queryMobidik();
    setMobidikPosition();
    getSetpointInFrontOfMobidik();
//     void startNavigation(wm::Elevator &elevator,nav_msgs::Path Pathmsg);
    void pauseNavigation();
    void resumeNavigation();
    void resetNavigation();
    void stopNavigation();
    bool isPositionValid();
    bool isWaypointAchieved();
    TaskFeedbackCcu callNavigationStateMachine(ros::Publisher &movbase_cancel_pub, move_base_msgs::MoveBaseGoal* goal_ptr, bool& sendgoal );
    
    private:

    nav_msgs::Path planned_route_;
    int planned_route_size_;
    bool route_busy_;
    bool nav_paused_req_;
    int  waypoint_cnt_;
    int nav_state_;
    int nav_next_state_;
    int nav_state_bpause_;
    int nav_next_state_wp_;
    move_base_msgs::MoveBaseActionFeedback::ConstPtr base_position_;
    tf::Transform base_positiontf_;
    tf::Transform waypoint_tf_;
    actionlib_msgs::GoalID emptyGoalID_;
    move_base_msgs::MoveBaseGoal goal_;
    ros::Time stamp_start_;
    ros::Duration stamp_wait_;

    ropod_ros_msgs::ropod_demo_status_update ropod_fb_msg_;
};



#endif

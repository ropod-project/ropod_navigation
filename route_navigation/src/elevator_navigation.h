#ifndef ELEVATOR_ELEV_NAV_HH
#define ELEVATOR_ELEV_NAV_HH

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <string>

#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/entity.h>

#include <geolib/ros/tf_conversions.h>
#include <geolib/Shape.h>

#include <angles/angles.h>

#include "simplified_world_model.h"
#include <ropod_ros_msgs/DoorDetection.h>

#include <ropod_ros_msgs/ropod_demo_status_update.h>

#define WAYP_REACHED_DIST 0.5
#define GOAL_REACHED_DIST 0.3
#define GOAL_REACHED_ANG  20.0*3.141592/180.0


class ElevatorNavigation
{

    enum { ELEV_NAV_HOLD = 0,
           ELEV_NAV_IDLE,
           ELEV_NAV_PAUSED,
           ELEV_NAV_BUSY,
           ELEV_NAV_CHECKDOOR_IN,
           ELEV_NAV_CHECKDOOR_OUT,
	   ELEV_NAV_TURN_FACING_EXIT,
           ELEV_NAV_WAIT_FLOOR_CHANGE,
           ELEV_NAV_GOTOPOINT,
           ELEV_NAV_WAYPOINT_DONE,
           ELEV_NAV_DONE
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
    geometry_msgs::PoseStamped::ConstPtr base_position;
    tf::Transform base_positiontf_;
    tf::Transform waypoint_tf_;    
    std_msgs::Bool true_boool_msg;
    move_base_msgs::MoveBaseGoal goal;
    ros::Time stamp_start;
    ros::Duration stamp_wait;

    ropod_ros_msgs::ropod_demo_status_update ropod_fb_msg;      


    ElevatorNavigation();
    ~ElevatorNavigation();

    void startNavigation(std::string areaID, const ed::WorldModel& world);
    void pauseNavigation();
    void resumeNavigation();
    void resetNavigation();
    void stopNavigation();
    bool isPositionValid();
    bool isWaypointAchieved();
    bool checkDoorStatus(ropod_ros_msgs::DoorDetection door_status);
    TaskFeedbackCcu callNavigationStateMachine(ros::Publisher &navigation_cancel_pub, move_base_msgs::MoveBaseGoal* goal_ptr,
        bool& sendgoal, ropod_ros_msgs::DoorDetection door_status);
private:
    wm::Elevator elevator;
};



#endif

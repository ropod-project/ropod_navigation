#ifndef ED_ROPOD_NAVIGATION_PLUGIN_H_
#define ED_ROPOD_NAVIGATION_PLUGIN_H_

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <string>
#include <geometry_msgs/PoseArray.h>

/* C++ */
#include <iostream>

/* ROPOD ROS messages */
#include <ropod_ros_msgs/DoorDetection.h>
#include <floor_detection/DetectFloor.h>

#include <ropod_ros_msgs/Action.h>
#include <ropod_ros_msgs/RobotAction.h>
#include <ropod_ros_msgs/TaskProgressGOTO.h>
#include <ropod_ros_msgs/TaskProgressELEVATOR.h>
#include <ropod_ros_msgs/Area.h>
#include <ropod_ros_msgs/SubArea.h>
#include <ropod_ros_msgs/Waypoint.h>
#include <ropod_ros_msgs/Status.h>
#include <ropod_ros_msgs/GetElevatorWaypointsAction.h>
#include <ropod_ros_msgs/DockingCommand.h>
#include <ropod_ros_msgs/DockingFeedback.h>

#include <actionlib/client/simple_action_client.h>
#include <ropod_ros_msgs/RoutePlannerAction.h>

#include "waypoint_navigation.h"
#include "elevator_navigation.h"
#include "route_navigation_defines.h"
#include "mobidik_collection_navigation.h"

#include <ed/plugin.h>
#include <maneuver_navigation/Goal.h>
#include <maneuver_navigation/Configuration.h>
#include <maneuver_navigation/Feedback.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class RopodNavigation : public ed::Plugin
{


public:

    RopodNavigation();

    ~RopodNavigation();

    void initialize(ed::InitData& init);

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

    std::string GetEnv( const std::string & var ) ;

            enum {
    NAVTYPE_WAYPOINT = 0,
    NAVTYPE_ELEVATOR_WAITING,
    NAVTYPE_ELEVATOR_ENTERING,
    NAVTYPE_ELEVATOR_RIDING,
    NAVTYPE_ELEVATOR_EXITING,
    NAVTYPE_MOBIDIK_COLLECTION,
    NAVTYPE_MOBIDIK_RELEASE,
    NAVTYPE_NONE
};

   bool robotReal;

   void actionRoutePlannerCallback(const actionlib::SimpleClientGoalState& state, const ropod_ros_msgs::RoutePlannerResultConstPtr& result);

private:

    ros::CallbackQueue cb_queue_;

    ros::Subscriber subdoor_status_;

    ros::Subscriber objectMarkers_;

    ros::Subscriber wrenchFront_;

    ros::Subscriber wrenchLeft_;

    ros::Subscriber wrenchBack_;

    ros::Subscriber wrenchRight_;

    ros::Subscriber LLCmodeApplied_;

    ros::Subscriber loadAttachedApplied_;

    ros::Subscriber sub_ccu_goto_commands_;

    ros::Subscriber sub_ccu_dock_commands_;

    ros::Subscriber sub_ccu_undock_commands_;

    ros::Subscriber sub_ccu_wait_for_elevator_commands_;

    ros::Subscriber sub_ccu_enter_elevator_commands_;

    ros::Subscriber sub_ccu_ride_elevator_commands_;

    ros::Subscriber sub_ccu_exit_elevator_commands_;

    ros::Publisher movbase_cancel_pub_;

    ros::Publisher ropod_task_goto_fb_pub_;

    ros::Publisher ropod_task_dock_fb_pub_;

    ros::Publisher ropod_task_elevator_fb_pub_;

    ros::Publisher ObjectMarkers_pub_; // TODO remove

    ros::Publisher LLCmodeSet_pub_;

    ros::Publisher loadAttachedSet_pub_;

    ros::Publisher cmd_vel_pub_;

    ros::Publisher poses_waypoints_pub_;

    nav_msgs::Path path_msg_;

    std::vector<std::basic_string< char > > waypoint_ids_;

    move_base_msgs::MoveBaseGoal goal_;

    maneuver_navigation::Goal mn_goal_;

    maneuver_navigation::Feedback mn_feedback_;

    bool send_goal_;

    bool mobidikConnected_;
    
    bool sensorBack_;

    std_msgs::UInt16 controlMode_;

    ros::Subscriber sub_model_med_commands_;

    ros::Subscriber sub_debug_roputer_planner_;

    ros::Publisher sendGoal_pub_;

    ros::Publisher mn_sendGoal_pub_;

    ros::Subscriber mn_feedback_sub_;

    ros::Subscriber sub_navigation_fb_;

    ros::Publisher docking_command_pub_;

    ros::Subscriber docking_status_sub_;

    ros::Subscriber robot_footprint_sub_;


    actionlib::SimpleActionClient<ropod_ros_msgs::RoutePlannerAction> * route_planner_action_client_ptr_;

    void maneuverNavCallback(const maneuver_navigation::Feedback::ConstPtr &msg);
    bool mn_feedback_received_;
};

#endif

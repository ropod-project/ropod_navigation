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
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <string>

/* ROPOD ROS messages */
#include <ropod_ros_msgs/ropod_door_detection.h>

#include <ropod_ros_msgs/Action.h>
#include <ropod_ros_msgs/RobotAction.h>
#include <ropod_ros_msgs/TaskProgressGOTO.h>

#include "waypoint_navigation.h"
#include "elevator_navigation.h"
#include "route_navigation_defines.h"
#include "mobidik_collection_navigation.h"

#include <ed/plugin.h>
#include "ed/featureProperties_info.h"
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
    NAVTYPE_ELEVATOR,
    NAVTYPE_MOBIDIK_COLLECTION,
    NAVTYPE_NONE
};

    bool robotReal;

private:

    ros::CallbackQueue cb_queue_;    
        
    ros::Subscriber sub_ccu_commands_;
        
    ros::Subscriber subdoor_status_; 
    
    ros::Subscriber objectMarkers_; 
    
    ros::Subscriber wrenchFront_; 
    
    ros::Subscriber wrenchLeft_; 
    
    ros::Subscriber wrenchBack_; 
    
    ros::Subscriber wrenchRight_; 
    
    ros::Subscriber LLCmodeApplied_; 
    
    ros::Subscriber loadAttachedApplied_; 
    
    ros::Publisher movbase_cancel_pub_;
    
    ros::Publisher ropod_task_fb_pub_;
    
    ros::Publisher ObjectMarkers_pub_; // TODO remove
    
    ros::Publisher LLCmodeSet_pub_; 
    
    ros::Publisher loadAttachedSet_pub_; 
    
    ros::Publisher cmd_vel_pub_;
            
    nav_msgs::Path path_msg_;
    
    std::vector<std::basic_string< char > > waypoint_ids_;

    move_base_msgs::MoveBaseGoal goal_;
    
    maneuver_navigation::Goal mn_goal_;
    
    maneuver_navigation::Feedback mn_feedback_;    
    
    bool send_goal_;
    
    ros::Subscriber sub_model_med_commands_;
    
    ros::Publisher sendGoal_pub_;
    
    ros::Publisher mn_sendGoal_pub_;
    
    ros::Subscriber sub_navigation_fb_;
};

#endif
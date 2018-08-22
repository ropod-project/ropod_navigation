#ifndef ED_ROPOD_NAVIGATION_PLUGIN_H_
#define ED_ROPOD_NAVIGATION_PLUGIN_H_

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

#include "waypoint_navigation.h"
#include "elevator_navigation.h"
#include "route_navigation_defines.h"
#include "mobidik_collection_navigation.h"

/* Simple world model */
#include "simplified_world_model.h"

#include <ed/plugin.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class RopodNavigation : public ed::Plugin
{

public:

    RopodNavigation();

    ~RopodNavigation();

    void initialize(ed::InitData& init);

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    ros::CallbackQueue cb_queue_;

    ros::Subscriber sub_movebase_fb_;
        
    ros::Subscriber sub_ccu_commands_;
        
    ros::Subscriber subdoor_status_; 
    
    ros::Subscriber objectMarkers_; 
    
    ros::Publisher movbase_cancel_pub_;
    
    ros::Publisher ropod_task_fb_pub_;
    
    ros::Publisher ObjectMarkers_pub_; // TODO remove
    
    nav_msgs::Path path_msg_;
    
    std::vector<std::basic_string< char > > waypoint_ids_;

    move_base_msgs::MoveBaseGoal goal_;
    
    bool send_goal_;
    
     //tell the action client that we want to spin a thread by default
    MoveBaseClient* ac_;
    
};

#endif
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
#include <ed/update_request.h>
#include <ed/entity.h>

#include "simplified_world_model.h"
#include <ropod_ros_msgs/ropod_door_detection.h>
#include <ropod_ros_msgs/ropod_demo_status_update.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geolib/ros/tf_conversions.h>
#include <geolib/Shape.h>
#include <math.h>

#define WAYP_REACHED_DIST 0.5
#define GOAL_REACHED_DIST 0.2
#define GOAL_REACHED_ANG  20.0*3.141592/180.0

#define WAYP_MOBID_COLL_REACHED_DIST 0.05
#define GOAL_MOBID_COLL_REACHED_DIST 0.05
#define GOAL_MOBID_REACHED_ANG  2.0*3.141592/180.0

#define ROPOD_LENGTH 0.6
#define ROPOD_WIDTH ROPOD_LENGTH
#define DIST_IN_FRONT_OFF_MOBID 0.5
#define MOBIDIK_LENGTH 0.8

class MobidikCollection
{

    enum { MOBID_COLL_NAV_HOLD = 0,
           MOBID_COLL_NAV_IDLE,
           MOBID_COLL_NAV_PAUSED,
           MOBID_COLL_NAV_BUSY,
           MOBID_COLL_FIND_MOBIDIK,
           MOBID_COLL_FIND_SETPOINT_FRONT,
           MOBID_COLL_ROTATE,
           MOBID_COLL_NAV_GOTOPOINT,
           MOBID_COLL_NAV_WAYPOINT_DONE,
           MOBID_COLL_NAV_CONNECTING,
           MOBID_COLL_NAV_COUPLING,
           MOBID_COLL_NAV_DONE           
         };
         
    public:

    MobidikCollection();
    
    ~MobidikCollection();

    bool getMobidik(visualization_msgs::MarkerArray markerArray, visualization_msgs::Marker *marker) ;

    template <class T>
    void wrap(T *angle);

    void setMobidikPosition ( const ed::WorldModel& world,ed::UpdateRequest& req,  std::string mobidikAreaID, visualization_msgs::Marker mobidikMarker, ed::UUID* id, visualization_msgs::Marker *points );
    
    void getSetpointInFrontOfMobidik ( const ed::WorldModel& world, ed::UUID mobidikID, geo::Pose3D *setpoint, visualization_msgs::Marker* points);
    
    void pauseNavigation();
    
    void resumeNavigation();
    
    void resetNavigation();
    
    void stopNavigation();
    
    bool isPositionValid();
    
    void initNavState();
    
    bool isWaypointAchieved();
     
    TaskFeedbackCcu callNavigationStateMachine(ros::Publisher &movbase_cancel_pub, move_base_msgs::MoveBaseGoal* goal_ptr, bool& sendgoal, visualization_msgs::MarkerArray markerArray, std::string areaID, const ed::WorldModel& world, ed::UpdateRequest& req, visualization_msgs::MarkerArray *markerArraytest);
    
    move_base_msgs::MoveBaseActionFeedback::ConstPtr base_position_;
    
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
    
    tf::Transform base_positiontf_;
    tf::Transform waypoint_tf_;
    actionlib_msgs::GoalID emptyGoalID_;
    move_base_msgs::MoveBaseGoal goal_;
    ros::Time stamp_start_;
    ros::Duration stamp_wait_;
    ed::UUID MobidikID_ED_;

    ropod_ros_msgs::ropod_demo_status_update ropod_fb_msg_;
};



#endif

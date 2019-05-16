#ifndef MOBIDIK_COLLECTION_NAV_HH
#define MOBIDIK_COLLECTION_NAV_HH

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <string>
#include <typeinfo>
#include <stddef.h>

#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/entity.h>
#include "ed_sensor_integration/properties/featureProperties_info.h"
#include "route_navigation.h"

#include "simplified_world_model.h"
#include "route_navigation_defines.h"

#include <ropod_ros_msgs/DoorDetection.h>
#include <ropod_ros_msgs/TaskProgressDOCK.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geolib/ros/tf_conversions.h>
#include <geolib/Shape.h>
#include <math.h>

#include <maneuver_navigation/Goal.h>
#include <maneuver_navigation/Configuration.h>
#include <maneuver_navigation/Feedback.h>

#define WAYP_MOBID_COLL_REACHED_DIST 0.1 // [m]
#define GOAL_MOBID_COLL_REACHED_DIST 0.1 // [m]
#define GOAL_MOBID_LOAD_REACHED_DIST 0.2 // [m]
#define GOAL_MOBID_REL_REACHED_DIST 0.1 // [m]
#define GOAL_MOBID_REACHED_ANG  5.0*M_PI/180.0 // [rad]
#define GOAL_MOBID_REACHED_ANG_UNDOCK  10.0*M_PI/180.0 // [rad]

#ifndef NULL
#define NULL 0
#endif

#define ROPOD_LENGTH 0.6 // [m]
#define ROPOD_WIDTH ROPOD_LENGTH // [m]
#define DIST_IN_FRONT_OFF_MOBID 0.4 // [m]
// #define MOBIDIK_LENGTH 0.8 // [m]
#define BACKWARD_VEL_DOCKING 0.2 // [m/s]

#define N_COUNTS_WRENCHES 10 // [-]
#define MIN_FORCE_TOUCHED 20 // [N]
#define MAX_TORQUE_TOUCHED 3 // [Nm]

#define DIST_INTERMEDIATE_WAYPOINT_MOBID_RELEASE 1.0 // [m]
#define DIST_DISCONNECT_MOBID_RELEASE (std::sqrt(std::pow(0.5*ROPOD_WIDTH, 2.0) + std::pow(0.5*ROPOD_LENGTH, 2.0) ) + 0.2) // [m]
// #define DIST_DISCONNECT_MOBID_RELEASE 0.2 // [m]
#define DIST_CONN_X 0.1 //[m]
#define DIST_CONN_Y 0.05 // [m]

#define DIST_MOVE_FRONT_POSTDOCKING 0.4 //[m]
#define DIST_MOVE_FRONT_POSTRELEASING (std::sqrt(std::pow(0.5*ROPOD_WIDTH, 2.0) + std::pow(0.5*ROPOD_LENGTH, 2.0) ) - 0.5*ROPOD_LENGTH + 0.2) //[m]
#define TIME_WAIT_CHANGE_OF_FOOTPRINT 0.5 //[s]

#define MOBIDIK_WIDTH 0.72                              // [m]
#define MOBIDIK_LENGTH 0.81                             // [m]
#define MOBIDIK_MARGIN 0.22                              // [m]

#define INF 10000

class MobidikCollection
{

    enum { MOBID_COLL_NAV_HOLD = 0,
           MOBID_COLL_NAV_IDLE,
           MOBID_COLL_NAV_PAUSED,
           MOBID_COLL_NAV_BUSY,
           MOBID_COLL_FIND_MOBIDIK,
           MOBID_COLL_FIND_SETPOINT_FRONT,
           MOBID_COLL_NAV_GOTOPOINT,
           MOBID_COLL_NAV_WAYPOINT_DONE,
           MOBID_COLL_NAV_CONNECTING,
           MOBID_COLL_NAV_COUPLING,
           MOBID_COLL_NAV_EXIT_COLLECT_AREA,
           MOBID_COLL_NAV_DONE
         };

    enum { MOBID_REL_NAV_IDLE = 0,
//            MOBID_REL_GET_SETPOINT_FRONT_PARALLEL,
           MOBID_REL_GET_SETPOINT_FRONT,
           MOBID_REL_GOTO_SETPOINT_FRONT,
           MOBID_REL_GOTO_FINAL_MOBIDIK_POS,
           MOBID_REL_DECOUPLING,
           MOBID_REL_ROTATE,
           MOBID_REL_NAV_GOTOPOINT,
           MOBID_REL_INIT_DECOUPLING,
           MOBID_REL_NAV_BUSY,
           MOBID_REL_NAV_WAYPOINT_DONE,
           MOBID_REL_NAV_WAIT_CHANGE_FOOTPRINT,
           MOBID_REL_NAV_DONE,
           MOBID_REL_NAV_HOLD,
           MOBID_REL_NAV_PAUSED
         };

         enum {
    DOCKING_COMMAND_DOCK = 1,
    DOCKING_COMMAND_RELEASE = 2,
    DOCKING_COMMAND_STOP = 3 // NOT IMPLEMENTED YET
};

            enum {
    DOCKING_FB_REST = 0,
    DOCKING_FB_DOCKED = 10,
};



    public:

    MobidikCollection();

    ~MobidikCollection();

//     ed::PropertyKey<ed::tracking::FeatureProperties> featurePropertiesKey;
     ed::PropertyKey<ed::tracking::FeatureProperties> featureProperties;

//     bool getMobidik(const ed::WorldModel& world, ed::EntityConstPtr* mobidikID);
    bool getMobidik(const ed::WorldModel& world, ed::UUID* mobidikID);

    template <class T>
    void wrap2pi(T *angle);

     template <class T>
    void wrap2twopi(T *angle);

    //void setMobidikPosition ( const ed::WorldModel& world,ed::UpdateRequest& req,  std::string mobidikAreaID, visualization_msgs::Marker mobidikMarker, ed::UUID* id, visualization_msgs::Marker *points );
//     bool setMobidikPosition ( const ed::WorldModel& world,ed::UpdateRequest& req, std::string mobidikAreaID, const ed::EntityConstPtr mobidikID, visualization_msgs::Marker* points ) ;
    bool setMobidikPosition ( const ed::WorldModel& world, ed::UpdateRequest& req, std::string mobidikAreaID,   ed::UUID mobidikID, visualization_msgs::Marker* points ) ;
//     bool getMobidikPosition( const ed::WorldModel& world, const ed::EntityConstPtr, geo::Pose3D *mobidikPose );
    
    bool updateMobidikPosition ( const ed::WorldModel& world,ed::UpdateRequest& req, ed::UUID mobidikID, visualization_msgs::Marker* points ) ;
    
    bool getEntityPointer(const ed::WorldModel& world, ed::UUID MobidikID_ED, ed::EntityConstPtr& entityPointer);

//    bool getSetpointInFrontOfMobidik ( const ed::WorldModel& world, const ed::EntityConstPtr mobidikID, geo::Pose3D *setpoint, visualization_msgs::Marker* points);
 bool getSetpointInFrontOfMobidik ( const ed::WorldModel& world, ed::UpdateRequest& req, const ed::UUID mobidikID, geo::Pose3D *setpoint, visualization_msgs::Marker* points, float distance);
    
//    void getSetpointInFrontOfMobidik ( const ed::WorldModel& world, ed::UUID mobidikID, geo::Pose3D *setpoint, visualization_msgs::Marker* points);

    void pauseNavigation();

    void resumeNavigation();

    void resetNavigation();

    void stopNavigation();

    bool isPositionValid();

    void printWrenches(geometry_msgs::WrenchStamped wrench);

    geometry_msgs::WrenchStamped determineAvgWrench(std::vector<geometry_msgs::WrenchStamped> wrenchVector);

    void determineAvgWrenches();

    void initAvgWrench(geometry_msgs::WrenchStamped *wrench);

    void initNavState();

    void initNavStateRelease();

    bool isWaypointAchieved(double& dist_tolerance, double& angle_tolerance);

    TaskFeedbackCcu callNavigationStateMachine(ros::Publisher &nav_cancel_pub, maneuver_navigation::Goal &mn_goal, bool& sendgoal, visualization_msgs::MarkerArray markerArray,
                                               std::string areaID, const ed::WorldModel& world, ed::UpdateRequest& req, visualization_msgs::MarkerArray *markerArraytest,
                                               std_msgs::UInt16 * controlMode, ros::Publisher &cmv_vel_pub,  ropodNavigation::wrenches bumperWrenches, const bool robotReal,
                                               ros::Publisher &docking_pub, ropod_ros_msgs::DockingFeedback dockingFeedback);

    void getFinalMobidikPos ( const ed::WorldModel& world, std::string mobidikAreaID, geo::Pose3D *mobidikPosition, geo::Pose3D *disconnectSetpoint , geo::Pose3D *setpointInFrontOfMobidik, visualization_msgs::Marker* points );

    void point2goal(geo::Pose3D *setpoint);

    TaskFeedbackCcu callReleasingStateMachine(ros::Publisher &movbase_cancel_pub, maneuver_navigation::Goal &mn_goal, bool& sendgoal, visualization_msgs::MarkerArray markerArray,
                                              std::string areaID, const ed::WorldModel& world, ed::UpdateRequest& req, visualization_msgs::MarkerArray *markerArraytest,
                                              std_msgs::UInt16* controlMode, ros::Publisher &cmv_vel_pub, ropodNavigation::wrenches bumperWrenches, bool *mobidikConnected,
                                              const bool robotReal, ros::Publisher &docking_pub, ropod_ros_msgs::DockingFeedback dockingFeedback);

    geometry_msgs::PoseStamped::ConstPtr base_position_;

    private:

    nav_msgs::Path planned_route_;
    int planned_route_size_;
    bool route_busy_;
    bool nav_paused_req_;
    int  waypoint_cnt_;
    int nav_state_;
    int nav_state_release_;
    int nav_next_state_;
    int nav_next_state_release_;
    int nav_state_bpause_;
    int nav_next_state_wp_;
    int nav_next_state_wp_release_;

    double xy_goal_tolerance_;
    double yaw_goal_tolerance_;

    tf::Transform base_positiontf_;
    tf::Transform waypoint_tf_;
    std_msgs::Bool true_bool_msg_;
    move_base_msgs::MoveBaseGoal goal_;
    ros::Time stamp_start_;
    ros::Duration stamp_wait_;
    ed::UUID MobidikID_ED_;
    geo::Pose3D setpoint_;
    std::string orientationWPID_;
//    ed::tracking::FeatureProperties mobidikFeatures_;  
    
    bool avgWrenchesDetermined_;
    ropodNavigation::wrenches avgWrenches_;

    ed::tracking::FeatureProperties mobidikFeatures_;

    std::vector<ropodNavigation::wrenches> bumperWrenchesVector_;

    geo::Pose3D finalMobidikPosition_, disconnectSetpoint_;
//     geo::Pose3D mobidikPos_   ;




};



#endif

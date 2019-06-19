#ifndef ELEVATOR_ELEV_NAV_HH
#define ELEVATOR_ELEV_NAV_HH

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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
#include <maneuver_navigation/Goal.h>

#include <ropod_ros_msgs/DoorDetection.h>
#include <floor_detection/DetectFloor.h>
#include <map_switcher/SwitchMap.h>
#include <ropod_ros_msgs/GetElevatorWaypointsAction.h>
#include <ropod_ros_msgs/GetTopologyNodeAction.h>
#include <ropod_ros_msgs/GetDoorStatus.h>
#include <ropod_ros_msgs/Position.h>

#include "simplified_world_model.h"
#include "route_navigation_defines.h"

#define ELEV_WAYP_REACHED_DIST 0.5
#define ELEV_GOAL_REACHED_DIST 0.3
#define ELEV_GOAL_REACHED_ANG  20.0*3.141592/180.0


class ElevatorNavigation
{
    enum
    {
        IDLE,
        GOTO_WAITING_POINT,
        WAIT_FOR_ELEVATOR,
        ENTER_ELEVATOR,
        RIDE_ELEVATOR,
        EXIT_ELEVATOR
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

    actionlib::SimpleActionClient<ropod_ros_msgs::GetElevatorWaypointsAction> *elevator_waypoints_client;
    actionlib::SimpleActionClient<ropod_ros_msgs::GetTopologyNodeAction> *topology_node_client;
    ros::ServiceClient get_door_status_client;
    ros::ServiceClient get_floor_client;
    ros::ServiceClient switch_map_client;
    ros::Publisher init_pose_publisher;

    ElevatorNavigation();
    ~ElevatorNavigation();

    void initElevatorNavigation(int elevator_id, int elevator_door_id);
    void getElevatorWaypoints(int elevator_id, int elevator_door_id);
    void setWaitingPose(maneuver_navigation::Goal &mn_goal, bool& send_goal);
    void setInsideElevatorPose(maneuver_navigation::Goal &mn_goal, bool& send_goal);
    void setOutsideElevatorPose(maneuver_navigation::Goal &mn_goal, bool& send_goal, std::string outside_area_id);
    void setDestinationFloor(int destination_floor);
    void pauseNavigation();
    void resumeNavigation();
    void resetNavigation();
    void stopNavigation();

    bool isPositionValid();
    bool isWaypointAchieved();
    bool isDoorOpen();
    bool destinationFloorReached();

    TaskFeedbackCcu callNavigationStateMachine(maneuver_navigation::Goal &mn_goal, bool& send_goal, std::string outside_area_id="", int destination_floor=-1);
private:
    wm::Elevator elevator;
    ropod_ros_msgs::Position elevator_door_position;
    geometry_msgs::Pose waiting_pose;
    geometry_msgs::Pose inside_elevator_pose;
    int elevator_id;
    int destination_floor;
    bool inside_elevator;
    bool goal_sent;
};

#endif

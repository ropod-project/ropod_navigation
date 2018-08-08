#include "elevator_navigation.h"

/*--------------------------------------------------------*/
ElevatorNavigation::ElevatorNavigation() {

};

/*--------------------------------------------------------*/
ElevatorNavigation::~ElevatorNavigation() {
    base_position.reset();
};

/*--------------------------------------------------------*/
void ElevatorNavigation::startNavigation(wm::Elevator &elevator,nav_msgs::Path Pathmsg) {
    resetNavigation();
    route_busy = true;
    // if we get elevator poses from ropod_wm_mediator, use them
    if (Pathmsg.poses.size() == 3)
    {
      elevator.setInsideElevatorPose(wm::getWMPose(Pathmsg.poses[0]), wm::getWMPose(Pathmsg.poses[1]));
      elevator.setOutsideElevatorPose(wm::getWMPose(Pathmsg.poses[2]));      
      ROS_INFO("Poses from elevator message assigned: %f", elevator.wayp1_elevator.position.x);
    }
    ROS_INFO("start navigation trough elevator");
    return;
}

/*--------------------------------------------------------*/
void ElevatorNavigation::pauseNavigation() {
    nav_paused_req = true;
    nav_state_bpause = nav_state;
    nav_next_state = ELEV_NAV_PAUSED;
    ROS_INFO("Navigation paused");
    return;
};

/*--------------------------------------------------------*/
void ElevatorNavigation::resumeNavigation() {
    nav_paused_req = false;
    if(nav_state_bpause == ELEV_NAV_BUSY)
        nav_next_state = ELEV_NAV_GOTOPOINT;
    else
        nav_next_state = nav_state_bpause;
    ROS_INFO("Navigation resumed");
    return;
}

/*--------------------------------------------------------*/
void ElevatorNavigation::resetNavigation() {
    base_position.reset();
    route_busy = false;
    nav_paused_req = false;
    waypoint_cnt = 0;
    nav_state = ELEV_NAV_IDLE;
    nav_next_state = ELEV_NAV_IDLE;
    nav_state_bpause = ELEV_NAV_IDLE;
    nav_next_state_wp = ELEV_NAV_IDLE;
}

/*--------------------------------------------------------*/
void ElevatorNavigation::stopNavigation() {
    base_position.reset();
    route_busy = false;
    waypoint_cnt = 0;
}

/*--------------------------------------------------------*/
bool ElevatorNavigation::isPositionValid() {
    if(base_position)
        return true;
    else
        return false;
}

/*--------------------------------------------------------*/
bool ElevatorNavigation::isWaypointAchieved() {

    // In elevator we checked whether teh specific position is reached.
    tf::Quaternion qtemp = tf::Quaternion(base_position->feedback.base_position.pose.orientation.x, base_position->feedback.base_position.pose.orientation.y,
                                          base_position->feedback.base_position.pose.orientation.z,base_position->feedback.base_position.pose.orientation.w);
    tf::Vector3 v3temp = tf::Vector3(base_position->feedback.base_position.pose.position.x,base_position->feedback.base_position.pose.position.y, 0.0);
    base_positiontf_ = tf::Transform( qtemp, v3temp);
    qtemp = tf::Quaternion(goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y,
                           goal.target_pose.pose.orientation.z,goal.target_pose.pose.orientation.w);
    v3temp = tf::Vector3(goal.target_pose.pose.position.x,goal.target_pose.pose.position.y, 0.0);
    waypoint_tf_ = tf::Transform( qtemp, v3temp);

    tf::Transform diff_tf = base_positiontf_.inverseTimes(waypoint_tf_);
    v3temp = diff_tf.getOrigin();
    qtemp = diff_tf.getRotation();

    if (pow( v3temp.x(),2) + pow(v3temp.y(),2) < pow(GOAL_REACHED_DIST,2)
            && fabs(qtemp.getAngle()) < GOAL_REACHED_ANG)
        return true;
    else
        return false;
}

/*--------------------------------------------------------*/
bool ElevatorNavigation::checkDoorStatus(ropod_ros_msgs::ropod_door_detection door_status) {
    if(door_status.undetectable)
        return false;
    else if(door_status.closed)
        return false;
    else if(door_status.open)
        return true;
    else
        return false;
}

/*--------------------------------------------------------*/
TaskFeedbackCcu ElevatorNavigation::callNavigationStateMachine(ros::Publisher &movbase_cancel_pub, move_base_msgs::MoveBaseGoal* goal_ptr,
						   bool& sendgoal, wm::Elevator elevator, ropod_ros_msgs::ropod_door_detection door_status) {
  TaskFeedbackCcu tfb_nav;
  tfb_nav.wayp_n = waypoint_cnt;
  tfb_nav.fb_nav = NAV_BUSY;
    sendgoal = false;

    switch(nav_state) {

    case ELEV_NAV_IDLE: // No waypoints received yet.
        tfb_nav.fb_nav = NAV_IDLE;
        if (route_busy == true){
            nav_next_state = ELEV_NAV_CHECKDOOR_IN;
	    ROS_INFO("Waiting for door");
	}
        break;

    case ELEV_NAV_CHECKDOOR_IN: 	//we'll send the the next goal to the robot
        if(checkDoorStatus(door_status)) {
	  waypoint_cnt = waypoint_cnt +1;
	  goal.target_pose.pose.position.x=elevator.wayp1_elevator.position.x;
	  goal.target_pose.pose.position.y=elevator.wayp1_elevator.position.y;
	  goal.target_pose.pose.position.z=elevator.wayp1_elevator.position.z;
	  goal.target_pose.pose.orientation.x=elevator.wayp1_elevator.orientation.x;
	  goal.target_pose.pose.orientation.y=elevator.wayp1_elevator.orientation.y;
	  goal.target_pose.pose.orientation.z=elevator.wayp1_elevator.orientation.z;
	  goal.target_pose.pose.orientation.w=elevator.wayp1_elevator.orientation.w;
	  nav_next_state_wp = ELEV_NAV_TURN_FACING_EXIT;
	  stamp_start = ros::Time::now();
	  stamp_wait = ros::Duration(20.0); // wait from the moment you want to enter to checl way out. This will be replaced by communication with elevator system
	  nav_next_state = ELEV_NAV_GOTOPOINT;
	  ROS_INFO("Door is open. Entering elevator");
        }
        break;

    case ELEV_NAV_GOTOPOINT:
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        ROS_INFO("Sending goal");
        sendgoal = true;
	tfb_nav.fb_nav = NAV_GOTOPOINT;
        nav_next_state = ELEV_NAV_BUSY;
        break;

    case ELEV_NAV_BUSY: //
        if (!isPositionValid()) {
            nav_next_state = ELEV_NAV_HOLD;
            break;
        }
        if (isWaypointAchieved())
            nav_next_state = ELEV_NAV_WAYPOINT_DONE;

        break;
    case ELEV_NAV_TURN_FACING_EXIT:
	waypoint_cnt = waypoint_cnt +1;
	goal.target_pose.pose.position.x=elevator.wayp2_elevator.position.x;
	goal.target_pose.pose.position.y=elevator.wayp2_elevator.position.y;
	goal.target_pose.pose.position.z=elevator.wayp2_elevator.position.z;
	goal.target_pose.pose.orientation.x=elevator.wayp2_elevator.orientation.x;
	goal.target_pose.pose.orientation.y=elevator.wayp2_elevator.orientation.y;
	goal.target_pose.pose.orientation.z=elevator.wayp2_elevator.orientation.z;
	goal.target_pose.pose.orientation.w=elevator.wayp2_elevator.orientation.w;
	nav_next_state_wp = ELEV_NAV_WAIT_FLOOR_CHANGE;
	nav_next_state = ELEV_NAV_GOTOPOINT;
	ROS_INFO("Turn to face elevator exit");      
    case ELEV_NAV_WAIT_FLOOR_CHANGE: //
        // wait for signal that we have arrived to the floor. For now just time based
        if( ros::Time::now() - stamp_start > stamp_wait) {
            nav_next_state = ELEV_NAV_CHECKDOOR_OUT;
            ROS_INFO("Reached target floor. Waiting for door");
        }
        break;

    case ELEV_NAV_CHECKDOOR_OUT: //
      ROS_INFO("checkDoorStatus(door_status): %d", checkDoorStatus(door_status));

        if(checkDoorStatus(door_status)) {
	    waypoint_cnt = waypoint_cnt +1;
            goal.target_pose.pose.position.x=elevator.wayp_out.position.x;
            goal.target_pose.pose.position.y=elevator.wayp_out.position.y;
            goal.target_pose.pose.position.z=elevator.wayp_out.position.z;
            goal.target_pose.pose.orientation.x=elevator.wayp_out.orientation.x;
            goal.target_pose.pose.orientation.y=elevator.wayp_out.orientation.y;
            goal.target_pose.pose.orientation.z=elevator.wayp_out.orientation.z;
            goal.target_pose.pose.orientation.w=elevator.wayp_out.orientation.w;

            nav_next_state_wp = ELEV_NAV_DONE;
            nav_next_state = ELEV_NAV_GOTOPOINT;
	    ROS_INFO("Door is open. Moving outside elevator");
        }
        break;

    case ELEV_NAV_WAYPOINT_DONE: //
        tfb_nav.fb_nav = NAV_WAYPOINT_DONE;
        nav_next_state = nav_next_state_wp;

        break;
    case ELEV_NAV_DONE: //
        ROS_INFO("Navigation done");
	tfb_nav.fb_nav = NAV_DONE;
        stopNavigation();
        movbase_cancel_pub.publish(emptyGoalID);
        nav_next_state = ELEV_NAV_IDLE;
        break;

    case ELEV_NAV_HOLD: //
        ROS_INFO("Navigation on hold to receive feedback");
        if (isPositionValid()) // check we have a valid position
            nav_next_state = ELEV_NAV_BUSY;
        break;

    case ELEV_NAV_PAUSED: // this state is reached via a callback
        if(nav_paused_req) {
            movbase_cancel_pub.publish(emptyGoalID);
            nav_paused_req = false;
        }
        break;

    default:
        nav_next_state = ELEV_NAV_IDLE;
    }

    nav_state = nav_next_state;

    *goal_ptr = goal;

    return tfb_nav;
}

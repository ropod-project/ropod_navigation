#include "elevator_navigation.h"

/*--------------------------------------------------------*/
Elevator_navigation::Elevator_navigation() {

};

/*--------------------------------------------------------*/
Elevator_navigation::~Elevator_navigation() {
    base_position_.reset();
};

/*--------------------------------------------------------*/
void Elevator_navigation::start_navigation() {
    reset_navigation();
    route_busy = true;
    ROS_INFO("start navigation trough elevator");

    return;
}

/*--------------------------------------------------------*/
void Elevator_navigation::pause_navigation() {
    nav_paused_req = true;
    nav_state_bpause = nav_state;
    nav_next_state = NAV_PAUSED;
    ROS_INFO("Navigation paused");
    return;
};

/*--------------------------------------------------------*/
void Elevator_navigation::resume_navigation() {
    nav_paused_req = false;
    if(nav_state_bpause == NAV_BUSY)
        nav_next_state = NAV_GOTOPOINT;
    else
        nav_next_state = nav_state_bpause;
    ROS_INFO("Navigation resumed");
    return;
}

/*--------------------------------------------------------*/
void Elevator_navigation::reset_navigation() {
    base_position_.reset();
    route_busy = false;
    nav_paused_req = false;
    waypoint_cnt = 0;
    nav_state = NAV_IDLE;
    nav_next_state = NAV_IDLE;
    nav_state_bpause = NAV_IDLE;
    nav_next_state_wp = NAV_IDLE;
}

/*--------------------------------------------------------*/
void Elevator_navigation::stop_navigation() {
    base_position_.reset();
    route_busy = false;
    waypoint_cnt = 0;
}

/*--------------------------------------------------------*/
bool Elevator_navigation::is_position_valid() {
    if(base_position_)
        return true;
    else
        return false;
}

/*--------------------------------------------------------*/
bool Elevator_navigation::is_waypoint_achieved() {

    // In elevator we checked whether teh specific position is reached.
    tf::Quaternion qtemp = tf::Quaternion(base_position_->feedback.base_position.pose.orientation.x, base_position_->feedback.base_position.pose.orientation.y,
                                          base_position_->feedback.base_position.pose.orientation.z,base_position_->feedback.base_position.pose.orientation.w);
    tf::Vector3 v3temp = tf::Vector3(base_position_->feedback.base_position.pose.position.x,base_position_->feedback.base_position.pose.position.y, 0.0);
    base_position_tf_ = tf::Transform( qtemp, v3temp);
    qtemp = tf::Quaternion(goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y,
                           goal.target_pose.pose.orientation.z,goal.target_pose.pose.orientation.w);
    v3temp = tf::Vector3(goal.target_pose.pose.position.x,goal.target_pose.pose.position.y, 0.0);
    waypoint_tf_ = tf::Transform( qtemp, v3temp);

    tf::Transform diff_tf = base_position_tf_.inverseTimes(waypoint_tf_);
    v3temp = diff_tf.getOrigin();
    qtemp = diff_tf.getRotation();

    if (pow( v3temp.x(),2) + pow(v3temp.y(),2) < pow(GOAL_REACHED_DIST,2)
            && fabs(qtemp.getAngle()) < GOAL_REACHED_ANG)
        return true;
    else
        return false;
}

/*--------------------------------------------------------*/
bool Elevator_navigation::check_door(ropod_demo_dec_2017::doorDetection doorStatus) {
    if(doorStatus.undetectable)
        return false;
    else if(doorStatus.closed)
        return false;
    else if(doorStatus.open)
        return true;
    else
        return false;
}

/*--------------------------------------------------------*/
void Elevator_navigation::navigation_state_machine(wm::Elevator elevator,ros::Publisher movbase_cancel_pub, move_base_msgs::MoveBaseGoal* goal_ptr, 
						   bool& sendgoal, ropod_demo_dec_2017::doorDetection doorStatus) {
    sendgoal = false;

    switch(nav_state) {
      
    case NAV_IDLE: // No waypoints received yet.
        if (route_busy == true)
            nav_next_state = NAV_CHECKDOOR_IN;
        break;

    case NAV_CHECKDOOR_IN: 	//we'll send the the next goal to the robot
        if(check_door(doorStatus)) {
            goal.target_pose.pose.position.x=elevator.wayp_elevator.position.x;
            goal.target_pose.pose.position.y=elevator.wayp_elevator.position.y;
            goal.target_pose.pose.position.z=elevator.wayp_elevator.position.z;
            goal.target_pose.pose.orientation.x=elevator.wayp_elevator.orientation.x;
            goal.target_pose.pose.orientation.y=elevator.wayp_elevator.orientation.y;
            goal.target_pose.pose.orientation.z=elevator.wayp_elevator.orientation.z;
            goal.target_pose.pose.orientation.w=elevator.wayp_elevator.orientation.w;
            //goal.target_pose.pose =
            nav_next_state_wp = NAV_WAIT_FLOOR_CHANGE;
            stamp_start = ros::Time::now();
            stamp_wait = ros::Duration(20.0); // wait five seconds from the moment you want to enter to checl way out. This will be replaced by communication with elevator system
            nav_next_state = NAV_GOTOPOINT;
        }
        break;
	
    case NAV_GOTOPOINT:
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        ROS_INFO("Sending goal");
        sendgoal = true;
        nav_next_state = NAV_BUSY;
        break;
	
    case NAV_BUSY: //
        if (!is_position_valid()) {
            nav_next_state = NAV_HOLD;
            break;
        }
        if (is_waypoint_achieved())
            nav_next_state = NAV_WAYPOINT_DONE;

        break;
	
    case NAV_WAIT_FLOOR_CHANGE: //
        // wait for signal that we have arrived to the floor. For now just time based
        if( ros::Time::now() - stamp_start > stamp_wait) {

            nav_next_state = NAV_CHECKDOOR_OUT;

        }
        break;

    case NAV_CHECKDOOR_OUT: //
        if(check_door(doorStatus)) {
            goal.target_pose.pose.position.x=elevator.wayp_out.position.x;
            goal.target_pose.pose.position.y=elevator.wayp_out.position.y;
            goal.target_pose.pose.position.z=elevator.wayp_out.position.z;
            goal.target_pose.pose.orientation.x=elevator.wayp_out.orientation.x;
            goal.target_pose.pose.orientation.y=elevator.wayp_out.orientation.y;
            goal.target_pose.pose.orientation.z=elevator.wayp_out.orientation.z;
            goal.target_pose.pose.orientation.w=elevator.wayp_out.orientation.w;

            nav_next_state_wp = NAV_DONE;
            nav_next_state = NAV_GOTOPOINT;
        }
        break;
	
    case NAV_WAYPOINT_DONE: //
        nav_next_state = nav_next_state_wp;

        break;
    case NAV_DONE: //
        ROS_INFO("Navigation done");
        stop_navigation();
        movbase_cancel_pub.publish(emptyGoalID);
        nav_next_state = NAV_IDLE;
        break;
	
    case NAV_HOLD: //
        ROS_INFO("Navigation on hold to receive feedback");
        if (is_position_valid()) // check we have a valid position
            nav_next_state = NAV_BUSY;
        break;
	
    case NAV_PAUSED: // this state is reached via a callback
        if(nav_paused_req) {
            movbase_cancel_pub.publish(emptyGoalID);
            nav_paused_req = false;
        }
        break;

    default:
        nav_next_state = NAV_IDLE;
    }

    nav_state = nav_next_state;

    *goal_ptr = goal;
}
#include "elevator_navigation.h"

/*--------------------------------------------------------*/
ElevatorNavigation::ElevatorNavigation()
{
    true_boool_msg.data = true;
};

/*--------------------------------------------------------*/
ElevatorNavigation::~ElevatorNavigation()
{
    base_position.reset();
};

/*--------------------------------------------------------*/
void ElevatorNavigation::startNavigation(std::string ElevatorareaID, const ed::WorldModel& world)
{
    resetNavigation();
    
    // get waypoints from wm
    geo::Quaternion rotationWP;
    
    std::string orientationWP_ID = "orient_wp_" +  ElevatorareaID;
    std::string orientationWP_out_ID = "orient_wp_out_" +  ElevatorareaID;
    std::string orientationWP_waiting_ID = "orient_wp_waiting_area_" +  ElevatorareaID;  
    
    geo::Pose3D poseWP;
    geo::Pose3D poseWP_orientationWP_ID;
    geo::Pose3D poseWP_orientationWP_out_ID;
    geo::Pose3D poseWP_orientationWP_waiting_ID;
    
    
    
    for ( ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it )
    {
        const ed::EntityConstPtr& e = *it; 

        if ( orientationWP_waiting_ID.compare ( e.get()->id().str() ) == 0 ) // function returns 0 if strings are equal
            poseWP_orientationWP_ID = e.get()->pose();
        
        if ( orientationWP_ID.compare ( e.get()->id().str() ) == 0 ) // function returns 0 if strings are equal
            poseWP_orientationWP_out_ID = e.get()->pose();
            
        if ( orientationWP_out_ID.compare ( e.get()->id().str() ) == 0 ) // function returns 0 if strings are equal
            poseWP_orientationWP_waiting_ID = e.get()->pose();
     
    }
    
    poseWP = poseWP_orientationWP_ID;
    rotationWP = poseWP.getQuaternion();
    elevator.wayp_wait.position.x = poseWP.getOrigin().getX();
    elevator.wayp_wait.position.y = poseWP.getOrigin().getY();
    elevator.wayp_wait.position.z = poseWP.getOrigin().getZ();
    elevator.wayp_wait.orientation.w = rotationWP.getW();
    elevator.wayp_wait.orientation.x = rotationWP.getX();
    elevator.wayp_wait.orientation.y = rotationWP.getY();
    elevator.wayp_wait.orientation.z = rotationWP.getZ();    
    
    
    
    poseWP = poseWP_orientationWP_out_ID;
    rotationWP = poseWP.getQuaternion();
    elevator.wayp1_elevator.position.x = poseWP.getOrigin().getX();
    elevator.wayp1_elevator.position.y = poseWP.getOrigin().getY();
    elevator.wayp1_elevator.position.z = poseWP.getOrigin().getZ();
    elevator.wayp1_elevator.orientation.w = rotationWP.getW();
    elevator.wayp1_elevator.orientation.x = rotationWP.getX();
    elevator.wayp1_elevator.orientation.y = rotationWP.getY();
    elevator.wayp1_elevator.orientation.z = rotationWP.getZ();
    elevator.wayp2_elevator.position.x = poseWP.getOrigin().getX();
    elevator.wayp2_elevator.position.y = poseWP.getOrigin().getY();
    elevator.wayp2_elevator.position.z = poseWP.getOrigin().getZ();
    elevator.wayp2_elevator.orientation.w = rotationWP.getW();
    elevator.wayp2_elevator.orientation.x = rotationWP.getX();
    elevator.wayp2_elevator.orientation.y = rotationWP.getY();
    elevator.wayp2_elevator.orientation.z = rotationWP.getZ();        
/*    tf::Quaternion q ( rotationWP.getX(), rotationWP.getY(), rotationWP.getZ(), rotationWP.getW() );
    tf::Matrix3x3 matrix ( q );
    double WP_roll, WP_pitch, WP_yaw;
    matrix.getRPY ( WP_roll, WP_pitch, WP_yaw );   
    WP_yaw = angles::normalize_angle( WP_yaw+ M_PI);
    q.setRPY(WP_roll, WP_pitch, WP_yaw);
    // TODO: only when no load is attached
    elevator.wayp2_elevator.orientation.w = q.getW();
    elevator.wayp2_elevator.orientation.x = q.getX();
    elevator.wayp2_elevator.orientation.y = q.getY();
    elevator.wayp2_elevator.orientation.z = q.getZ();  */            

    
    poseWP = poseWP_orientationWP_waiting_ID;
    rotationWP = poseWP.getQuaternion();
    elevator.wayp_out.position.x = poseWP.getOrigin().getX();
    elevator.wayp_out.position.y = poseWP.getOrigin().getY();
    elevator.wayp_out.position.z = poseWP.getOrigin().getZ();
    elevator.wayp_out.orientation.w = rotationWP.getW();
    elevator.wayp_out.orientation.x = rotationWP.getX();
    elevator.wayp_out.orientation.y = rotationWP.getY();
    elevator.wayp_out.orientation.z = rotationWP.getZ();            
    
    ROS_INFO("Poses from elevator message assigned: %f", elevator.wayp1_elevator.position.x);

    
    
    ROS_INFO("start navigation trough elevator");
    
    route_busy = true;
    
    return;
}

/*--------------------------------------------------------*/
void ElevatorNavigation::pauseNavigation()
{
    nav_paused_req = true;
    nav_state_bpause = nav_state;
    nav_next_state = ELEV_NAV_PAUSED;
    ROS_INFO("Navigation paused");
    return;
};

/*--------------------------------------------------------*/
void ElevatorNavigation::resumeNavigation()
{
    nav_paused_req = false;
    if(nav_state_bpause == ELEV_NAV_BUSY)
        nav_next_state = ELEV_NAV_GOTOPOINT;
    else
        nav_next_state = nav_state_bpause;
    ROS_INFO("Navigation resumed");
    return;
}

/*--------------------------------------------------------*/
void ElevatorNavigation::resetNavigation()
{
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
void ElevatorNavigation::stopNavigation()
{
    base_position.reset();
    route_busy = false;
    waypoint_cnt = 0;
}

/*--------------------------------------------------------*/
bool ElevatorNavigation::isPositionValid()
{
    if(base_position)
        return true;
    else
        return false;
}

/*--------------------------------------------------------*/
bool ElevatorNavigation::isWaypointAchieved()
{

    // In elevator we checked whether teh specific position is reached.
    tf::Quaternion qtemp = tf::Quaternion(base_position->pose.orientation.x, base_position->pose.orientation.y, 
                                          base_position->pose.orientation.z, base_position->pose.orientation.w);
    tf::Vector3 v3temp = tf::Vector3(base_position->pose.position.x, base_position->pose.position.y, 0.0);
    base_positiontf_ = tf::Transform( qtemp, v3temp);
    qtemp = tf::Quaternion(goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y,
                           goal.target_pose.pose.orientation.z,goal.target_pose.pose.orientation.w);
    v3temp = tf::Vector3(goal.target_pose.pose.position.x,goal.target_pose.pose.position.y, 0.0);
    waypoint_tf_ = tf::Transform( qtemp, v3temp);

    tf::Transform diff_tf = base_positiontf_.inverseTimes(waypoint_tf_);
    v3temp = diff_tf.getOrigin();
    qtemp = diff_tf.getRotation();

    if (pow( v3temp.x(),2) + pow(v3temp.y(),2) < pow(ELEV_GOAL_REACHED_DIST,2)
            && fabs(qtemp.getAngle()) < ELEV_GOAL_REACHED_ANG)
        return true;
    else
        return false;
}

/*--------------------------------------------------------*/
bool ElevatorNavigation::checkDoorStatus(ropod_ros_msgs::DoorDetection door_status)
{
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
TaskFeedbackCcu ElevatorNavigation::callNavigationStateMachine(ros::Publisher &navigation_cancel_pub, move_base_msgs::MoveBaseGoal* goal_ptr, 
        bool& sendgoal, ropod_ros_msgs::DoorDetection door_status)
{
    TaskFeedbackCcu tfb_nav;
    tfb_nav.wayp_n = waypoint_cnt;
    tfb_nav.fb_nav = NAV_BUSY;
    sendgoal = false;

    switch(nav_state)
    {

    case ELEV_NAV_IDLE: // No waypoints received yet.
        tfb_nav.fb_nav = NAV_IDLE;
        if (route_busy == true)
        {
            nav_next_state = ELEV_NAV_CHECKDOOR_IN;
            ROS_INFO("Waiting for door");
        }
        break;

    case ELEV_NAV_CHECKDOOR_IN:         //we'll send the the next goal to the robot
        if(checkDoorStatus(door_status))
        {
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
        if (!isPositionValid())
        {
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
        if( ros::Time::now() - stamp_start > stamp_wait)
        {
            nav_next_state = ELEV_NAV_CHECKDOOR_OUT;
            ROS_INFO("Reached target floor. Waiting for door");
        }
        break;

    case ELEV_NAV_CHECKDOOR_OUT: //
        ROS_INFO("checkDoorStatus(door_status): %d", checkDoorStatus(door_status));

        if(checkDoorStatus(door_status))
        {
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
        navigation_cancel_pub.publish(true_boool_msg);
        nav_next_state = ELEV_NAV_IDLE;
        break;

    case ELEV_NAV_HOLD: //
        ROS_INFO("Navigation on hold to receive feedback");
        if (isPositionValid()) // check we have a valid position
            nav_next_state = ELEV_NAV_BUSY;
        break;

    case ELEV_NAV_PAUSED: // this state is reached via a callback
        if(nav_paused_req)
        {
            navigation_cancel_pub.publish(true_boool_msg);
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

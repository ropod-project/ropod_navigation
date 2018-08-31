#include "waypoint_navigation.h"

/*--------------------------------------------------------*/
WaypointNavigation::WaypointNavigation()
{
    true_bool_msg_.data = true;
    resetNavigation();
};

/*--------------------------------------------------------*/
WaypointNavigation::~WaypointNavigation()
{
    base_position.reset();
};

/*--------------------------------------------------------*/
void WaypointNavigation::startNavigation(nav_msgs::Path Pathmsg)
{
    resetNavigation();
    route_busy = true;
    planned_route = Pathmsg;
    planned_route_size = (int)planned_route.poses.size();
    ROS_INFO("Route received: size: %d waypoints", planned_route_size);

    return;
}

/*--------------------------------------------------------*/
void WaypointNavigation::pauseNavigation()
{
    nav_paused_req = true;
    nav_state_bpause = nav_state;
    nav_next_state = WAYP_NAV_PAUSED;
    ROS_INFO("Navigation paused");
    return;
}

/*--------------------------------------------------------*/
void WaypointNavigation::resumeNavigation()
{
    nav_paused_req = false;
    if (nav_state_bpause == WAYP_NAV_BUSY)
        nav_next_state = WAYP_NAV_GOTOPOINT;
    else
        nav_next_state = nav_state_bpause;
    ROS_INFO("Navigation resumed");
    return;
}

/*--------------------------------------------------------*/
void WaypointNavigation::resetNavigation()
{
    base_position.reset();
    route_busy = false;
    nav_paused_req = false;
    waypoint_cnt = 0;
    nav_state = WAYP_NAV_IDLE;
    nav_next_state = WAYP_NAV_IDLE;
    nav_state_bpause = WAYP_NAV_IDLE;
}

/*--------------------------------------------------------*/
void WaypointNavigation::stopNavigation()
{
    base_position.reset();
    route_busy = false;
    waypoint_cnt = 0;
}

/*--------------------------------------------------------*/
bool WaypointNavigation::isPositionValid()
{
    if (base_position)
        return true;
    else
        return false;
}

/*--------------------------------------------------------*/
bool WaypointNavigation::isWaypointAchieved(const geometry_msgs::PoseStamped &goal)
{
    tf::Quaternion qtemp = tf::Quaternion(base_position->pose.orientation.x, base_position->pose.orientation.y, 
                                          base_position->pose.orientation.z, base_position->pose.orientation.w);
    tf::Vector3 v3temp = tf::Vector3(base_position->pose.position.x, base_position->pose.position.y, 0.0);
    base_positiontf_ = tf::Transform(qtemp, v3temp);
    qtemp = tf::Quaternion(goal.pose.orientation.x,
                           goal.pose.orientation.y,
                           goal.pose.orientation.z,
                           goal.pose.orientation.w);
    v3temp = tf::Vector3(goal.pose.position.x,
                         goal.pose.position.y,
                         0.0);
    waypoint_tf_ = tf::Transform(qtemp, v3temp);

    tf::Transform diff_tf = base_positiontf_.inverseTimes(waypoint_tf_);
    v3temp = diff_tf.getOrigin();
    qtemp = diff_tf.getRotation();

    if (waypoint_cnt < planned_route_size) 
    { // Check succced only by looking at distance to waypoint
        if (pow(v3temp.x(), 2) + pow(v3temp.y(), 2) < pow(WAYP_REACHED_DIST, 2)) 
        {
            ROS_INFO("Hooray, Intermediate waypoint passed");
            return true;
        }
    }
    else if (pow(v3temp.x(), 2) + pow(v3temp.y(), 2) < pow(GOAL_REACHED_DIST, 2)
             && fabs(qtemp.getAngle()) < GOAL_REACHED_ANG) 
    {
        //(ac_state == actionlib::SimpleClientGoalState::SUCCEEDED)
        return true;
    }
    else
        return false;
}

/*--------------------------------------------------------*/
bool WaypointNavigation::isLastWaypoint()
{
    if (waypoint_cnt >= planned_route_size)
        return true;
    else
        return false;
}

/*--------------------------------------------------------*/
geometry_msgs::Pose WaypointNavigation::getNextWaypoint(void)
{
    if (waypoint_cnt < planned_route_size) 
        waypoint_cnt = waypoint_cnt + 1;
    
    return planned_route.poses[waypoint_cnt - 1].pose;
}

/*--------------------------------------------------------*/
TaskFeedbackCcu WaypointNavigation::callNavigationStateMachine(ros::Publisher &nav_cancel_pub, maneuver_navigation::Goal &mn_goal_, maneuver_navigation::Feedback &mn_feedback_, bool& sendgoal)
{
  TaskFeedbackCcu tfb_nav;
  tfb_nav.wayp_n = waypoint_cnt;
  tfb_nav.fb_nav = NAV_BUSY;
  sendgoal = false;

    switch (nav_state) {
    case WAYP_NAV_IDLE: // No waypoints received yet.
      tfb_nav.fb_nav = NAV_IDLE;
        if (route_busy == true)
            nav_next_state = WAYP_NAV_GETPOINT;
        break;

    case WAYP_NAV_GETPOINT: //we'll send the the next goal to the robot
        mn_goal_.goal.pose = getNextWaypoint();	
        nav_next_state = WAYP_NAV_GOTOPOINT;
        break;
	
    case WAYP_NAV_GOTOPOINT:
        mn_goal_.goal.header.frame_id = "map";
        mn_goal_.goal.header.stamp = ros::Time::now();
        ROS_INFO("Sending goal");
        sendgoal = true;	
	tfb_nav.fb_nav = NAV_GOTOPOINT;
        nav_next_state = WAYP_NAV_BUSY;
        break;
	
    case WAYP_NAV_BUSY: //
        if (!isPositionValid()) 
        {
            nav_next_state = WAYP_NAV_HOLD;
            break;
        }
        if (isWaypointAchieved(mn_goal_.goal))
            nav_next_state = WAYP_NAV_WAYPOINT_DONE;
        break;
	
    case WAYP_NAV_WAYPOINT_DONE: //
        tfb_nav.fb_nav = NAV_WAYPOINT_DONE;
        if (isLastWaypoint())
            nav_next_state = WAYP_NAV_DONE;
        else
            nav_next_state = WAYP_NAV_GETPOINT;
	
        break;
	
    case WAYP_NAV_DONE: //
        tfb_nav.fb_nav = NAV_DONE;
        ROS_INFO("Navigation done");
        stopNavigation();
        nav_cancel_pub.publish(true_bool_msg_);
        nav_next_state = WAYP_NAV_IDLE;	
        break;
	
    case WAYP_NAV_HOLD: //
        ROS_INFO("Navigation on hold to receive feedback");
        if (isPositionValid()) // check we have a valid position
            nav_next_state = WAYP_NAV_BUSY;
        break;
	
    case WAYP_NAV_PAUSED: // this state is reached via a callback
        if (nav_paused_req) 
        {
            nav_cancel_pub.publish(true_bool_msg_);
            nav_paused_req = false;
        }
        break;

    default:
        nav_next_state = WAYP_NAV_IDLE;
    }

    nav_state = nav_next_state;

    return tfb_nav;
}



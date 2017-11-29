#include "waypoint_navigation.h"

/*--------------------------------------------------------*/
Waypoint_navigation::Waypoint_navigation()
{

    reset_navigation();
};

/*--------------------------------------------------------*/
Waypoint_navigation::~Waypoint_navigation()
{
    base_position_.reset();
};

/*--------------------------------------------------------*/
void Waypoint_navigation::start_navigation(nav_msgs::Path Pathmsg)
{
    reset_navigation();
    route_busy = true;
    planned_route = Pathmsg;
    planned_route_size = (int)planned_route.poses.size();
    ROS_INFO("Route received: size: %d waypoints", planned_route_size);

    return;
}

/*--------------------------------------------------------*/
void Waypoint_navigation::pause_navigation()
{
    nav_paused_req = true;
    nav_state_bpause = nav_state;
    nav_next_state = WAYP_NAV_PAUSED;
    ROS_INFO("Navigation paused");
    return;
}

/*--------------------------------------------------------*/
void Waypoint_navigation::resume_navigation()
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
void Waypoint_navigation::reset_navigation()
{
    base_position_.reset();
    route_busy = false;
    nav_paused_req = false;
    waypoint_cnt = 0;
    nav_state = WAYP_NAV_IDLE;
    nav_next_state = WAYP_NAV_IDLE;
    nav_state_bpause = WAYP_NAV_IDLE;
}

/*--------------------------------------------------------*/
void Waypoint_navigation::stop_navigation()
{
    base_position_.reset();
    route_busy = false;
    waypoint_cnt = 0;
}

/*--------------------------------------------------------*/
bool Waypoint_navigation::is_position_valid()
{
    if (base_position_)
        return true;
    else
        return false;
}

/*--------------------------------------------------------*/
bool Waypoint_navigation::is_waypoint_achieved()
{
    tf::Quaternion qtemp = tf::Quaternion(base_position_->feedback.base_position.pose.orientation.x,
                                          base_position_->feedback.base_position.pose.orientation.y,
                                          base_position_->feedback.base_position.pose.orientation.z,
                                          base_position_->feedback.base_position.pose.orientation.w);
    tf::Vector3 v3temp = tf::Vector3(base_position_->feedback.base_position.pose.position.x,
                                     base_position_->feedback.base_position.pose.position.y,
                                     0.0);
    base_position_tf_ = tf::Transform(qtemp, v3temp);
    qtemp = tf::Quaternion(goal.target_pose.pose.orientation.x,
                           goal.target_pose.pose.orientation.y,
                           goal.target_pose.pose.orientation.z,
                           goal.target_pose.pose.orientation.w);
    v3temp = tf::Vector3(goal.target_pose.pose.position.x,
                         goal.target_pose.pose.position.y,
                         0.0);
    waypoint_tf_ = tf::Transform(qtemp, v3temp);

    tf::Transform diff_tf = base_position_tf_.inverseTimes(waypoint_tf_);
    v3temp = diff_tf.getOrigin();
    qtemp = diff_tf.getRotation();

    if (waypoint_cnt < planned_route_size) { // Check succced only by looking at distance to waypoint
        if (pow(v3temp.x(), 2) + pow(v3temp.y(), 2) < pow(WAYP_REACHED_DIST, 2)) {
            ROS_INFO("Hooray, Intermediate waypoint passed");
            return true;
        }
    }
    else if (pow(v3temp.x(), 2) + pow(v3temp.y(), 2) < pow(GOAL_REACHED_DIST, 2)
             && fabs(qtemp.getAngle()) < GOAL_REACHED_ANG) {
        //(ac_state == actionlib::SimpleClientGoalState::SUCCEEDED)

        return true;
    }
    else
        return false;
}

/*--------------------------------------------------------*/
bool Waypoint_navigation::is_last_waypoint()
{
    if (waypoint_cnt >= planned_route_size)
        return true;
    else
        return false;
}

/*--------------------------------------------------------*/
geometry_msgs::Pose Waypoint_navigation::get_next_point(void)
{
    if (waypoint_cnt < planned_route_size) {
        waypoint_cnt = waypoint_cnt + 1;
    }
    return planned_route.poses[waypoint_cnt - 1].pose;
}

/*--------------------------------------------------------*/
task_fb_ccu Waypoint_navigation::navigation_state_machine(ros::Publisher &movbase_cancel_pub, move_base_msgs::MoveBaseGoal* goal_ptr, bool& sendgoal)
{
  task_fb_ccu tfb_nav;
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
        goal.target_pose.pose = get_next_point();	
        nav_next_state = WAYP_NAV_GOTOPOINT;
        break;
	
    case WAYP_NAV_GOTOPOINT:
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        ROS_INFO("Sending goal");
        sendgoal = true;	
	tfb_nav.fb_nav = NAV_GOTOPOINT;
        nav_next_state = WAYP_NAV_BUSY;
        break;
	
    case WAYP_NAV_BUSY: //
        if (!is_position_valid()) {
            nav_next_state = WAYP_NAV_HOLD;
            break;
        }
        if (is_waypoint_achieved())
            nav_next_state = WAYP_NAV_WAYPOINT_DONE;
        break;
	
    case WAYP_NAV_WAYPOINT_DONE: //
        tfb_nav.fb_nav = NAV_WAYPOINT_DONE;
        if (is_last_waypoint())
            nav_next_state = WAYP_NAV_DONE;
        else
            nav_next_state = WAYP_NAV_GETPOINT;
	
        break;
	
    case WAYP_NAV_DONE: //
        tfb_nav.fb_nav = NAV_DONE;
        ROS_INFO("Navigation done");
        stop_navigation();
        movbase_cancel_pub.publish(emptyGoalID);
        nav_next_state = WAYP_NAV_IDLE;	
        break;
	
    case WAYP_NAV_HOLD: //
        ROS_INFO("Navigation on hold to receive feedback");
        if (is_position_valid()) // check we have a valid position
            nav_next_state = WAYP_NAV_BUSY;
        break;
	
    case WAYP_NAV_PAUSED: // this state is reached via a callback
        if (nav_paused_req) {
            movbase_cancel_pub.publish(emptyGoalID);
            nav_paused_req = false;
        }
        break;

    default:
        nav_next_state = WAYP_NAV_IDLE;
    }

    nav_state = nav_next_state;

    *goal_ptr = goal;
    return tfb_nav;
}



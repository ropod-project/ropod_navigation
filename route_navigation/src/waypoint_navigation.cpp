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

void WaypointNavigation::startNavigation(std::vector<ropod_ros_msgs::NavigationArea>& navigation_areas_plan_msg)
{

    planned_full_route = navigation_areas_plan_msg;   
    ROS_INFO("Route received: size: %d areas", (int)planned_full_route.size());    
    if( planned_full_route.size() > 0 )
    {
        curr_nav_area = planned_full_route.begin();
    }
    else
    {
        ROS_ERROR("EMPTY PLAN RECEIVED");
        return;
    }
    
    if ( curr_nav_area->waypoints.size() > 0 )
    {
        
        curr_nav_waypoint = curr_nav_area->waypoints.begin();
        resetNavigation();
        route_busy = true;                
    }
    else
    {
        ROS_ERROR("FIRST AREA SHOULD CONTAIN AT LEAST ONE WAYPOINT");
        return;
    }

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
    change_of_area = false;
    last_area_loaded = false;
    last_waypoint_loaded = false;
    perform_initial_rotation = false;
    // Setd efault goal configuration
    nav_configuration.precise_goal = true;
    nav_configuration.use_line_planner = false;
    
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

    if ( !isLastWaypoint() ) 
    { // Check succced only by looking at distance to waypoint
        if (pow(v3temp.x(), 2) + pow(v3temp.y(), 2) < pow(WAYP_REACHED_DIST, 2) && (!perform_initial_rotation || fabs(qtemp.getAngle() < GOAL_REACHED_ANG ) ) )
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
    return last_waypoint_loaded;
}


bool WaypointNavigation::getNextWaypoint(maneuver_navigation::Goal &mn_goal)
{    

    
    
    if(last_waypoint_loaded)
        return false;
    
    if (curr_nav_area == planned_full_route.begin())
    {
        ROS_INFO("Load first area");        
        if ( curr_nav_waypoint == curr_nav_area->waypoints.begin())
        {
            // First waypoint is treated in a special way, we first turn in place towards the corridor orientation within desired orientation threshold.
            // Later we can create a maneuver that takes care of that
            // TODO: Important. Docking navigation should put a waypoint in front to avoid colliding when turning.In any case, when in a rail, the docking navigation should take care
            // of leaving the docking area in a proper way. Or this can be put as part of the semantic behavior around those areas.
            tf::Pose base_position_TF;
            tf::Pose goal_TF;
            tf::poseMsgToTF(base_position->pose,base_position_TF);
            tf::poseMsgToTF(curr_nav_waypoint->waypoint_pose,goal_TF);
            double yaw_diff = std::atan2(curr_nav_waypoint->waypoint_pose.position.y - base_position->pose.position.y, curr_nav_waypoint->waypoint_pose.position.x - base_position->pose.position.x);
            tf::Quaternion diff_quat;
            diff_quat.setRPY(0.0,0.0,yaw_diff);
            double base_yaw = tf::getYaw(base_position_TF.getRotation());            
            
            if(fabs( base_yaw-yaw_diff) > GOAL_REACHED_ANG)
            {
                tf::Quaternion quat_tf_pose;
                quat_tf_pose.setRPY(0.0,0.0,yaw_diff);                
                mn_goal.start.pose = base_position->pose;
                mn_goal.goal.pose.position = base_position->pose.position;                                                          
                mn_goal.goal.pose.orientation.w = quat_tf_pose.getW();
                mn_goal.goal.pose.orientation.x = quat_tf_pose.getX();
                mn_goal.goal.pose.orientation.y = quat_tf_pose.getY();
                mn_goal.goal.pose.orientation.z = quat_tf_pose.getZ();
                perform_initial_rotation = true;
                return true;
                
            }
            else
            {
                perform_initial_rotation = false;
                mn_goal.goal.pose = curr_nav_waypoint->waypoint_pose;
                mn_goal.start.pose = base_position->pose;             
            }                        
            
        }
        else
        {
            mn_goal.goal.pose = curr_nav_waypoint->waypoint_pose;
            mn_goal.start.pose = base_position->pose;          
        }
        
    }
    else
    {
        if ( change_of_area )
        {
            mn_goal.goal.pose = curr_nav_waypoint->waypoint_pose;
            mn_goal.start.pose.position = base_position->pose.position;
            mn_goal.start.pose.orientation = prev_nav_waypoint->waypoint_pose.orientation;            
        }
        else
        {
            mn_goal.goal.pose = curr_nav_waypoint->waypoint_pose;
            mn_goal.start.pose = base_position->pose;                  
        }
    }
    
    prev_nav_waypoint =  curr_nav_waypoint;  
    curr_nav_waypoint++;
    waypoint_cnt ++;
    change_of_area = false;
    bool next_area_is_valid = false;
    if ( curr_nav_waypoint == curr_nav_area->waypoints.end() )
    {
        if (last_area_loaded )
        {
            last_waypoint_loaded = true;
            return true;
        }
        // change of area, realign by using pose from previous corridor
        change_of_area = true;
        while (true)
        {
            // Look for next non-empty corridor
            curr_nav_area++;
            if ( curr_nav_area < planned_full_route.end())
            {
                if( curr_nav_area->type != "door" && curr_nav_area->waypoints.size() != 0)
                {
                    next_area_is_valid = true;
                    curr_nav_waypoint = curr_nav_area->waypoints.begin();
                    break;
                }                
            }
            else
            {
                last_area_loaded = true;  
                last_waypoint_loaded = !next_area_is_valid;
                break;
            }
        }            
    }
       
    return true;
       
    
}

/*--------------------------------------------------------*/
TaskFeedbackCcu WaypointNavigation::callNavigationStateMachine(ros::Publisher &nav_cancel_pub, maneuver_navigation::Goal &mn_goal, maneuver_navigation::Feedback &mn_feedback_, bool& sendgoal)
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
        //mn_goal.goal.pose = getNextWaypoint();	
        getNextWaypoint(mn_goal);        
        mn_goal.goal.header.frame_id = "map";
        mn_goal.goal.header.stamp = ros::Time::now();
        mn_goal.start.header.frame_id = "map";
        mn_goal.start.header.stamp = ros::Time::now();
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
        if (isWaypointAchieved(mn_goal.goal))
        {
            tfb_nav.fb_nav = NAV_WAYPOINT_DONE;
            if (isLastWaypoint())
                nav_next_state = WAYP_NAV_DONE;
            else
                nav_next_state = WAYP_NAV_GETPOINT;         
        }        
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



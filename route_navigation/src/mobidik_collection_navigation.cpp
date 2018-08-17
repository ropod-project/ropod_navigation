#include "mobidik_collection_navigation.h"

/*--------------------------------------------------------*/
MobidikCollection::MobidikCollection()
{

};

/*--------------------------------------------------------*/
MobidikCollection::~MobidikCollection()
{
    base_position_.reset();
};


MobidikCollection::queryMobidik() 
// TODO via ED WM if data-association solved, or for now, as there is no data-association, remove all the previous entities described by the laser and create new ones
{
    
};

MobidikCollection::setMobidikPosition() // Store mobidik in WM as it might not be detected anymore when the robot rotates
{
    
};

MobidikCollection::getSetpointInFrontOfMobidik ( const ed::WorldModel& world )
{
    for ( ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it )
    {
        const ed::EntityConstPtr& e = *it;
        
        // check which area is the closest (calculate centerpoint of polygon?) -> this is assumed to be the area at which the mobidik should be collected
        // or is this information known? How is this information received?
    }

};


/*--------------------------------------------------------*/
// void MobidikCollection::startNavigation(wm::Elevator &elevator,nav_msgs::Path Pathmsg)
// {
//     resetNavigation();
//     route_busy_ = true;
//     // if we get elevator poses from ropod_wm_mediator, use them
//     if (Pathmsg.poses.size() == 3)
//     {
//         elevator.setInsideElevatorPose(wm::getWMPose(Pathmsg.poses[0]), wm::getWMPose(Pathmsg.poses[1]));
//         elevator.setOutsideElevatorPose(wm::getWMPose(Pathmsg.poses[2]));
//         ROS_INFO("Poses from elevator message assigned: %f", elevator.wayp1_elevator.position.x);
//     }
//     ROS_INFO("start navigation trough elevator");
//     return;
// }

/*--------------------------------------------------------*/
void MobidikCollection::pauseNavigation()
{
    nav_paused_req_ = true;
    nav_state_bpause_ = nav_state_;
    nav_next_state_ = MOBID_COLL_NAV_PAUSED;
    ROS_INFO("Navigation paused");
    return;
};

/*--------------------------------------------------------*/
void MobidikCollection::resumeNavigation()
{
    nav_paused_req_ = false;
    if(nav_state_bpause_ == MOBID_COLL_NAV_BUSY)
        nav_next_state_ = MOBID_COLL_NAV_GOTOPOINT;
    else
        nav_next_state_ = nav_state_bpause_;
    ROS_INFO("Navigation resumed");
    return;
}

/*--------------------------------------------------------*/
void MobidikCollection::resetNavigation()
{
    base_position_.reset();
    route_busy_ = false;
    nav_paused_req_ = false;
    waypoint_cnt_ = 0;
    nav_state_ = MOBID_COLL_NAV_IDLE;
    nav_next_state_ = MOBID_COLL_NAV_IDLE;
    nav_state_bpause_ = MOBID_COLL_NAV_IDLE;
    nav_next_state_wp_ = MOBID_COLL_NAV_IDLE;
    ROS_INFO("Navigation reset");
}

/*--------------------------------------------------------*/
void MobidikCollection::stopNavigation()
{
    base_position_.reset();
    route_busy_ = false;
    waypoint_cnt_ = 0;
    ROS_INFO("Navigation stopped");
}

/*--------------------------------------------------------*/
bool MobidikCollection::isPositionValid()
{
    if(base_position_)
        return true;
    else
        return false;
}

/*--------------------------------------------------------*/
bool MobidikCollection::isWaypointAchieved()
{

    // In elevator we checked whether teh specific position is reached.
    tf::Quaternion qtemp = tf::Quaternion(base_position_->feedback.base_position.pose.orientation.x, base_position_->feedback.base_position.pose.orientation.y,
                                          base_position_->feedback.base_position.pose.orientation.z,base_position_->feedback.base_position.pose.orientation.w);
    tf::Vector3 v3temp = tf::Vector3(base_position_->feedback.base_position.pose.position.x,base_position_->feedback.base_position.pose.position.y, 0.0);
    base_positiontf_ = tf::Transform( qtemp, v3temp);
    qtemp = tf::Quaternion(goal_.target_pose.pose.orientation.x, goal_.target_pose.pose.orientation.y,
                           goal_.target_pose.pose.orientation.z,goal_.target_pose.pose.orientation.w);
    v3temp = tf::Vector3(goal_.target_pose.pose.position.x,goal_.target_pose.pose.position.y, 0.0);
    waypoint_tf_ = tf::Transform( qtemp, v3temp);

    tf::Transform diff_tf = base_positiontf_.inverseTimes(waypoint_tf_);
    v3temp = diff_tf.getOrigin();
    qtemp = diff_tf.getRotation();
    
    if (pow( v3temp.x(),2) + pow(v3temp.y(),2) < pow(GOAL_MOBID_COLL_REACHED_DIST,2)
            && fabs(qtemp.getAngle()) < GOAL_MOBID_REACHED_ANG)
        return true;
    else
        return false;
}

/*--------------------------------------------------------*/
TaskFeedbackCcu MobidikCollection::callNavigationStateMachine(ros::Publisher &movbase_cancel_pub, move_base_msgs::MoveBaseGoal* goal_ptr, bool& sendgoal)
{
    TaskFeedbackCcu tfb_nav;
    tfb_nav.wayp_n = waypoint_cnt_;
    tfb_nav.fb_nav = NAV_BUSY;
    sendgoal = false;

    switch(nav_state_)
    { 
            
    case MOBID_COLL_NAV_IDLE: // No waypoints received yet.
        tfb_nav.fb_nav = NAV_IDLE;
//         if (route_busy_ == true)
//         {
//              nav_next_state_ = MOBID_COLL_NAV_GOTOPOINT;
//             ROS_INFO("Waiting for door");
//         }
        break;
        
    case MOBID_COLL_FIND_MOBIDIK:
            if( queryMobidik() ) // Assumption: there is only 1 mobidik available at the moment
            {           
                    setMobidikPosition(); // set the position of the mobidik in the WM
                    goal_.target_pose = getSetpointInFrontOfMobidik();
                    
                    nav_next_state_wp_ = MOBID_COLL_ROTATE;
                    nav_next_state_ = MOBID_COLL_NAV_GOTOPOINT;
                    ROS_INFO("Mobidik Collection: Mobidik found");
            }
            
            else 
            {
                    ROS_WARN("Mobidik Collection: No mobidik found");
            }
        break;
        
    case MOBID_COLL_ROTATE:
            goal_.target_pose = getSetpointInFrontOfMobidik(); // + pi rad ratotion!
            nav_next_state_wp_ = MOBID_COLL_NAV_CONNECTING;
            nav_next_state_ = MOBID_COLL_NAV_GOTOPOINT;
         break;
         
    case MOBID_COLL_NAV_CONNECTING: 
            // publish this state on a topic and send a low velocity such that the mpc-controller can do its job.
            // Read the force measured by the bumper, and check when it touches the mobidik
            
            bool touched = false;
            
            if (touched)
                  nav_next_state_  = MOBID_COLL_NAV_COUPLING;
            
         break;
         
    case MOBID_COLL_NAV_COUPLING:
            // couple mobidik manually and wait for signal;
            bool signal = false;
            
            if(signal)
            {
                    MOBID_COLL_NAV_DONE;
            }
         break;

    case MOBID_COLL_NAV_GOTOPOINT:
        goal_.target_pose.header.frame_id = "map";
        goal_.target_pose.header.stamp = ros::Time::now();
        ROS_INFO("Sending goal");
        sendgoal = true;
        tfb_nav.fb_nav = NAV_GOTOPOINT;
        nav_next_state_ = MOBID_COLL_NAV_BUSY;
        break;
         
    case MOBID_COLL_NAV_BUSY: 
        if (!isPositionValid())
        {
            nav_next_state_ = MOBID_COLL_NAV_HOLD;
            break;
        }
        if (isWaypointAchieved())
            nav_next_state_ = MOBID_COLL_NAV_WAYPOINT_DONE;

        break;

    case MOBID_COLL_NAV_WAYPOINT_DONE: //
        tfb_nav.fb_nav = NAV_WAYPOINT_DONE; // is this still valid when we integrate an extra waypoint for this mobidik-collection? What should we send as feedback?
        nav_next_state_ = nav_next_state_wp_;

        break;
        
    case MOBID_COLL_NAV_DONE: //
        ROS_INFO("Navigation done");
        tfb_nav.fb_nav = NAV_DONE;
        stopNavigation();
        movbase_cancel_pub.publish(emptyGoalID_);
        nav_next_state_ = MOBID_COLL_NAV_IDLE;
        break;

    case MOBID_COLL_NAV_HOLD: //
        ROS_INFO("Navigation on hold to receive feedback");
        if (isPositionValid()) // check we have a valid position
            nav_next_state_ = MOBID_COLL_NAV_BUSY;
        break;

    case MOBID_COLL_NAV_PAUSED: // this state is reached via a callback
        if(nav_paused_req_)
        {
            movbase_cancel_pub.publish(emptyGoalID_);
            nav_paused_req_ = false;
        }
        break;

    default:
        nav_next_state_ = MOBID_COLL_NAV_IDLE;
    }

    nav_state_ = nav_next_state_;

    *goal_ptr = goal_;

    return tfb_nav;
}

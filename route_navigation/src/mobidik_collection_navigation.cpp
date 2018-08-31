#include "mobidik_collection_navigation.h"
#include <ed_gui_server/EntityInfos.h>

/*--------------------------------------------------------*/
MobidikCollection::MobidikCollection()
{
    true_bool_msg_.data = true;
};

/*--------------------------------------------------------*/
MobidikCollection::~MobidikCollection()
{
    base_position_.reset();
};


bool MobidikCollection::getMobidik(visualization_msgs::MarkerArray markerArray, visualization_msgs::Marker *marker) 
// TODO via ED WM (if data-association solved!?), or for now, as there is no data-association, remove all the previous entities described by the laser and create new ones
// assumption: there is only 1 object which can be recognized as a mobidik, so the first one is taken now
{
        std::cout << "MarkerArray size = " <<  markerArray.markers.size() << std::endl;
        for (unsigned int ii = 0; ii < markerArray.markers.size(); ii++)
        {
                
                visualization_msgs::Marker markerToCheck = markerArray.markers.at(ii);
                if (markerToCheck.ns == "Mobidik")
                {
//                         std::cout << "getMobidik-function: mobidik found" << std::endl;
//                                                 
//                         std::cout << "Mobidik Position = \n" <<
//                      markerToCheck.pose.position.x << " \n " <<
//                      markerToCheck.pose.position.y << " \n " <<
//                      markerToCheck.pose.position.z << " \n " <<
//                      markerToCheck.pose.orientation.x << " \n " << 
//                      markerToCheck.pose.orientation.y << " \n " <<
//                      markerToCheck.pose.orientation.z << " \n " <<
//                      markerToCheck.pose.orientation.w << " \n " <<
//                     std::endl;
                        *marker = markerToCheck;
                        return true;
                }                    
        }       
};

template <class T>
void MobidikCollection::wrap ( T *angle )
{
    *angle -= 2*M_PI * std::floor ( *angle * ( 1 / ( 2*M_PI )) );
}

void MobidikCollection::setMobidikPosition ( const ed::WorldModel& world,ed::UpdateRequest& req,  std::string mobidikAreaID, visualization_msgs::Marker mobidikMarker, ed::UUID* id, visualization_msgs::Marker* points ) // Store mobidik in ED as it might not be detected anymore when the robot rotates
{
    // Area where the mobidik will be collected is assumed to be known
    geo::Pose3D mobidikPose;

    visualization_msgs::Marker mobidikPoseMarker = mobidikMarker;
    double mobidikWidth = mobidikPoseMarker.scale.x;
    double mobidikLength = mobidikPoseMarker.scale.y;


    tf::Quaternion q ( mobidikPoseMarker.pose.orientation.x, mobidikPoseMarker.pose.orientation.y, mobidikPoseMarker.pose.orientation.z, mobidikPoseMarker.pose.orientation.w );
    tf::Matrix3x3 m ( q );
    double MD_roll, MD_pitch, MD_yaw; // MD = mobidik
    m.getRPY ( MD_roll, MD_pitch, MD_yaw );
    wrap ( &MD_yaw );

    double xPos = mobidikPoseMarker.pose.position.x;
    double yPos = mobidikPoseMarker.pose.position.y;

    geo::Vec3d origin ( xPos, yPos, mobidikPoseMarker.pose.position.z );
    mobidikPose.setOrigin ( origin );

    for ( ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it )
    {
        const ed::EntityConstPtr& e = *it;

        std::string orientationWPID = "orient_wp_" +  mobidikAreaID;

        if ( orientationWPID.compare ( e.get()->id().str() ) == 0 ) // function returns 0 if strings are equal
        {
            geo::Pose3D poseWP = e.get()->pose();
            geo::Quaternion rotationWP = poseWP.getQuaternion();

            tf::Quaternion q ( rotationWP.getX(), rotationWP.getY(), rotationWP.getZ(), rotationWP.getW() );
            tf::Matrix3x3 matrix ( q );
            double WP_roll, WP_pitch, WP_yaw;
            matrix.getRPY ( WP_roll, WP_pitch, WP_yaw );
            wrap ( &WP_yaw );

            std::cout << "yaw = " << WP_yaw << std::endl;

            // find quadrant closest to the orientation of the orienation
            geo::real diffAngleMin = INFINITY;
            for ( unsigned int ii = 0; ii < 4; ii++ )
            {
                double angle = MD_yaw + ii*M_PI_2;
                wrap ( &angle );

                geo::real diffAngle = std::min ( std::fabs ( angle - WP_yaw ), std::fabs ( angle + 2*M_PI - WP_yaw ) );
                if ( diffAngle < diffAngleMin )
                {
                    diffAngleMin = diffAngle;
                    MD_yaw = angle;
                }
            }

            geo::Mat3 rotation;
            rotation.setRPY ( MD_roll, MD_pitch, MD_yaw );
            mobidikPose.setBasis ( rotation );

            ed::ConvexHull chull;
            for ( unsigned int ii = 0; ii < 4; ii++ )
            {
                float rotation = MD_yaw + M_PI/4 + M_PI/2*ii;

                float length = std::sqrt ( std::pow ( 0.5*mobidikWidth, 2.0 ) + std::pow ( 0.5*mobidikLength, 2.0 ) );
                geo::Vec2f point ( mobidikPose.getOrigin().getX() + length*cos ( rotation ), mobidikPose.getOrigin().getY() + length*sin ( rotation ) );
                chull.points.push_back ( point );

                geometry_msgs::Point p;
                p.x = point.x;
                p.y = point.y;
                p.z = 0.0;

                points->points.push_back ( p );
            }

            *id = ed::Entity::generateID().str() + "-Mobidik";  // Generate unique ID
            req.setExistenceProbability ( *id, 1.0 ); // TODO magic number
            req.setConvexHullNew ( *id, chull, mobidikPose, mobidikMarker.header.stamp.toSec(), mobidikMarker.header.frame_id );

            ROS_INFO ( "Mobidik Position set" );
            return;
        }

    }

    return;

};                      

void MobidikCollection::getSetpointInFrontOfMobidik ( const ed::WorldModel& world, ed::UUID mobidikID, geo::Pose3D *setpoint, visualization_msgs::Marker* points )
{
    for ( ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it )
    {
        const ed::EntityConstPtr& e = *it;

        if ( mobidikID.str().compare ( e.get()->id().str() )  == 0 )
        {
            geo::Pose3D mobidikPose = e.get()->pose();
            geo::Quaternion rotation = mobidikPose.getQuaternion();
            tf::Quaternion q ( rotation.getX(), rotation.getY(), rotation.getZ(), rotation.getW() );
            tf::Matrix3x3 matrix ( q );
            double roll, pitch, yaw;
            matrix.getRPY ( roll, pitch, yaw );

            float dist = 0.5* ( ROPOD_LENGTH + MOBIDIK_LENGTH ) + DIST_IN_FRONT_OFF_MOBID;
            *setpoint = mobidikPose;

            geo::Vec3d origin ( mobidikPose.getOrigin().getX() + dist*cos ( yaw ), mobidikPose.getOrigin().getY() + dist*sin ( yaw ), 0 );
            setpoint->setOrigin ( origin );

            geometry_msgs::Point p;
            p.x = setpoint->getOrigin().getX();
            p.y = setpoint->getOrigin().getY();
            p.z = 0.0;

            points->points.push_back ( p );

            return;
        }
    }

    return;
   
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
    nav_state_release_ = MOBID_REL_NAV_IDLE;
    nav_next_state_release_ = MOBID_REL_NAV_IDLE;
    nav_next_state_wp_release_ = MOBID_REL_NAV_IDLE;
    
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

void MobidikCollection::initAvgWrench(geometry_msgs::WrenchStamped *wrench)
{
        wrench->wrench.force.x = INFINITY;
        wrench->wrench.force.y = INFINITY;
        wrench->wrench.force.z = INFINITY;
        wrench->wrench.force.x = INFINITY;
        wrench->wrench.torque.y = INFINITY;
        wrench->wrench.torque.z = INFINITY;
}

void MobidikCollection::initNavState()
{
        nav_state_ = MOBID_COLL_FIND_MOBIDIK;
        avgWrenchesDetermined_ = false;
        initAvgWrench(&avgWrenches_.front);
        initAvgWrench(&avgWrenches_.left);
        initAvgWrench(&avgWrenches_.back);
        initAvgWrench(&avgWrenches_.right);
        std::cout << "Nave state initialised at " << MOBID_COLL_FIND_MOBIDIK << std::endl;
}

geometry_msgs::WrenchStamped MobidikCollection::determineAvgWrench(std::vector<geometry_msgs::WrenchStamped> wrenchVector)
{
        float sumForcex = 0.0, sumForcey = 0.0, sumForcez = 0.0, sumTorquex = 0.0, sumTorquey = 0.0, sumTorquez = 0.0;
            for ( unsigned int ii = 0; ii < wrenchVector.size(); ii++ )
            {
                sumForcex += wrenchVector[ii].wrench.force.x;
                sumForcey += wrenchVector[ii].wrench.force.y;
                sumForcez += wrenchVector[ii].wrench.force.z;
                sumTorquex += wrenchVector[ii].wrench.force.x;
                sumTorquey += wrenchVector[ii].wrench.force.y;
                sumTorquez += wrenchVector[ii].wrench.force.z;
            }

            geometry_msgs::WrenchStamped avgWrench;
            avgWrench.wrench.force.x = sumForcex / wrenchVector.size();
            avgWrench.wrench.force.y = sumForcey / wrenchVector.size();
            avgWrench.wrench.force.z = sumForcez / wrenchVector.size();
            avgWrench.wrench.torque.x = sumTorquex / wrenchVector.size();
            avgWrench.wrench.torque.y = sumTorquey / wrenchVector.size();
            avgWrench.wrench.torque.z = sumTorquez/ wrenchVector.size();
            
            return avgWrench;            
}

void MobidikCollection::determineAvgWrenches()
{
        std::vector<geometry_msgs::WrenchStamped> wrenchesFront, wrenchesLeft, wrenchesBack, wrenchesRight;
        for ( unsigned int ii = 0; ii < bumperWrenchesVector_.size(); ii++ )
        {
                wrenchesFront.push_back( bumperWrenchesVector_[ii].front );
                wrenchesLeft.push_back( bumperWrenchesVector_[ii].left );
                wrenchesBack.push_back( bumperWrenchesVector_[ii].back );
                wrenchesRight.push_back( bumperWrenchesVector_[ii].right );
        }
        
        avgWrenches_.front = determineAvgWrench( wrenchesFront );
        avgWrenches_.left = determineAvgWrench( wrenchesLeft );
        avgWrenches_.back = determineAvgWrench( wrenchesBack );
        avgWrenches_.right = determineAvgWrench( wrenchesRight );
        
        avgWrenchesDetermined_ = true;
}

void MobidikCollection::initRelState()
{
        nav_state_release_ = MOBID_REL_GET_SETPOINT_FRONT;
        std::cout << "Nave state initialised at " << MOBID_REL_GET_SETPOINT_FRONT << std::endl;
}

/*--------------------------------------------------------*/
bool MobidikCollection::isWaypointAchieved()
{

    // In elevator we checked whether teh specific position is reached.
    tf::Quaternion qtemp = tf::Quaternion(base_position_->pose.orientation.x, base_position_->pose.orientation.y, 
                                          base_position_->pose.orientation.z, base_position_->pose.orientation.w);
    tf::Vector3 v3temp = tf::Vector3(base_position_->pose.position.x, base_position_->pose.position.y, 0.0);
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
TaskFeedbackCcu MobidikCollection::callNavigationStateMachine(ros::Publisher &movbase_cancel_pub, move_base_msgs::MoveBaseGoal* goal_ptr, bool& sendgoal, visualization_msgs::MarkerArray markerArray, std::string areaID, const ed::WorldModel& world, ed::UpdateRequest& req, visualization_msgs::MarkerArray *markerArraytest, std_msgs::UInt16* controlMode, ros::Publisher &cmv_vel_pub, ropodNavigation::wrenches bumperWrenches)
{
    TaskFeedbackCcu tfb_nav;
    tfb_nav.wayp_n = waypoint_cnt_;
    tfb_nav.fb_nav = NAV_BUSY;
    sendgoal = false;
    visualization_msgs::Marker marker;
    
    tf::Quaternion q;
    geometry_msgs::Twist output_vel;
    
    float avgForce, avgTorque;
    bool touched, forceCheck, torqueCheck;
    
    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/map";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    
     // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.1;
    points.scale.y = 0.1;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    switch(nav_state_)
    { 
            
    case MOBID_COLL_NAV_IDLE: // No waypoints received yet.
        tfb_nav.fb_nav = NAV_IDLE;
        ROS_INFO("NAV_IDLE");
//         if (route_busy_ == true)
//         {
//              nav_next_state_ = MOBID_COLL_NAV_GOTOPOINT;
//             ROS_INFO("Waiting for door");
//         }
        break;
        
    case MOBID_COLL_FIND_MOBIDIK:
            ROS_INFO("MOBID_COLL_FIND_MOBIDIK");

            if( getMobidik(markerArray,  &marker)  ) // Assumption: there is only 1 mobidik available at the moment
            {           
                    setMobidikPosition ( world, req, areaID, marker, &MobidikID_ED_, &points ); 
                    std::cout << "Mobidik ID = " << MobidikID_ED_ << std::endl;
                    markerArraytest->markers.push_back( points );
                    nav_next_state_ = MOBID_COLL_FIND_SETPOINT_FRONT;
                    ROS_INFO ( "Mobidik Collection: Mobidik found" );
             }
            else 
            {
                    ROS_WARN("Mobidik Collection: No mobidik found"); // TODO Recovery behaviour
            }
                     
                     
         break;
         
    case MOBID_COLL_FIND_SETPOINT_FRONT:
            ROS_INFO("MOBID_COLL_FIND_SETPOINT_FRONT");
            std::cout << "Test1" << std::endl;
            getSetpointInFrontOfMobidik ( world, MobidikID_ED_, &setpoint_, &points);
            goal_.target_pose.pose.position.x = setpoint_.getOrigin().getX();
            goal_.target_pose.pose.position.y = setpoint_.getOrigin().getY();
            goal_.target_pose.pose.position.z = setpoint_.getOrigin().getZ();
            goal_.target_pose.pose.orientation.x = base_position_->pose.orientation.x; // TODO: segfault when no autonomous movement was made in reading the base_position
            goal_.target_pose.pose.orientation.y = base_position_->pose.orientation.y;
            goal_.target_pose.pose.orientation.z = base_position_->pose.orientation.z            ;
            goal_.target_pose.pose.orientation.w = base_position_->pose.orientation.w;
            markerArraytest->markers.push_back ( points );

            nav_next_state_wp_ = MOBID_COLL_ROTATE_IN_FRONT;
            nav_next_state_ = MOBID_COLL_NAV_GOTOPOINT;

            bumperWrenchesVector_.clear();
        break;
    case MOBID_COLL_ROTATE_IN_FRONT: // TODO
            goal_.target_pose.pose.position.x = setpoint_.getOrigin().getX();
            goal_.target_pose.pose.position.y = setpoint_.getOrigin().getY();
            goal_.target_pose.pose.position.z = setpoint_.getOrigin().getZ();            
            goal_.target_pose.pose.orientation.x = setpoint_.getQuaternion().getX();
            goal_.target_pose.pose.orientation.y = setpoint_.getQuaternion().getY();
            goal_.target_pose.pose.orientation.z = setpoint_.getQuaternion().getZ();
            goal_.target_pose.pose.orientation.w = setpoint_.getQuaternion().getW();
            
            nav_next_state_wp_ = MOBID_COLL_NAV_CONNECTING;
            nav_next_state_ = MOBID_COLL_NAV_GOTOPOINT;
            
        break;
        
    case MOBID_COLL_NAV_CONNECTING: // TODO
        ROS_INFO ( "MOBID_COLL_NAV_CONNECTING" );



        /*  TODO: REMOVE, ONLY FOR SIMULATION */
//             nav_next_state_  = MOBID_COLL_NAV_DONE;
//             bumperWrenchesVector_.clear();
//             break;
        /*************************************/

        touched = false;
        bumperWrenchesVector_.push_back ( bumperWrenches );
        if ( bumperWrenchesVector_.size() > N_COUNTS_WRENCHES ) // TODO update goal and how? How to let the software know there is actually no goal
        {

            bumperWrenchesVector_.erase ( bumperWrenchesVector_.begin() );

            if ( !avgWrenchesDetermined_ )
            {
                    determineAvgWrenches();

            }
            else
            {

                controlMode->data = ropodNavigation::LLC_DOCKING;
                output_vel.linear.x = -BACKWARD_VEL_DOCKING;
                cmv_vel_pub.publish ( output_vel );

                avgForce = 0.0;
                avgTorque = 0.0;
                for ( unsigned int ii = 0; ii < bumperWrenchesVector_.size(); ii++ ) // TODO can be more efficient by storing the sum
                {
                    avgForce += bumperWrenchesVector_[ii].back.wrench.force.x;
                    avgTorque += bumperWrenchesVector_[ii].back.wrench.torque.x;
                }

                avgForce /= bumperWrenchesVector_.size();
                avgTorque /= bumperWrenchesVector_.size();
                
                forceCheck = std::fabs ( avgForce - avgWrenches_.back.wrench.force.x ) > MIN_FORCE_TOUCHED;
                torqueCheck = std::fabs ( avgTorque - avgWrenches_.back.wrench.torque.x ) > MAX_TORQUE_TOUCHED;

                if ( forceCheck && torqueCheck )
                {
                    touched = true;
                }
            }
        }

        if ( touched )
        {
            nav_next_state_  = MOBID_COLL_NAV_COUPLING;
//              bumperWrenchesVector_.clear();
        }

        break;
         
    case MOBID_COLL_NAV_COUPLING: // TODO
        ROS_INFO ( "MOBID_COLL_NAV_COUPLING" );
        controlMode->data = ropodNavigation::LLC_VEL;
        // couple mobidik manually and wait for signal;

        bumperWrenchesVector_.push_back ( bumperWrenches );
        if ( bumperWrenchesVector_.size() > N_COUNTS_WRENCHES ) // TODO update goal and how? How to let the software know there is actually no goal
        {
            bumperWrenchesVector_.erase ( bumperWrenchesVector_.begin() ); // just to be sure
            if ( !avgWrenchesDetermined_ )
            {
                determineAvgWrenches();

            }
            else
            {

                avgForce = 0.0;
                for ( unsigned int ii = 0; ii < bumperWrenchesVector_.size(); ii++ )
                {
                    avgForce += bumperWrenchesVector_[ii].front.wrench.force.x;
                }

                avgForce /= bumperWrenchesVector_.size();
                
                forceCheck = std::fabs ( avgForce - avgWrenches_.front.wrench.force.x ) > MIN_FORCE_TOUCHED;

                if ( std::fabs ( avgForce ) > MIN_FORCE_TOUCHED )
                {
                    touched = true;
                }
            }
        }

        if ( touched )
        {
            nav_next_state_  = MOBID_COLL_NAV_DONE;
            bumperWrenchesVector_.clear();
        }
        break;

    case MOBID_COLL_NAV_GOTOPOINT:
            ROS_INFO("MOBID_COLL_NAV_GOTOPOINT");
        goal_.target_pose.header.frame_id = "map";
        goal_.target_pose.header.stamp = ros::Time::now();
        ROS_INFO("Sending goal");
        sendgoal = true;
        tfb_nav.fb_nav = NAV_GOTOPOINT;
        nav_next_state_ = MOBID_COLL_NAV_BUSY;
        break;
         
    case MOBID_COLL_NAV_BUSY:
            ROS_INFO("MOBID_COLL_NAV_BUSY");
        if (!isPositionValid())
        {
            nav_next_state_ = MOBID_COLL_NAV_HOLD;
            break;
        }
        if (isWaypointAchieved()) // TODO set the tolerance parameters of the base_local_planner_params equal to the tolerances used in this function -> at the ros param server
        {
                ROS_INFO("Waypoint achieved");
            nav_next_state_ = MOBID_COLL_NAV_WAYPOINT_DONE;
        }
        else 
        {
                ROS_INFO("Waypoint not achieved");
        }

        break;

    case MOBID_COLL_NAV_WAYPOINT_DONE: //
            ROS_INFO("MOBID_COLL_NAV_DONE");
        tfb_nav.fb_nav = NAV_WAYPOINT_DONE; // is this still valid when we integrate an extra waypoint for this mobidik-collection? What should we send as feedback?
        nav_next_state_ = nav_next_state_wp_;

        break;
        
    case MOBID_COLL_NAV_DONE: //
            ROS_INFO("MOBID_COLL_NAV_DONE");
        ROS_INFO("Navigation done");
        tfb_nav.fb_nav = NAV_DONE;
        stopNavigation();
        movbase_cancel_pub.publish(true_bool_msg_);
        nav_next_state_ = MOBID_COLL_NAV_IDLE;
        break;

    case MOBID_COLL_NAV_HOLD: //
            ROS_INFO("MOBID_COLL_NAV_HOLD");
        ROS_INFO("Navigation on hold to receive feedback");
        if (isPositionValid()) // check we have a valid position
            nav_next_state_ = MOBID_COLL_NAV_BUSY;
        break;

    case MOBID_COLL_NAV_PAUSED: // this state is reached via a callback
            ROS_INFO("MOBID_COLL_NAV_PAUSED");
        if(nav_paused_req_)
        {
            movbase_cancel_pub.publish(true_bool_msg_);
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


void MobidikCollection::getFinalMobidikPos ( const ed::WorldModel& world, std::string mobidikAreaID, geo::Pose3D *mobidikPosition, geo::Pose3D *disconnectSetpoint , geo::Pose3D *setpointInFrontOfMobidik )
{
    for ( ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it )
    {
        const ed::EntityConstPtr& e = *it;

        if ( mobidikAreaID.compare ( e.get()->id().str() )  == 0 ) // Correct mobidikarea found
        {
            // It is assumed here that there is a navigation task, so only points on the ground are taken into consideration

            std::vector<geo::Vector3> points = e.get()->shape().get()->getMesh().getPoints();
            std::vector<geo::Vector3> groundPoints;
            const geo::Vec3T<double> pose = e.get()->pose().getOrigin();

            float sumX = 0;
            float sumY = 0;
            for ( unsigned int iPoints = 0; iPoints < points.size(); iPoints++ )
            {

                if ( points[iPoints].getZ() == 0 )
                {
                    sumX+= points[iPoints].getX();
                    sumY+= points[iPoints].getY();
                }
            }
            double centerX = sumX / points.size();
            double centerY = sumY / points.size();
            
            geo::Vec3d originMobidik ( centerX, centerY, 0.0 ); // Assumption: mobidikArea's are rectangular, so the centerpoint is always within the mobidik-area

            std::string orientationWPID = "orient_wp_" +  mobidikAreaID;
            for ( ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it ) // find corresponding orientation 
            {
                const ed::EntityConstPtr& e = *it;
                if ( orientationWPID.compare ( e.get()->id().str() ) == 0 ) // function returns 0 if strings are equal
                {
                    geo::Pose3D poseWP = e.get()->pose();
                    geo::Quaternion rotationWP = poseWP.getQuaternion();

                    tf::Quaternion q ( rotationWP.getX(), rotationWP.getY(), rotationWP.getZ(), rotationWP.getW() );
                    tf::Matrix3x3 matrix ( q );
                    double WP_roll, WP_pitch, WP_yaw;
                    matrix.getRPY ( WP_roll, WP_pitch, WP_yaw );

                    geo::Mat3 rotation;
                    rotation.setRPY ( WP_roll, WP_pitch, WP_yaw );

                    mobidikPosition->setOrigin( originMobidik );
                    mobidikPosition->setBasis( rotation );
                    
                    geo::Vec3d originInFrontOfMobidik(centerX + DIST_INTERMEDIATE_WAYPOINT_MOBID_RELEASE *cos(WP_yaw), centerY + DIST_INTERMEDIATE_WAYPOINT_MOBID_RELEASE *sin(WP_yaw), 0.0);
                    setpointInFrontOfMobidik->setOrigin( originInFrontOfMobidik );
                    setpointInFrontOfMobidik->setBasis( rotation );
                    
                    geo::Vec3d originAfterDisconnection(centerX + DIST_DISCONNECT_MOBID_RELEASE *cos(WP_yaw), centerY + DIST_DISCONNECT_MOBID_RELEASE *sin(WP_yaw), 0.0);
                    disconnectSetpoint->setOrigin( originAfterDisconnection );
                    disconnectSetpoint->setBasis( rotation );
                    return;
                    
                    
                }
            }            
        }
    }

    return;
}

TaskFeedbackCcu MobidikCollection::callReleasingStateMachine ( ros::Publisher &movbase_cancel_pub, move_base_msgs::MoveBaseGoal* goal_ptr, bool& sendgoal, visualization_msgs::MarkerArray markerArray, std::string areaID, const ed::WorldModel& world, ed::UpdateRequest& req, visualization_msgs::MarkerArray *markerArraytest, std_msgs::UInt16* controlMode, ros::Publisher &cmv_vel_pub, ropodNavigation::wrenches bumperWrenches, bool *mobidikConnected )
{
    //TODO set controlmode in all the states
    TaskFeedbackCcu tfb_nav;
    tfb_nav.wayp_n = waypoint_cnt_;
    tfb_nav.fb_nav = NAV_BUSY;
    sendgoal = false;
    geo::Pose3D finalMobidikPosition, setpoint, disconnectSetpoint;
    tf::Quaternion q;
    geometry_msgs::Twist output_vel;
    bool touched;
    float avgForce;

    switch ( nav_state_release_ )
    {

    case MOBID_REL_NAV_IDLE: // No waypoints received yet.
        tfb_nav.fb_nav = NAV_IDLE;
        ROS_INFO ( "MOBID_REL_NAV_IDLE" );
//         if (route_busy_ == true)
//         {
//              nav_next_state_ = MOBID_COLL_NAV_GOTOPOINT;
//             ROS_INFO("Waiting for door");
//         }
        break;

    case MOBID_REL_GET_SETPOINT_FRONT:
        ROS_INFO ( "MOBID_REL_GOTO_SETPOINT_FRONT" );
        // getFinalMobidikPos(markerArray);
        // navigate to final setpoint + offset to front
        getFinalMobidikPos ( world, areaID, &finalMobidikPosition, &disconnectSetpoint, &setpoint );

        goal_.target_pose.pose.position.x = setpoint.getOrigin().getX();
        goal_.target_pose.pose.position.y = setpoint.getOrigin().getY();
        goal_.target_pose.pose.position.z = setpoint.getOrigin().getZ();
        goal_.target_pose.pose.orientation.x = setpoint.getQuaternion().getX();
        goal_.target_pose.pose.orientation.y = setpoint.getQuaternion().getY();
        goal_.target_pose.pose.orientation.z = setpoint.getQuaternion().getZ();
        goal_.target_pose.pose.orientation.w = setpoint.getQuaternion().getW();

        nav_next_state_release_ = MOBID_REL_NAV_GOTOPOINT;
        nav_next_state_wp_release_ = MOBID_REL_GET_FINAL_MOBIDIK_POS;
        break;

    case MOBID_REL_GET_FINAL_MOBIDIK_POS:
        ROS_INFO ( "MOBID_REL_GOTO_FINAL_MOBIDIK_POS" );
        // navigate to final setpoint
        setpoint = finalMobidikPosition;
        goal_.target_pose.pose.position.x = setpoint.getOrigin().getX();
        goal_.target_pose.pose.position.y = setpoint.getOrigin().getY();
        goal_.target_pose.pose.position.z = setpoint.getOrigin().getZ();
        goal_.target_pose.pose.orientation.x = setpoint.getQuaternion().getX();
        goal_.target_pose.pose.orientation.y = setpoint.getQuaternion().getY();
        goal_.target_pose.pose.orientation.z = setpoint.getQuaternion().getZ();
        goal_.target_pose.pose.orientation.w = setpoint.getQuaternion().getW();

        nav_next_state_release_ = MOBID_REL_NAV_GOTOPOINT;
        nav_next_state_wp_release_ = MOBID_REL_DECOUPLING;
        bumperWrenchesVector_.clear();
        break;

    case MOBID_REL_DECOUPLING:
        ROS_INFO ( "MOBID_REL_DECOUPLING" );
        // Wait untill signal is given with force sensor at the front and go to the position which is slightly in front of the robot.

        touched = false;
        bumperWrenchesVector_.push_back ( bumperWrenches );
        if ( bumperWrenchesVector_.size() > N_COUNTS_WRENCHES ) // TODO update goal and how? How to let the software know there is actually no goal
        {
            bumperWrenchesVector_.erase ( bumperWrenchesVector_.begin() );
            avgForce = 0.0;
            for ( unsigned int ii = 0; ii < bumperWrenchesVector_.size(); ii++ )
            {
                avgForce += bumperWrenchesVector_[ii].front.wrench.force.x;
            }

            avgForce /= bumperWrenchesVector_.size();

            if ( std::fabs ( avgForce ) > MIN_FORCE_TOUCHED )
            {
                touched = true;
            }
        }

        if ( touched )
        {
            setpoint = disconnectSetpoint;
            goal_.target_pose.pose.position.x = setpoint.getOrigin().getX();
            goal_.target_pose.pose.position.y = setpoint.getOrigin().getY();
            goal_.target_pose.pose.position.z = setpoint.getOrigin().getZ();
            goal_.target_pose.pose.orientation.x = setpoint.getQuaternion().getX();
            goal_.target_pose.pose.orientation.y = setpoint.getQuaternion().getY();
            goal_.target_pose.pose.orientation.z = setpoint.getQuaternion().getZ();
            goal_.target_pose.pose.orientation.w = setpoint.getQuaternion().getW();
            
            *mobidikConnected = false;

            nav_next_state_release_ = MOBID_REL_NAV_GOTOPOINT;
            nav_next_state_wp_release_ = MOBID_REL_NAV_DONE;
        }
        
        break;

    case MOBID_REL_NAV_GOTOPOINT:
        ROS_INFO ( "MOBID_REL_NAV_GOTOPOINT" );
        goal_.target_pose.header.frame_id = "map";
        goal_.target_pose.header.stamp = ros::Time::now();
        ROS_INFO ( "Sending goal" );
        sendgoal = true;
        tfb_nav.fb_nav = NAV_GOTOPOINT;
        nav_next_state_release_ = MOBID_COLL_NAV_BUSY;
        break;

    case MOBID_REL_NAV_BUSY:
        ROS_INFO ( "MOBID_REL_NAV_BUSY" );
        if ( !isPositionValid() )
        {
            nav_next_state_release_ = MOBID_COLL_NAV_HOLD;
            break;
        }
        if ( isWaypointAchieved() ) // TODO set the tolerance parameters of the base_local_planner_params equal to the tolerances used in this function -> at the ros param server
        {
            ROS_INFO ( "Waypoint achieved" );
            nav_next_state_release_ = MOBID_COLL_NAV_WAYPOINT_DONE;
        }
        else
        {
            ROS_INFO ( "Waypoint not achieved" );
        }

        break;

    case MOBID_REL_NAV_WAYPOINT_DONE: //
        ROS_INFO ( "MOBID_REL_NAV_WAYPOINT_DONE" );
        tfb_nav.fb_nav = NAV_WAYPOINT_DONE; // is this still valid when we integrate an extra waypoint for this mobidik-collection? What should we send as feedback?
        nav_next_state_release_ = nav_next_state_wp_release_;

        break;

    case MOBID_REL_NAV_DONE: //
        ROS_INFO ( "MOBID_REL_NAV_DONE" );
        ROS_INFO ( "Navigation done" );
        tfb_nav.fb_nav = NAV_DONE;
        controlMode->data = ropodNavigation::LLC_NORMAL;
        stopNavigation();
        movbase_cancel_pub.publish ( true_bool_msg_ );
        nav_next_state_release_ = MOBID_COLL_NAV_IDLE;
        break;

    case MOBID_REL_NAV_HOLD: //
        ROS_INFO ( "MOBID_REL_NAV_HOLD" );
        ROS_INFO ( "Navigation on hold to receive feedback" );
        if ( isPositionValid() ) // check we have a valid position
            nav_next_state_ = MOBID_COLL_NAV_BUSY;
        break;

    case MOBID_REL_NAV_PAUSED: // this state is reached via a callback
        ROS_INFO ( "MOBID_REL_NAV_PAUSED" );
        if ( nav_paused_req_ )
        {
            movbase_cancel_pub.publish ( true_bool_msg_ );
            nav_paused_req_ = false;
        }
        break;

    default:
        nav_next_state_ = MOBID_REL_NAV_IDLE;
    }

    nav_state_ = nav_next_state_;

    *goal_ptr = goal_;

    return tfb_nav;
}

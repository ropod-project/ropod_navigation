#include "mobidik_collection_navigation.h"
#include <ed_gui_server/EntityInfos.h>

/*--------------------------------------------------------*/
MobidikCollection::MobidikCollection( )
{
    true_bool_msg_.data = true;
};

/*--------------------------------------------------------*/
MobidikCollection::~MobidikCollection()
{
    base_position_.reset();
};

bool MobidikCollection::getMobidik(visualization_msgs::MarkerArray markerArray, visualization_msgs::Marker *marker) 
// TODO via ED WM (if data-association solved!?)
{
        for (unsigned int ii = 0; ii < markerArray.markers.size(); ii++)
        {
                visualization_msgs::Marker markerToCheck = markerArray.markers.at(ii);
                if (markerToCheck.ns == "Mobidik")
                {
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
    double MD_roll, MD_pitch, MD_yaw;
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

            // find quadrant closest to the orientation of the orienation
            geo::real diffAngleMin = INFINITY;
            double angle, angleAtMinDiff, deltaAngleCorrected;
            for ( unsigned int ii = 0; ii < 4; ii++ )
            {
                double deltaAngle = ii*M_PI_2;
                angle = MD_yaw + deltaAngle;
                wrap ( &angle );

                geo::real diffAngle = std::min ( std::fabs ( angle - WP_yaw ), std::fabs ( angle + 2*M_PI - WP_yaw ) );
                if ( diffAngle < diffAngleMin )
                {
                    diffAngleMin = diffAngle;
                    angleAtMinDiff = angle;
                    deltaAngleCorrected = deltaAngle;
                }
            }
            
            MD_yaw = angleAtMinDiff;
            if ( deltaAngleCorrected == M_PI_2 || deltaAngleCorrected == 3*M_PI_2 ) // change of pi/2 of 3*pi/2-> width and length change
            {
                double mobidikWidthOld = mobidikWidth;
                mobidikWidth = mobidikLength;
                mobidikLength = mobidikWidthOld;
            }

            geo::Mat3 rotation;
            rotation.setRPY ( MD_roll, MD_pitch, MD_yaw );
            mobidikPose.setBasis ( rotation );

            ed::ConvexHull chull;
            
            float length = std::sqrt ( std::pow (0.5* mobidikWidth, 2.0 ) + std::pow ( 0.5*mobidikLength, 2.0 ) );

            geometry_msgs::Point p;
            angle = MD_yaw + atan2(mobidikLength, mobidikWidth);
            geo::Vec2f point ( mobidikPose.getOrigin().getX() + length*cos ( angle ), mobidikPose.getOrigin().getY() + length*sin ( angle )  );
            chull.points.push_back ( point ); 
            p.x = point.x; p.y = point.y; p.z = 0.0;
            points->points.push_back ( p );
          
            angle = MD_yaw + atan2(mobidikLength, -mobidikWidth);
            point.x = mobidikPose.getOrigin().getX() + length*cos ( angle ); point.y = mobidikPose.getOrigin().getY() + length*sin ( angle );
            p.x = point.x; p.y = point.y; p.z = 0.0;
            points->points.push_back ( p );
            
            angle = MD_yaw + atan2(-mobidikLength, -mobidikWidth);
            point.x = mobidikPose.getOrigin().getX() + length*cos ( angle ); point.y = mobidikPose.getOrigin().getY() + length*sin ( angle );
            p.x = point.x; p.y = point.y; p.z = 0.0;
            points->points.push_back ( p ); 
            
            angle = MD_yaw + atan2(-mobidikLength, mobidikWidth);
            point.x = mobidikPose.getOrigin().getX() + length*cos ( angle ); point.y = mobidikPose.getOrigin().getY() + length*sin ( angle );
            p.x = point.x; p.y = point.y; p.z = 0.0;
            points->points.push_back ( p );
            
            ed::tracking::FeatureProperties entityProperties; // TODO several properties double defined (both in key-properties and entitydescriptions) -> Make it consistent!
            entityProperties.rectangle_.set_w(mobidikWidth);
            entityProperties.rectangle_.set_d(mobidikLength);
            entityProperties.rectangle_.set_yaw(MD_yaw);

            *id = ed::Entity::generateID().str() + "-Mobidik";  // Generate unique ID
            req.setExistenceProbability ( *id, 1.0 ); // TODO magic number
            req.setConvexHullNew ( *id, chull, mobidikPose, mobidikMarker.header.stamp.toSec(), mobidikMarker.header.frame_id );
            req.setProperty ( *id, featurePropertiesKey, entityProperties );

            ROS_INFO ( "Mobidik Position set" );
            return;
        }

    }

    return;

};  

bool MobidikCollection::getMobidikPosition( const ed::WorldModel& world, ed::UUID mobidikID, geo::Pose3D *mobidikPose )
{
    for ( ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it )
    {
        const ed::EntityConstPtr& e = *it;

        if ( mobidikID.str().compare ( e.get()->id().str() )  == 0 )
        {
            *mobidikPose =  e.get()->pose();

            return true;
        }
    }

    return false;
}

void MobidikCollection::getSetpointInFrontOfMobidik ( const ed::WorldModel& world, ed::UUID mobidikID, geo::Pose3D *setpoint, visualization_msgs::Marker* points )
{

    geo::Pose3D mobidikPose;
    for ( ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it )
    {
        const ed::EntityConstPtr& e = *it;
        if ( mobidikID.str().compare ( e.get()->id().str() )  == 0 )
        {
                float mobidikLength;
                if( e->property ( featurePropertiesKey ))
                {
                ed::tracking::FeatureProperties entityProperties = e->property ( featurePropertiesKey );
                 mobidikLength = entityProperties.rectangle_.get_w();
                } else
                {
                        mobidikLength = MOBIDIK_LENGTH; 
                }
            mobidikPose =  e.get()->pose();
            float dist = 0.5* ( ROPOD_LENGTH + mobidikLength ) + DIST_IN_FRONT_OFF_MOBID;
            *setpoint = mobidikPose;
            
            geo::Quaternion rotation = mobidikPose.getQuaternion();
            tf::Quaternion q ( rotation.getX(), rotation.getY(), rotation.getZ(), rotation.getW() );
            tf::Matrix3x3 matrix ( q );
            double roll, pitch, yaw;
            matrix.getRPY ( roll, pitch, yaw );
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
        wrench->wrench.torque.x = INFINITY;
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
        xy_goal_tolerance_  = GOAL_MOBID_COLL_REACHED_DIST;
        yaw_goal_tolerance_ = GOAL_MOBID_REACHED_ANG;
}

void MobidikCollection::initNavStateRelease()
{
        nav_state_release_ = MOBID_REL_GET_SETPOINT_FRONT;
        avgWrenchesDetermined_ = false;
        initAvgWrench(&avgWrenches_.front);
        initAvgWrench(&avgWrenches_.left);
        initAvgWrench(&avgWrenches_.back);
        initAvgWrench(&avgWrenches_.right);
        xy_goal_tolerance_  = GOAL_MOBID_LOAD_REACHED_DIST; // we start with lower tolerance
        yaw_goal_tolerance_ = GOAL_MOBID_REACHED_ANG;        
}

geometry_msgs::WrenchStamped MobidikCollection::determineAvgWrench(std::vector<geometry_msgs::WrenchStamped> wrenchVector)
{
        float sumForcex = 0.0, sumForcey = 0.0, sumForcez = 0.0, sumTorquex = 0.0, sumTorquey = 0.0, sumTorquez = 0.0;
            for ( unsigned int ii = 0; ii < wrenchVector.size(); ii++ )
            {
                sumForcex += wrenchVector[ii].wrench.force.x;
                sumForcey += wrenchVector[ii].wrench.force.y;
                sumForcez += wrenchVector[ii].wrench.force.z;
                sumTorquex += wrenchVector[ii].wrench.torque.x;
                sumTorquey += wrenchVector[ii].wrench.torque.y;
                sumTorquez += wrenchVector[ii].wrench.torque.z;
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

void MobidikCollection::printWrenches(geometry_msgs::WrenchStamped wrench)
{
        std::cout <<
        wrench.wrench.force.x << ", " << 
        wrench.wrench.force.y << ", " << 
        wrench.wrench.force.z << ", " << 
        wrench.wrench.torque.x << ", " << 
        wrench.wrench.torque.y << ", " << 
        wrench.wrench.torque.z << std::endl;
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
        
        printWrenches(avgWrenches_.front);
        printWrenches(avgWrenches_.left);
        printWrenches(avgWrenches_.back);
        printWrenches(avgWrenches_.right);
        
        avgWrenchesDetermined_ = true;
}

// void MobidikCollection::initRelState()
// {
//         nav_state_release_ = MOBID_REL_GET_SETPOINT_FRONT;
// }

/*--------------------------------------------------------*/
bool MobidikCollection::isWaypointAchieved(double& dist_tolerance, double& angle_tolerance)
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
    
    if (pow( v3temp.x(),2) + pow(v3temp.y(),2) < pow(dist_tolerance,2)
            && fabs(qtemp.getAngle()) < angle_tolerance)
        return true;
    else
        return false;
}


/*--------------------------------------------------------*/
TaskFeedbackCcu MobidikCollection::callNavigationStateMachine(ros::Publisher &movbase_cancel_pub, move_base_msgs::MoveBaseGoal* goal_ptr, bool& sendgoal, visualization_msgs::MarkerArray markerArray, std::string areaID, const ed::WorldModel& world, ed::UpdateRequest& req, visualization_msgs::MarkerArray *markerArraytest, std_msgs::UInt16* controlMode, ros::Publisher &cmv_vel_pub, ropodNavigation::wrenches bumperWrenches, const bool robotReal)
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
    
    tf::Pose base_position_pose;
    double robot_yaw;
    
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
        break;
        
    case MOBID_COLL_FIND_MOBIDIK:
            ROS_INFO("MOBID_COLL_FIND_MOBIDIK");
            
            // Configure holonomic robot to move more accurately towards target
            system("rosrun dynamic_reconfigure dynparam set /maneuver_navigation/TebLocalPlannerROS max_vel_y 0.5 &");
            system("rosrun dynamic_reconfigure dynparam set /maneuver_navigation/TebLocalPlannerROS weight_kinematics_nh 0 &");
            system("rosrun dynamic_reconfigure dynparam set /maneuver_navigation/TebLocalPlannerROS weight_kinematics_forward_drive 0 &");

            if( getMobidik(markerArray,  &marker)  ) // Assumption: there is only 1 mobidik available at the moment
            {           
                    setMobidikPosition ( world, req, areaID, marker, &MobidikID_ED_, &points ); 
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
            getSetpointInFrontOfMobidik ( world, MobidikID_ED_, &setpoint_, &points);
            point2goal(&setpoint_);
            
            markerArraytest->markers.push_back ( points );

            nav_next_state_wp_ = MOBID_COLL_NAV_CONNECTING;
            nav_next_state_ = MOBID_COLL_NAV_GOTOPOINT;

            bumperWrenchesVector_.clear();
        break;
        
    case MOBID_COLL_NAV_CONNECTING:
        ROS_INFO ( "MOBID_COLL_NAV_CONNECTING" );


        touched = false;

        if ( ! robotReal )
        {
            geo::Pose3D mobidikPose;
            getMobidikPosition ( world, MobidikID_ED_, &mobidikPose );

            output_vel.linear.x = -BACKWARD_VEL_DOCKING;
            cmv_vel_pub.publish ( output_vel );

            float mobidikLength;
            for ( ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it )
            {
                const ed::EntityConstPtr& e = *it;
                if ( MobidikID_ED_.str().compare ( e.get()->id().str() )  == 0 )
                {

                    if ( e->property ( featurePropertiesKey ) )
                    {
                        ed::tracking::FeatureProperties entityProperties = e->property ( featurePropertiesKey );
                        mobidikLength = entityProperties.rectangle_.get_w();
                    }
                    else
                    {
                        mobidikLength = MOBIDIK_LENGTH;
                    }
                }
            }

            double dist2 = std::pow ( mobidikPose.getOrigin().getX() - base_position_->pose.position.x , 2.0 ) + std::pow ( mobidikPose.getOrigin().getY() - base_position_->pose.position.y , 2.0 );
            if ( dist2 < std::pow ( 0.5* ( ROPOD_LENGTH + mobidikLength ) + DIST_CONN_SIM, 2.0 ) )
            {
                touched = true;
            }
        }
        else
        {

            bumperWrenchesVector_.push_back ( bumperWrenches );
            if ( bumperWrenchesVector_.size() > N_COUNTS_WRENCHES )
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
                        avgTorque += bumperWrenchesVector_[ii].back.wrench.torque.z;
                    }

                    avgForce /= bumperWrenchesVector_.size();
                    avgTorque /= bumperWrenchesVector_.size();

                    std::cout << "avgForce Now= " << avgForce << ", avgTorque = " << avgTorque << std::endl;
                    std::cout << "Avg initially: " <<  avgWrenches_.back.wrench.force.x << ", " <<  avgWrenches_.back.wrench.torque.z << std::endl;
                    
                    forceCheck = std::fabs ( avgForce - avgWrenches_.back.wrench.force.x ) > MIN_FORCE_TOUCHED;
                    torqueCheck = std::fabs ( avgTorque - avgWrenches_.back.wrench.torque.z ) < MAX_TORQUE_TOUCHED;
                    
                    std::cout << "Checks " << forceCheck << ", " << torqueCheck << std::endl;

                    if ( forceCheck && torqueCheck )
                    {
                        touched = true;
                    }
                }
            }
        }

        if ( touched )
        {
                std::cout << "robotReal = " << robotReal << std::endl;
            if ( ! robotReal )
            {
                tfb_nav.fb_nav = NAV_DOCKED;
                nav_next_state_ = MOBID_COLL_NAV_EXIT_COLLECT_AREA;
                ROS_WARN ( " You are in simulation mode. The mobidik is assumed to be connected now." );
            }  else {
                bumperWrenchesVector_.clear();
                avgWrenchesDetermined_ = false;
                controlMode->data = ropodNavigation::LLC_NORMAL;
                nav_next_state_  = MOBID_COLL_NAV_COUPLING;
                
            }
                stamp_start_ = ros::Time::now();
                stamp_wait_ = ros::Duration(TIME_WAIT_CHANGE_OF_FOOTPRINT);         
        }

        break;
         
    case MOBID_COLL_NAV_COUPLING: // TODO
        ROS_INFO ( "MOBID_COLL_NAV_COUPLING" );
        controlMode->data = ropodNavigation::LLC_VEL;
        // couple mobidik manually and wait for signal;

        touched = false;
        
        bumperWrenchesVector_.push_back ( bumperWrenches );
        if ( bumperWrenchesVector_.size() > N_COUNTS_WRENCHES ) 
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
                
                 std::cout << "avgForce Now= " << avgForce << std::endl;
                    std::cout << "Avg initially: " <<  avgWrenches_.front.wrench.force.x << std::endl;
                     std::cout << "Check " << forceCheck << std::endl;

                if ( forceCheck )
                {
                    touched = true;
                }
            }
        }

        if ( touched )
        {
            // Docking done!
            tfb_nav.fb_nav = NAV_DOCKED;
            nav_next_state_ = MOBID_COLL_NAV_EXIT_COLLECT_AREA;
            bumperWrenchesVector_.clear();
            stamp_start_ = ros::Time::now();
            stamp_wait_ = ros::Duration(TIME_WAIT_CHANGE_OF_FOOTPRINT);
        }
        break;
        
    case MOBID_COLL_NAV_EXIT_COLLECT_AREA:
            ROS_INFO("MOBID_COLL_NAV_EXIT_COLLECT_AREA");
        if( ros::Time::now() - stamp_start_< stamp_wait_)    
            break;
        // For now just move a bit forward. Later the mobidik should exit the rail area.
        tf::poseMsgToTF(base_position_->pose,base_position_pose);
        robot_yaw = tf::getYaw(base_position_pose.getRotation());
        goal_.target_pose.pose = base_position_->pose;
        goal_.target_pose.pose.position.x += DIST_MOVE_FRONT_POSTDOCKING*std::cos(robot_yaw);
        goal_.target_pose.pose.position.y += DIST_MOVE_FRONT_POSTDOCKING*std::sin(robot_yaw);
        nav_next_state_wp_ = MOBID_COLL_NAV_DONE;
        nav_next_state_ = MOBID_COLL_NAV_GOTOPOINT;      
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
            ROS_INFO("MOBID_COLL_NAV_BUSY"); std::cout << "nav_next_state_ = " << nav_next_state_ << std::endl;
        if (!isPositionValid())
        {
            nav_next_state_ = MOBID_COLL_NAV_HOLD;
            break;
        }
        if (isWaypointAchieved(xy_goal_tolerance_,yaw_goal_tolerance_)) // TODO set the tolerance parameters of the base_local_planner_params equal to the tolerances used in this function -> at the ros param server
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
        controlMode->data = ropodNavigation::LLC_NORMAL;
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


void MobidikCollection::getFinalMobidikPos ( const ed::WorldModel& world, std::string mobidikAreaID, geo::Pose3D *mobidikPosition, geo::Pose3D *disconnectSetpoint , geo::Pose3D *setpointInFrontOfMobidik, visualization_msgs::Marker* points )
{
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

            geo::Vec3d finalPosMobidik ( poseWP.getOrigin().getX(), poseWP.getOrigin().getY(), 0.0 );
            mobidikPosition->setOrigin ( finalPosMobidik );
            mobidikPosition->setBasis ( rotation );


            geo::Vec3d originInFrontOfMobidik ( mobidikPosition->getOrigin().getX() + DIST_INTERMEDIATE_WAYPOINT_MOBID_RELEASE *cos ( WP_yaw ), mobidikPosition->getOrigin().getY() + DIST_INTERMEDIATE_WAYPOINT_MOBID_RELEASE *sin ( WP_yaw ), 0.0 );
            setpointInFrontOfMobidik->setOrigin ( originInFrontOfMobidik );
            setpointInFrontOfMobidik->setBasis ( rotation );

            geo::Vec3d originAfterDisconnection ( mobidikPosition->getOrigin().getX() + DIST_DISCONNECT_MOBID_RELEASE *cos ( WP_yaw ), mobidikPosition->getOrigin().getY() + DIST_DISCONNECT_MOBID_RELEASE *sin ( WP_yaw ), 0.0 );
            disconnectSetpoint->setOrigin ( originAfterDisconnection );
            disconnectSetpoint->setBasis ( rotation );

            geometry_msgs::Point p;
            p.x = mobidikPosition->getOrigin().getX();
            p.y = mobidikPosition->getOrigin().getY();
            p.z= 0.0;
            points->points.push_back ( p );

            p.x = setpointInFrontOfMobidik->getOrigin().getX();
            p.y = setpointInFrontOfMobidik->getOrigin().getY();
            p.z= 0.0;
            points->points.push_back ( p );

            p.x = disconnectSetpoint->getOrigin().getX();
            p.y = disconnectSetpoint->getOrigin().getY();
            p.z = 0.0;
            points->points.push_back ( p );
            return;
        }
    }
    
    return;
}

void MobidikCollection::point2goal(geo::Pose3D *setpoint)
{
        goal_.target_pose.pose.position.x = setpoint->getOrigin().getX();
        goal_.target_pose.pose.position.y = setpoint->getOrigin().getY();
        goal_.target_pose.pose.position.z = setpoint->getOrigin().getZ();
        goal_.target_pose.pose.orientation.x = setpoint->getQuaternion().getX();
        goal_.target_pose.pose.orientation.y = setpoint->getQuaternion().getY();
        goal_.target_pose.pose.orientation.z = setpoint->getQuaternion().getZ();
        goal_.target_pose.pose.orientation.w = setpoint->getQuaternion().getW();
}

TaskFeedbackCcu MobidikCollection::callReleasingStateMachine ( ros::Publisher &movbase_cancel_pub, move_base_msgs::MoveBaseGoal* goal_ptr, bool& sendgoal, visualization_msgs::MarkerArray markerArray, std::string areaID, const ed::WorldModel& world, ed::UpdateRequest& req, visualization_msgs::MarkerArray *markerArraytest, std_msgs::UInt16* controlMode, ros::Publisher &cmv_vel_pub, ropodNavigation::wrenches bumperWrenches, bool *mobidikConnected,  const bool robotReal )
{
    //TODO set controlmode in all the states
    TaskFeedbackCcu tfb_nav;
    tfb_nav.wayp_n = waypoint_cnt_;
    tfb_nav.fb_nav = NAV_BUSY;
    sendgoal = false;
    geo::Quaternion rotationWP;
    geo::Pose3D rotationSetpoint;
    tf::Quaternion q;
    tf::Matrix3x3 matrix;
    double WP_roll, WP_pitch, WP_yaw;
    geo::Mat3 rotation;
    geometry_msgs::Twist output_vel;
    bool touched;
    float avgForce;
    tf::Pose goal_tfpose;
    tf::Quaternion quat_temp;
    double robot_yaw;
    
    
    /* Just for visualisation purposes TODO to be removed */
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
// ---------------------------------------------------------------------------
    
    switch ( nav_state_release_ )
    {

    case MOBID_REL_NAV_IDLE: // No waypoints received yet.
        tfb_nav.fb_nav = NAV_IDLE;
        ROS_INFO ( "MOBID_REL_NAV_IDLE" );
        break;
    case MOBID_REL_GET_SETPOINT_FRONT:
        ROS_INFO ( "MOBID_REL_GOTO_SETPOINT_FRONT" );
        
                    // Configure slow movement
        system("rosrun dynamic_reconfigure dynparam set /maneuver_navigation/TebLocalPlannerROS max_vel_x 0.4 &");
        system("rosrun dynamic_reconfigure dynparam set /maneuver_navigation/TebLocalPlannerROS max_vel_x_backwards 0.2 &");
        system("rosrun dynamic_reconfigure dynparam set /maneuver_navigation/TebLocalPlannerROS max_vel_theta 0.5 &");     
//         system("rosrun dynamic_reconfigure dynparam set /maneuver_navigation/TebLocalPlannerROS global_plan_overwrite_orientation True &");
        

        getFinalMobidikPos ( world, areaID, &finalMobidikPosition_, &disconnectSetpoint_, &setpoint_, &points );
        point2goal(&setpoint_);
        
        markerArraytest->markers.push_back( points );
     
        nav_next_state_release_ = MOBID_REL_NAV_GOTOPOINT;
        nav_next_state_wp_release_ = MOBID_REL_GOTO_FINAL_MOBIDIK_POS;
        break;        
    case MOBID_REL_GOTO_FINAL_MOBIDIK_POS:
        ROS_INFO ( "MOBID_REL_GOTO_FINAL_MOBIDIK_POS" );
        // navigate to final setpoint
        point2goal(&finalMobidikPosition_);

        nav_next_state_release_ = MOBID_REL_NAV_GOTOPOINT;
        nav_next_state_wp_release_ = MOBID_REL_DECOUPLING;
        bumperWrenchesVector_.clear();
        break;

    case MOBID_REL_DECOUPLING:
        ROS_INFO ( "MOBID_REL_DECOUPLING" );
        // Wait untill signal is given with force sensor at the front and go to the position which is slightly in front of the robot.

        touched = false;

        if ( !robotReal )
        {
            touched = true;
            ROS_WARN ( " You are in simulation mode. The mobidik is assumed to be disconnected now." );
        }
        else
        {
            bumperWrenchesVector_.push_back ( bumperWrenches );
            if ( bumperWrenchesVector_.size() > N_COUNTS_WRENCHES ) // TODO update goal and how? How to let the software know there is actually no goal
            {
                bumperWrenchesVector_.erase ( bumperWrenchesVector_.begin() );

                if ( !avgWrenchesDetermined_ )
                {
                    determineAvgWrenches();
                }
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
        }
        
        if ( touched )
        {                 
            *mobidikConnected = false;
            tfb_nav.fb_nav = NAV_UNDOCKED;
            nav_next_state_release_ = MOBID_REL_NAV_WAIT_CHANGE_FOOTPRINT;
            stamp_start_ = ros::Time::now();
            stamp_wait_ = ros::Duration(TIME_WAIT_CHANGE_OF_FOOTPRINT);
            controlMode->data = ropodNavigation::LLC_VEL;
            xy_goal_tolerance_  = GOAL_MOBID_REL_REACHED_DIST; // for the last part we decrease tolerance
            yaw_goal_tolerance_ = GOAL_MOBID_REACHED_ANG; 
        }
        
        break;
    case MOBID_REL_NAV_WAIT_CHANGE_FOOTPRINT:
        ROS_INFO ( "MOBID_REL_NAV_WAIT_CHANGE_FOOTPRINT" );
        if( ros::Time::now() - stamp_start_< stamp_wait_)
            break;
        tf::poseMsgToTF(base_position_->pose,goal_tfpose);
        robot_yaw = tf::getYaw(goal_tfpose.getRotation());
        goal_.target_pose.pose = base_position_->pose;
        goal_.target_pose.pose.position.x += DIST_MOVE_FRONT_POSTRELEASING*std::cos(robot_yaw);
        goal_.target_pose.pose.position.y += DIST_MOVE_FRONT_POSTRELEASING*std::sin(robot_yaw);  
        nav_next_state_release_ = MOBID_REL_NAV_GOTOPOINT;
        nav_next_state_wp_release_ = MOBID_REL_ROTATE;
        break;
        
    case MOBID_REL_ROTATE: // TODO should be removed: rotating now done to measure the environment again, while we actually know that we just disconnected the mobidik
        ROS_INFO ( "MOBID_REL_ROTATE" );
        tf::poseMsgToTF(base_position_->pose,goal_tfpose);
        robot_yaw = tf::getYaw(goal_tfpose.getRotation());        
        quat_temp.setRPY(0.0,0.0,angles::normalize_angle(robot_yaw+M_PI));
        goal_tfpose.setRotation(quat_temp);
        tf::poseTFToMsg(goal_tfpose,goal_.target_pose.pose);  
        
        controlMode->data = ropodNavigation::LLC_NORMAL;
        
        nav_next_state_release_ = MOBID_REL_NAV_GOTOPOINT;
        nav_next_state_wp_release_ = MOBID_REL_NAV_DONE;
        break;

    case MOBID_REL_NAV_GOTOPOINT:
        ROS_INFO ( "MOBID_REL_NAV_GOTOPOINT" );
        goal_.target_pose.header.frame_id = "map";
        goal_.target_pose.header.stamp = ros::Time::now();
        ROS_INFO ( "Sending goal" );
        sendgoal = true;
        tfb_nav.fb_nav = NAV_GOTOPOINT;
        nav_next_state_release_ = MOBID_REL_NAV_BUSY;
        break;

    case MOBID_REL_NAV_BUSY:
        ROS_INFO ( "MOBID_REL_NAV_BUSY" );
        if ( !isPositionValid() )
        {
            nav_next_state_release_ = MOBID_REL_NAV_HOLD;
            break;
        }
        if ( isWaypointAchieved(xy_goal_tolerance_,yaw_goal_tolerance_) ) // TODO set the tolerance parameters of the base_local_planner_params equal to the tolerances used in this function -> at the ros param server
        {
            ROS_INFO ( "Waypoint achieved" );
            nav_next_state_release_ = MOBID_REL_NAV_WAYPOINT_DONE;
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
        nav_next_state_release_ = MOBID_REL_NAV_IDLE;
        break;

    case MOBID_REL_NAV_HOLD: //
        ROS_INFO ( "MOBID_REL_NAV_HOLD" );
        ROS_INFO ( "Navigation on hold to receive feedback" );
        if ( isPositionValid() ) // check we have a valid position
            nav_next_state_ = MOBID_REL_NAV_BUSY;
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

    nav_state_release_ = nav_next_state_release_;
    *goal_ptr = goal_;

    return tfb_nav;
}

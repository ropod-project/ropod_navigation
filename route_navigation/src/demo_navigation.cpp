#include "demo_navigation.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

wm::SimplifiedWorldModel simple_wm;
WaypointNavigation waypoint_navigation;
ElevatorNavigation elevator_navigation;
MobidikCollection mobidik_collection_navigation;
ropod_ros_msgs::ropod_door_detection door_status;
ropod_ros_msgs::TaskProgressGOTO ropod_progress_msg;
visualization_msgs::MarkerArray objectMarkerArray;
std::string areaID;

int active_nav = RopodNavigation::NAVTYPE_NONE;
bool action_msg_received = false;
bool prepare_next_loc = false;
std_msgs::Bool mobidikConnected;
ropod_ros_msgs::Action action_msg;
ropod_ros_msgs::Action action_msg_rec;
std_msgs::UInt16 LLCmodeSet, LLCmodeApplied;
std_msgs::Bool loadAttachedSet, loadAttachedApplied;

ropodNavigation::wrenches bumperWrenches;



bool robot_action_msg_received = false;
ropod_ros_msgs::RobotAction robot_action_msg;
ropod_ros_msgs::RobotAction robot_action_msg_rec;

void actionModelMediatorCallback(const ropod_ros_msgs::RobotAction::ConstPtr& robot_action_msg_)
{
//     ROS_INFO("Action message from Model mediator received. Action ID %s, %d areas", robot_action_msg_->action_id.c_str(), (int) robot_action_msg_->navigation_areas.size());
    std::cout << "Action message from Model mediator received. Action ID " << robot_action_msg_->action_id.c_str() << ", " << (int) robot_action_msg_->navigation_areas.size() << " areas" <<std::endl;
    robot_action_msg_rec = *robot_action_msg_;
    robot_action_msg_received = true;
}

void navigation_fbCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    waypoint_navigation.base_position = msg;
    elevator_navigation.base_position = msg;
    mobidik_collection_navigation.base_position_ = msg;
}

void actionCallback(const ropod_ros_msgs::Action::ConstPtr& action_msg_)
{
    ROS_INFO("Action message received. Action ID %s, %d areas", action_msg_->action_id.c_str(), (int) action_msg_->areas.size());
    action_msg_rec = *action_msg_;
    action_msg_received = true;
}
void doorDetectCallback(const ropod_ros_msgs::ropod_door_detection::ConstPtr& DoorStmsg)
{
    door_status = *DoorStmsg;
}

void MarkerArrayCallback(const visualization_msgs::MarkerArray::ConstPtr& MarkerArrayStmsg)
{
    objectMarkerArray = *MarkerArrayStmsg;
}

void wrenchFrontCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench_frontStmsg)
{
    bumperWrenches.front = *wrench_frontStmsg;
}

void wrenchLeftCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench_leftStmsg)
{
    bumperWrenches.left = *wrench_leftStmsg;
}

void wrenchBackCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench_backStmsg)
{
    bumperWrenches.back = *wrench_backStmsg;
}

void wrenchRightCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench_rightStmsg)
{
    bumperWrenches.right = *wrench_rightStmsg;
}

void LLCmodeAppliedCallback(const std_msgs::UInt16::ConstPtr& LLCmodeAppliedStmsg)
{
    LLCmodeApplied = *LLCmodeAppliedStmsg;
}

void loadAttachedCallback(const std_msgs::Bool::ConstPtr& loadAttachedStmsg)
{
    loadAttachedApplied = *loadAttachedStmsg;
}


// TODO this needs to be replaced with a query to the world model
geometry_msgs::PoseStamped getPoseFromWorldModel(const std::string &area_name)
{
    std::string param_name = "/areas/" + area_name;
    std::vector<double> waypoint;
    ros::param::get(param_name, waypoint);    
    geometry_msgs::PoseStamped p;
    p.pose.position.x = waypoint[0];
    p.pose.position.y = waypoint[1];
    tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, waypoint[2]);
    q.normalize();
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();
    return p;
}

// this needs to be replaced with a query to the world model
std::string getAreaTypeFromWorldModel(const std::string &area_name)
{
    std::string param_name = "/area_types/" + area_name;
    std::string area_type;
    ros::param::get(param_name, area_type);
    return area_type;
}

RopodNavigation::RopodNavigation()
{
}

RopodNavigation::~RopodNavigation()
{
}


void RopodNavigation::initialize ( ed::InitData& init )
{
    ros::NodeHandle n ( "~" );
    n.setCallbackQueue(&cb_queue_);
    
    std::string moveBaseServerName;
    std::string navigationFeedbackTopic;
    std::string navigationCancelTopic;

    n.param<std::string> ( "move_base_server", moveBaseServerName, "/move_base" );
    n.param<std::string> ( "move_base_feedback_topic", navigationFeedbackTopic, "/maneuver_navigation/feedback" );
    n.param<std::string> ( "move_base_cancel_topic", navigationCancelTopic, "/move_base/cancel" );
    
    sub_ccu_commands_ = n.subscribe<ropod_ros_msgs::Action> ( "goto_action", 10, actionCallback );
    subdoor_status_ = n.subscribe<ropod_ros_msgs::ropod_door_detection> ( "/door", 10, doorDetectCallback );
    objectMarkers_ = n.subscribe<visualization_msgs::MarkerArray> ( "/ed/gui/objectMarkers", 10, MarkerArrayCallback ); // TODO query these properties via ED instead of ROS
    LLCmodeApplied_ = n.subscribe<std_msgs::UInt16> ( "/ropod/LLCmode_Applied", 10, LLCmodeAppliedCallback );
    loadAttachedApplied_ = n.subscribe<std_msgs::Bool> ( "/ropod/load_attached_applied", 10, loadAttachedCallback ); 
    
    // Communication to the bumpers 
    wrenchFront_ = n.subscribe<geometry_msgs::WrenchStamped> ( "/ropod/wrench_front", 10, wrenchFrontCallback );
    wrenchLeft_ = n.subscribe<geometry_msgs::WrenchStamped> ( "/ropod/wrench_left", 10, wrenchLeftCallback );
    wrenchBack_ = n.subscribe<geometry_msgs::WrenchStamped> ( "/ropod/wrench_back", 10, wrenchBackCallback );
    wrenchRight_ = n.subscribe<geometry_msgs::WrenchStamped> ( "/ropod/wrench_right", 10, wrenchRightCallback );
    
    LLCmodeSet_pub_ = n.advertise<std_msgs::UInt16> ( "/ropod/Set_LLCmode", 1 );
    loadAttachedSet_pub_ = n.advertise<std_msgs::Bool> ( "/ropod/Set_load_attached", 1 );
    
    
    sub_model_med_commands_ = n.subscribe<ropod_ros_msgs::RobotAction> ( "/model_mediator_action", 10, actionModelMediatorCallback );
    sendGoal_pub_ = n.advertise<geometry_msgs::PoseStamped> ("/route_navigation/goal", 1);
    sub_navigation_fb_ =   n.subscribe<geometry_msgs::PoseStamped> ( navigationFeedbackTopic, 10, navigation_fbCallback );
           

    door_status.closed = false;
    door_status.open = false;
    door_status.undetectable = true;
    
    mobidikConnected.data = false;
    
    movbase_cancel_pub_ = n.advertise<actionlib_msgs::GoalID> ( navigationCancelTopic, 1 );
    ropod_task_fb_pub_ = n.advertise<ropod_ros_msgs::TaskProgressGOTO> ( "/progress", 1 );
    ObjectMarkers_pub_ = n.advertise<visualization_msgs::MarkerArray> ( "/ed/gui/objectMarkers2", 3 ); // TODO remove
    cmd_vel_pub_ = n.advertise<geometry_msgs::Twist> ( "/ropod/cmd_vel", 1 );

//     ac_ = new MoveBaseClient ( moveBaseServerName, true );
// 
//     while ( !ac_->waitForServer ( ros::Duration ( 5.0 ) ) )
//     {
//         ROS_INFO ( "Waiting for the move_base action server to come up" );
//     }

    send_goal_ = false;

    // Wait for route to be published
    ROS_INFO ( "Wait for route" );

}

void RopodNavigation::process ( const ed::WorldModel& world, ed::UpdateRequest& req )
{
    cb_queue_.callAvailable();    

    int curr_loc;
    std::vector<ropod_ros_msgs::Area>::const_iterator curr_area;
    std::vector<ropod_ros_msgs::Waypoint>::const_iterator curr_wp;
    int it_idwp;
    visualization_msgs::MarkerArray markerArray; // TODO remove
    std_msgs::UInt16 controlMode;
    controlMode.data = ropodNavigation::LLC_NORMAL;

    
    // Process the received Action message. Point to first location
    if ( action_msg_received ) // checks need to be done if a current navigation is taking place
    {
        action_msg_received = false;
        curr_loc = 0;
        action_msg = action_msg_rec;
        waypoint_ids_.clear();
        path_msg_.poses.clear();

        if ( action_msg.type == "GOTO" )
        {
                 ROS_INFO("GOTO-action set");
            // Here query world model mediator with OSM to obtain waypoints from the area id path plan
        }
        else if ( action_msg.type == "ENTER_ELEVATOR" )
        {
                 ROS_INFO("Enter elevator action set");
           
            active_nav = NAVTYPE_ELEVATOR;
            areaID = action_msg.areas[0].area_id; // Get ID of elevator area
            elevator_navigation.startNavigation(areaID,  world); // Set waypoint from worldmodel
        }
         else if ( action_msg.type == "COLLECT_MOBIDIK" ) // TODO: integrated in the CCU!?
        {
                ROS_INFO("Collect mobidik-action set");
            active_nav = NAVTYPE_MOBIDIK_COLLECTION;
            mobidik_collection_navigation.initNavState();
            areaID = action_msg.areas[0].area_id;
        }

    }

    // Process the received RobotAction message.
    if ( robot_action_msg_received ) // checks need to be done if a current navigation is taking place
    {
        robot_action_msg_received = false;
        robot_action_msg = robot_action_msg_rec;
        waypoint_ids_.clear();
        path_msg_.poses.clear();
        std::cout << "robot_action_msg.type = " << robot_action_msg_received << std::endl;
        if ( robot_action_msg.type == "GOTO" )
        {
                 ROS_INFO("GOTO-action set");
            // Extract all areas and waypoints to go the corresponding location
            for ( std::vector<ropod_ros_msgs::NavigationArea>::const_iterator curr_area = robot_action_msg.navigation_areas.begin();
                    curr_area != robot_action_msg.navigation_areas.end(); ++curr_area )
            {
                if (curr_area->type == "Intersection")
                    continue;
                for ( std::vector<ropod_ros_msgs::Waypoint>::const_iterator curr_wayp = curr_area->waypoints.begin();
                    curr_wayp != curr_area->waypoints.end(); ++curr_wayp )
                {
                    geometry_msgs::PoseStamped p;
                    p.pose = curr_wayp->waypoint_pose ;
                    std::string area_type = curr_area->type;
                    path_msg_.poses.push_back ( p );
                    waypoint_ids_.push_back ( curr_wayp->semantic_id );
                }
            }
            waypoint_navigation.startNavigation ( path_msg_ );
            active_nav = NAVTYPE_WAYPOINT;
        }
        else 
        {
            ROS_ERROR("Wrong message: Commands other than GOTO do not need World model mediation");
        }

    }    
   
    
    TaskFeedbackCcu nav_state;
    
//     std::cout << "Before state-machine: demo navigation.cpp: nav_state.fb_nav = " << nav_state.fb_nav << std::endl;
//     std::cout << "active nav = " << active_nav << std::endl;
    
    // Select the corresponding navigation
    switch ( active_nav )
    {

    case NAVTYPE_WAYPOINT:
         ROS_INFO("NAV_WAYPOINT");
        nav_state = waypoint_navigation.callNavigationStateMachine ( movbase_cancel_pub_, &goal_, send_goal_ );
        break;

    case NAVTYPE_ELEVATOR:
         ROS_INFO("NAV_ELEVATOR");
        nav_state = elevator_navigation.callNavigationStateMachine ( movbase_cancel_pub_, &goal_, send_goal_, door_status );
        if ( nav_state.fb_nav == NAV_DONE )
        {
            active_nav = NAVTYPE_NONE;
            prepare_next_loc = true;
        }
        break;
        
    case NAVTYPE_MOBIDIK_COLLECTION:
         ROS_INFO("NAVTYPE_MOBIDIK_COLLECTION");
        nav_state = mobidik_collection_navigation.callNavigationStateMachine ( movbase_cancel_pub_, &goal_, send_goal_, objectMarkerArray, areaID, world, req, &markerArray, &controlMode, cmd_vel_pub_, bumperWrenches);
       std::cout << "demo navigation.cpp: nav_state.fb_nav = " << nav_state.fb_nav << std::endl;
        std::cout << "send_goal_ = " << send_goal_ << std::endl;
        if ( send_goal_ )
        {
                std::cout << 
                goal_.target_pose.pose.position.x << ", " <<
                goal_.target_pose.pose.position.y << ", " <<
                goal_.target_pose.pose.position.z<< ", " <<
                goal_.target_pose.pose.orientation.x<< ", " <<
                goal_.target_pose.pose.orientation.y<< ", " <<
                goal_.target_pose.pose.orientation.z<< ", " <<
                goal_.target_pose.pose.orientation.w << std::endl;
        }
        
        if ( nav_state.fb_nav == NAV_DONE )
        {
                ROS_INFO("nav_state.fb_nav == NAV_DONE");
            active_nav = NAVTYPE_NONE;
            prepare_next_loc = true;
            mobidikConnected.data = true;
        }
        break;


    default:
        nav_state.fb_nav = NAV_IDLE;
        break;
    }

    // Send feedback
    // TODO: how to properly give feedback when taking elevator since waypoints are not fixed? e.g. multiple possible waiting areas outside elevator
    if ( nav_state.fb_nav == NAV_DONE )
    {
        ROS_INFO ( "NAV_ELEVATOR/MOBIDIK DONE!" ); // TODO Separate nav_state for MOBIDIK COLLECTION?
        active_nav = NAVTYPE_NONE;
        prepare_next_loc = true;
    }
    else if ( nav_state.fb_nav == NAV_WAYPOINT_DONE )
    {
            int state = NAVTYPE_MOBIDIK_COLLECTION;
            int active_nav_copy = active_nav ;
            bool check = (active_nav_copy =! state );
        if ( check ) // TODO what kind of feedback during mobidik collection?
        {
            ROS_INFO ( "Waypoint done notification received" );
            if ( nav_state.wayp_n<=waypoint_ids_.size() )
                ropod_progress_msg.area_name = waypoint_ids_[nav_state.wayp_n-1];
            ropod_progress_msg.action_id = action_msg.action_id;
            ropod_progress_msg.action_type = action_msg.type;
            ropod_progress_msg.status = "reached";
            ropod_progress_msg.sequenceNumber = nav_state.wayp_n;
            ropod_progress_msg.totalNumber = waypoint_ids_.size();
            ropod_task_fb_pub_.publish ( ropod_progress_msg );
            // Update coming waypoint
        }
    }
    else if ( nav_state.fb_nav == NAV_GOTOPOINT )
    {
            int state = NAVTYPE_MOBIDIK_COLLECTION;
            int active_nav_copy = active_nav ;
            bool check = (active_nav_copy =! state );
            
        if ( check ) // TODO what kind of feedback during mobidik collection?
        {
            if ( nav_state.wayp_n<=waypoint_ids_.size() )
                ropod_progress_msg.area_name = waypoint_ids_[nav_state.wayp_n-1];
            ropod_progress_msg.action_id = action_msg.action_id;
            ropod_progress_msg.action_type = action_msg.type;
            ropod_progress_msg.status = "reaching";
            ropod_progress_msg.sequenceNumber = nav_state.wayp_n;
            ropod_progress_msg.totalNumber = waypoint_ids_.size();
            ropod_task_fb_pub_.publish ( ropod_progress_msg );
        }
        
    }

    // Send navigation command
    if ( send_goal_ )
         sendGoal_pub_.publish(goal_.target_pose); //ac_->sendGoal ( goal_ );
    
//     std::cout << "active nav after sending goal = " << active_nav << std::endl;
    ObjectMarkers_pub_.publish( markerArray );
    LLCmodeSet_pub_.publish(controlMode);
    loadAttachedSet_pub_.publish(mobidikConnected);
}

ED_REGISTER_PLUGIN(RopodNavigation)
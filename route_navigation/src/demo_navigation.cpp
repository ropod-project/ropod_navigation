#include "demo_navigation.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

wm::SimplifiedWorldModel simple_wm;
WaypointNavigation waypoint_navigation;
ElevatorNavigation elevator_navigation;
MobidikCollection mobidik_collection_navigation;
ropod_ros_msgs::DoorDetection door_status;
ropod_ros_msgs::TaskProgressGOTO ropod_progress_msg;
ropod_ros_msgs::TaskProgressDOCK ropod_progress_dock_msg;
visualization_msgs::MarkerArray objectMarkerArray;
std::string areaID;

int active_nav = RopodNavigation::NAVTYPE_NONE;
bool action_msg_received = false;
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

void RopodNavigation::actionRoutePlannerCallback(const actionlib::SimpleClientGoalState& state, const ropod_ros_msgs::RoutePlannerResultConstPtr& result)
{
         
  // Extract waypoints into RobotActionMessage
  ropod_ros_msgs::NavigationArea robot_nav_area;  
  robot_action_msg_rec.navigation_areas.clear();
  
  ropod_ros_msgs::RoutePlannerResult result_internal = *result;
  
    for (std::vector<ropod_ros_msgs::Area>::iterator area_it = result_internal.areas.begin(); area_it != result_internal.areas.end(); area_it++) 
    {
        int no_of_waypts = 0;
        robot_nav_area.waypoints.clear();
        
        for (std::vector<ropod_ros_msgs::Waypoint>::iterator wayp_area_it = area_it->waypoints.begin(); wayp_area_it != area_it->waypoints.end(); wayp_area_it++)
        {
            robot_nav_area.area_id = wayp_area_it->area_id;
            robot_nav_area.name = wayp_area_it->semantic_id;
            robot_nav_area.type = area_it->type;
            robot_nav_area.waypoints.push_back(*wayp_area_it);
            std::cout << "Waypoint" << std::endl;
            std::cout << "Type: " << area_it->type << std::endl;
            std::cout << "ID: " << wayp_area_it->area_id << std::endl;
            std::cout << "pos(" << wayp_area_it->waypoint_pose.position.x << "," << wayp_area_it->waypoint_pose.position.y << ")" << std::endl; 
            std::cout << "quat(" << wayp_area_it->waypoint_pose.orientation.w << "," << wayp_area_it->waypoint_pose.orientation.x << "," << wayp_area_it->waypoint_pose.orientation.y << "," << wayp_area_it->waypoint_pose.orientation.z << ") \n" << std::endl; 
           
        }
        robot_action_msg_rec.navigation_areas.push_back(robot_nav_area);
    }
     
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
void doorDetectCallback(const ropod_ros_msgs::DoorDetection::ConstPtr& DoorStmsg)
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


RopodNavigation::RopodNavigation()
{
}

RopodNavigation::~RopodNavigation()
{
    delete route_planner_action_client_ptr_;
}

std::string RopodNavigation::GetEnv( const std::string & var ) 
{
     const char * val = std::getenv( var.c_str() );
     if ( val == 0 ) 
     {
         return "";
         ROS_ERROR("ROBOT_REAL variable not set. ");
     }
     else 
     {
         return val;
     }
}

void RopodNavigation::initialize ( ed::InitData& init )
{
    ros::NodeHandle n ( "~" );
    n.setCallbackQueue(&cb_queue_);
    
    tue::Configuration& config = init.config;
    
    std::string moveBaseServerName;
    std::string navigationFeedbackTopic;
    std::string navigationCancelTopic;

    n.param<std::string> ( "move_base_server", moveBaseServerName, "/move_base" );
    n.param<std::string> ( "move_base_feedback_topic", navigationFeedbackTopic, "/maneuver_navigation/feedback" );

    sub_ccu_goto_commands_ = n.subscribe<ropod_ros_msgs::Action> ( "goto_action", 10, actionCallback );
    sub_ccu_dock_commands_ = n.subscribe<ropod_ros_msgs::Action> ( "dock_action", 10, actionCallback );
    sub_ccu_undock_commands_ = n.subscribe<ropod_ros_msgs::Action> ( "undock_action", 10, actionCallback );

    subdoor_status_ = n.subscribe<ropod_ros_msgs::DoorDetection> ( "/door", 10, doorDetectCallback );
    objectMarkers_ = n.subscribe<visualization_msgs::MarkerArray> ( "/ed/gui/objectMarkers", 10, MarkerArrayCallback ); // TODO query these properties via ED instead of ROS
    LLCmodeApplied_ = n.subscribe<std_msgs::UInt16> ( "/ropod/LLCmode_Applied", 10, LLCmodeAppliedCallback );
    loadAttachedApplied_ = n.subscribe<std_msgs::Bool> ( "/ropod/load_attached_Applied", 10, loadAttachedCallback ); 
    
    // Communication to the bumpers 
    wrenchFront_ = n.subscribe<geometry_msgs::WrenchStamped> ( "/ropod/wrench_front", 10, wrenchFrontCallback );
    wrenchLeft_ = n.subscribe<geometry_msgs::WrenchStamped> ( "/ropod/wrench_left", 10, wrenchLeftCallback );
    wrenchBack_ = n.subscribe<geometry_msgs::WrenchStamped> ( "/ropod/wrench_back", 10, wrenchBackCallback );
    wrenchRight_ = n.subscribe<geometry_msgs::WrenchStamped> ( "/ropod/wrench_right", 10, wrenchRightCallback );
    
    LLCmodeSet_pub_ = n.advertise<std_msgs::UInt16> ( "/ropod/Set_LLCmode", 1 );
    loadAttachedSet_pub_ = n.advertise<std_msgs::Bool> ( "/route_navigation/set_load_attached", 1 );
    
    sub_model_med_commands_ = n.subscribe<ropod_ros_msgs::RobotAction> ( "/model_mediator_action", 10, actionModelMediatorCallback );
    sendGoal_pub_ = n.advertise<geometry_msgs::PoseStamped> ("/route_navigation/simple_goal", 1);
    mn_sendGoal_pub_ = n.advertise<maneuver_navigation::Goal> ("/route_navigation/goal", 1);
    sub_navigation_fb_ =   n.subscribe<geometry_msgs::PoseStamped> ( navigationFeedbackTopic, 10, navigation_fbCallback );

    movbase_cancel_pub_ = n.advertise<std_msgs::Bool>("/route_navigation/cancel", 1 );
    ropod_task_goto_fb_pub_ = n.advertise<ropod_ros_msgs::TaskProgressGOTO> ( "/task_progress/goto", 1 );
    ropod_task_dock_fb_pub_ = n.advertise<ropod_ros_msgs::TaskProgressDOCK> ( "/task_progress/dock", 1 );
    ObjectMarkers_pub_ = n.advertise<visualization_msgs::MarkerArray> ( "/ed/gui/objectMarkers2", 3 ); // TODO remove
    cmd_vel_pub_ = n.advertise<geometry_msgs::Twist> ( "/ropod/cmd_vel", 1 );
    poses_waypoints_pub_ = n.advertise<geometry_msgs::PoseArray> ( "/route_navigation/nav_waypoints", 1 );
    

    send_goal_ = false;
    
    door_status.closed = false;
    door_status.open = false;
    door_status.undetectable = true;
    
    bool mobikConnected = false;
    config.value("mobidik_initially_connected", mobikConnected, tue::OPTIONAL);
    mobidikConnected_ = (mobikConnected != false);
    
    std::string robotReal_str = GetEnv("ROBOT_REAL");  
    robotReal = (robotReal_str.compare( "true" ) == 0);
    if(robotReal)
    {
            ROS_INFO("Robot is running in real mode");
    } else {
            ROS_INFO("Robot running in simulation mode");
            }
    
    init.properties.registerProperty ( "Feature", mobidik_collection_navigation.featurePropertiesKey, new FeaturPropertiesInfo );
    controlMode_.data = ropodNavigation::LLC_NORMAL;
    

    route_planner_action_client_ptr_ = new actionlib::SimpleActionClient<ropod_ros_msgs::RoutePlannerAction>("/route_planner",true);    
    route_planner_action_client_ptr_->waitForServer();    


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
                 
            ropod_ros_msgs::RoutePlannerGoal req;
            req.areas = action_msg.areas;
            route_planner_action_client_ptr_->sendGoal(req, boost::bind(&RopodNavigation::actionRoutePlannerCallback, this, _1, _2));
            // Make request and wait for result in a callback

                 
        }
        else if ( action_msg.type == "ENTER_ELEVATOR" )
        {
                 ROS_INFO("Enter elevator action set");
           
            active_nav = NAVTYPE_ELEVATOR;
            areaID = action_msg.areas[0].area_id; // Get ID of elevator area
            elevator_navigation.startNavigation(areaID,  world); // Set waypoint from worldmodel
        }
         else if ( action_msg.type == "DOCK" ) // TODO: integrated in the CCU!?
        {
                ROS_INFO("Collect mobidik-action set");
            active_nav = NAVTYPE_MOBIDIK_COLLECTION;
            mobidik_collection_navigation.initNavState();
            areaID = action_msg.areas[0].area_id;
        }
        else if ( action_msg.type == "UNDOCK" ) // TODO: integrated in the CCU!?
        {
                ROS_INFO("Release mobidik-action set");
            active_nav = NAVTYPE_MOBIDIK_RELEASE;
            mobidik_collection_navigation.initNavStateRelease();
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
        
        geometry_msgs::PoseArray vispos_array;
        vispos_array.header.frame_id = "map";       

        ROS_INFO("GOTO-action set");
        // Extract all areas and waypoints to go the corresponding location
        for ( std::vector<ropod_ros_msgs::NavigationArea>::const_iterator curr_area = robot_action_msg.navigation_areas.begin();
                curr_area != robot_action_msg.navigation_areas.end(); ++curr_area )
        {
            if (curr_area->type == "door")
                continue;
            for ( std::vector<ropod_ros_msgs::Waypoint>::const_iterator curr_wayp = curr_area->waypoints.begin();
                curr_wayp != curr_area->waypoints.end(); ++curr_wayp )
            {
                geometry_msgs::PoseStamped p;
                p.pose = curr_wayp->waypoint_pose ;
                std::string area_type = curr_area->type;
                path_msg_.poses.push_back ( p );
                vispos_array.poses.push_back(p.pose);
                
            std::cout << "Waypoint" << std::endl;
            std::cout << "Type: " << curr_area->type << std::endl;
            std::cout << "pos(" << curr_wayp->waypoint_pose.position.x << "," << curr_wayp->waypoint_pose.position.y << ")" << std::endl; 
            std::cout << "quat(" << curr_wayp->waypoint_pose.orientation.w << "," << curr_wayp->waypoint_pose.orientation.x << "," << curr_wayp->waypoint_pose.orientation.y << "," << curr_wayp->waypoint_pose.orientation.z << ") \n" << std::endl; 
                
                waypoint_ids_.push_back ( curr_wayp->semantic_id );
            }
        }
        // waypoint_navigation.startNavigation ( path_msg_ );
        waypoint_navigation.startNavigation(robot_action_msg.navigation_areas);
        active_nav = NAVTYPE_WAYPOINT;
        poses_waypoints_pub_.publish(vispos_array);
        

    }    
   
    
    TaskFeedbackCcu nav_state;
    
    bool send_mn_goal_ = false;    
    // Select the corresponding navigation
    switch ( active_nav )
    {

    case NAVTYPE_WAYPOINT:
         ROS_INFO("NAV_WAYPOINT");
        nav_state = waypoint_navigation.callNavigationStateMachine (movbase_cancel_pub_, mn_goal_, mn_feedback_, send_mn_goal_);
        break;

    case NAVTYPE_ELEVATOR:
         ROS_INFO("NAV_ELEVATOR");
        nav_state = elevator_navigation.callNavigationStateMachine ( movbase_cancel_pub_, &goal_, send_goal_, door_status );
        if ( nav_state.fb_nav == NAV_DONE )
        {
            active_nav = NAVTYPE_NONE;
        }
        break;
        
    case NAVTYPE_MOBIDIK_COLLECTION:
         ROS_INFO("NAVTYPE_MOBIDIK_COLLECTION");
        nav_state = mobidik_collection_navigation.callNavigationStateMachine ( movbase_cancel_pub_, &goal_, send_goal_, objectMarkerArray, areaID, world, req, &markerArray, &controlMode_, cmd_vel_pub_, bumperWrenches, robotReal);

        if ( nav_state.fb_nav == NAV_DONE )
        {
                ROS_INFO("nav_state.fb_nav == NAV_DONE");
            active_nav = NAVTYPE_NONE;

            // Sending feedback to ropod_task_executor
            if ( nav_state.wayp_n<=waypoint_ids_.size() )
                ropod_progress_dock_msg.area_name = waypoint_ids_[nav_state.wayp_n-1];
            ropod_progress_dock_msg.action_id = action_msg.action_id;
            ropod_progress_dock_msg.action_type = action_msg.type;
            ropod_progress_dock_msg.status = ropod_ros_msgs::TaskProgressDOCK::DOCKED;
            ropod_task_dock_fb_pub_.publish ( ropod_progress_msg );

        }

        if ( nav_state.fb_nav == NAV_DOCKED )
        {
                ROS_INFO("nav_state.fb_nav == NAV_DOCKED");
            mobidikConnected_ = true;
            mobidikConnected.data = mobidikConnected_;
            loadAttachedSet_pub_.publish(mobidikConnected);            
        }        
        break;

        
    case NAVTYPE_MOBIDIK_RELEASE:
          ROS_INFO("NAVTYPE_MOBIDIK_RELEASE");
        nav_state = mobidik_collection_navigation.callReleasingStateMachine (movbase_cancel_pub_,  &goal_, send_goal_, objectMarkerArray, areaID, world, req, &markerArray, &controlMode_, cmd_vel_pub_, bumperWrenches, &mobidikConnected_, robotReal );
        
        if ( nav_state.fb_nav == NAV_DONE )
        {
                ROS_INFO("nav_state.fb_nav == NAV_DONE");
            active_nav = NAVTYPE_NONE;

            // Sending feedback to ropod_task_executor
            if ( nav_state.wayp_n<=waypoint_ids_.size() )
                ropod_progress_dock_msg.area_name = waypoint_ids_[nav_state.wayp_n-1];
            ropod_progress_dock_msg.action_id = action_msg.action_id;
            ropod_progress_dock_msg.action_type = action_msg.type;
            ropod_progress_dock_msg.status = ropod_ros_msgs::TaskProgressDOCK::UNDOCKED;
            ropod_task_dock_fb_pub_.publish ( ropod_progress_msg );

        }
        if ( nav_state.fb_nav == NAV_UNDOCKED )
        {
                ROS_INFO("nav_state.fb_nav == NAV_UNDOCKED");
            mobidikConnected_ = false;
            mobidikConnected.data = mobidikConnected_;
            loadAttachedSet_pub_.publish(mobidikConnected);            
        }          
        break;


    default:
        ROS_INFO("NAVTYPE_IDLE");
        nav_state.fb_nav = NAV_IDLE;
        break;
    }

    // Send feedback
    // TODO: how to properly give feedback when taking elevator since waypoints are not fixed? e.g. multiple possible waiting areas outside elevator
    if ( nav_state.fb_nav == NAV_DONE )
    {
        ROS_INFO ( "NAV DONE!" ); // TODO Separate nav_state for MOBIDIK COLLECTION?
        active_nav = NAVTYPE_NONE;
    }
    else if ( nav_state.fb_nav == NAV_WAYPOINT_DONE )
    {
            int state = NAVTYPE_MOBIDIK_COLLECTION;
            int active_nav_copy = active_nav ;
            bool check = (active_nav_copy =! state );
            
            int state2 = NAVTYPE_MOBIDIK_RELEASE;
            bool check2 = (active_nav_copy =! state2 );
           
        if ( check  && check2) // TODO what kind of feedback during mobidik collection?
        {
            ROS_INFO ( "Waypoint done notification received" );
            if ( nav_state.wayp_n<=waypoint_ids_.size() )
                ropod_progress_msg.area_name = waypoint_ids_[nav_state.wayp_n-1];
                ropod_progress_msg.action_id = action_msg.action_id;
                ropod_progress_msg.action_type = action_msg.type;
                ropod_progress_msg.status = ropod_ros_msgs::TaskProgressGOTO::REACHED;
                ropod_progress_msg.sequenceNumber = nav_state.wayp_n;
                ropod_progress_msg.totalNumber = waypoint_ids_.size();
                ropod_task_goto_fb_pub_.publish ( ropod_progress_msg );
            // Update coming waypoint
        }
    }
    else if ( nav_state.fb_nav == NAV_GOTOPOINT )
    {
            int state = NAVTYPE_MOBIDIK_COLLECTION;
            int active_nav_copy = active_nav ;
            bool check = (active_nav_copy =! state );
            
            int state2 = NAVTYPE_MOBIDIK_RELEASE;
            bool check2 = (active_nav_copy =! state2 );
           
        if ( check  && check2) // TODO what kind of feedback during mobidik collection?
        {
            if ( nav_state.wayp_n<=waypoint_ids_.size() )
                ropod_progress_msg.area_name = waypoint_ids_[nav_state.wayp_n-1];
            ropod_progress_msg.action_id = action_msg.action_id;
            ropod_progress_msg.action_type = action_msg.type;
            ropod_progress_msg.status = ropod_ros_msgs::TaskProgressGOTO::ONGOING;
            ropod_progress_msg.sequenceNumber = nav_state.wayp_n;
            ropod_progress_msg.totalNumber = waypoint_ids_.size();
            ropod_task_goto_fb_pub_.publish ( ropod_progress_msg );
        }
    }

    // Send navigation command
    if ( send_goal_ )
         sendGoal_pub_.publish(goal_.target_pose); //ac_->sendGoal ( goal_ );
    if (send_mn_goal_)
        mn_sendGoal_pub_.publish(mn_goal_);
    
    ObjectMarkers_pub_.publish( markerArray );
    LLCmodeSet_pub_.publish(controlMode_);
        

}

ED_REGISTER_PLUGIN(RopodNavigation)
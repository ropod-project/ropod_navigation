#include "demo_navigation.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

wm::SimplifiedWorldModel simple_wm;
WaypointNavigation waypoint_navigation;
ElevatorNavigation elevator_navigation;
ropod_ros_msgs::ropod_door_detection door_status;
ropod_ros_msgs::TaskProgressGOTO ropod_progress_msg;


enum {
    NAVTYPE_WAYPOINT = 0,
    NAVTYPE_ELEVATOR,
    NAVTYPE_NONE
};

int active_nav = NAVTYPE_NONE;
bool action_msg_received = false;
bool prepare_next_loc = false;
ropod_ros_msgs::Action action_msg;
ropod_ros_msgs::Action action_msg_rec;

void move_base_fbCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg)
{
    waypoint_navigation.base_position = msg;
    elevator_navigation.base_position = msg;
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

// this needs to be replaced with a query to the world model
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
    std::string moveBaseFeedbackTopic;
    std::string moveBaseCancelTopic;

    n.param<std::string> ( "move_base_server", moveBaseServerName, "/move_base" );
    n.param<std::string> ( "move_base_feedback_topic", moveBaseFeedbackTopic, "/move_base/feedback" );
    n.param<std::string> ( "move_base_cancel_topic", moveBaseCancelTopic, "/move_base/cancel" );

    sub_movebase_fb_ =   n.subscribe<move_base_msgs::MoveBaseActionFeedback> ( moveBaseFeedbackTopic, 10, move_base_fbCallback );
    sub_ccu_commands_ = n.subscribe<ropod_ros_msgs::Action> ( "/ropod_demo_plan", 10, actionCallback );
    subdoor_status_ = n.subscribe<ropod_ros_msgs::ropod_door_detection> ( "/door", 10, doorDetectCallback );

    door_status.closed = false;
    door_status.open = false;
    door_status.undetectable = true;
    
    movbase_cancel_pub_ = n.advertise<actionlib_msgs::GoalID> ( moveBaseCancelTopic, 1 );
    ropod_task_fb_pub_ = n.advertise<ropod_ros_msgs::ropod_demo_status_update> ( "/ropod_task_feedback", 1 );

    ac_ = new MoveBaseClient ( moveBaseServerName, true );

    while ( !ac_->waitForServer ( ros::Duration ( 5.0 ) ) )
    {
        ROS_INFO ( "Waiting for the move_base action server to come up" );
    }

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
            // Extract all areas and waypoints to go the corresponding location
            for ( std::vector<ropod_ros_msgs::Area>::const_iterator curr_area = action_msg.areas.begin();
                    curr_area != action_msg.areas.end(); ++curr_area )
            {
                geometry_msgs::PoseStamped p = getPoseFromWorldModel ( curr_area->name );
                std::string area_type = getAreaTypeFromWorldModel ( curr_area->name );
                path_msg_.poses.push_back ( p );
                waypoint_ids_.push_back ( curr_area->name );
            }
            waypoint_navigation.startNavigation ( path_msg_ );
            active_nav = NAVTYPE_WAYPOINT;
        }
        else if ( action_msg.type == "ENTER_ELEVATOR" )
        {
            // here we need to extract the two navigation poses to enter elevator
            // and pass it to elevator_navigation
            // geometry_msgs::PoseStamped p = getElevatorPoseFromWorldModel(floor id?)
//            elevator_navigation.startNavigation(simple_wm.elevator1,path_msg);
            active_nav = NAVTYPE_ELEVATOR;
        }
    }

    TaskFeedbackCcu nav_state;

    // Select the corresponding navigation
    switch ( active_nav )
    {

    case NAVTYPE_WAYPOINT:
        // ROS_INFO("NAV_WAYPOINT");
        nav_state = waypoint_navigation.callNavigationStateMachine ( movbase_cancel_pub_, &goal_, send_goal_ );
        break;

    case NAVTYPE_ELEVATOR:
        // ROS_INFO("NAV_ELEVATOR");
        nav_state = elevator_navigation.callNavigationStateMachine ( movbase_cancel_pub_, &goal_, send_goal_, simple_wm.elevator1, door_status );
        if ( nav_state.fb_nav == NAV_DONE )
        {
            active_nav = NAVTYPE_NONE;
            prepare_next_loc = true;
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
        ROS_INFO ( "NAV_ELEVATOR DONE!" );
        active_nav = NAVTYPE_NONE;
        prepare_next_loc = true;
    }
    else if ( nav_state.fb_nav == NAV_WAYPOINT_DONE )
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
    else if ( nav_state.fb_nav == NAV_GOTOPOINT )
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

    // Send navigation command
    if ( send_goal_ )
        ac_->sendGoal ( goal_ );

}

ED_REGISTER_PLUGIN(RopodNavigation)
#include "elevator_navigation.h"

/*--------------------------------------------------------*/
ElevatorNavigation::ElevatorNavigation()
{
    this->inside_elevator = false;
    this->goal_sent = false;
};

/*--------------------------------------------------------*/
ElevatorNavigation::~ElevatorNavigation()
{
    base_position.reset();
};

void ElevatorNavigation::initElevatorNavigation(int elevator_id, int elevator_door_id)
{
    this->elevator_id = elevator_id;
    if (!this->inside_elevator)
    {
        this->resetNavigation();
        this->getElevatorWaypoints(elevator_id, elevator_door_id);
        this->nav_state = GOTO_WAITING_POINT;
    }
    else
    {
        this->nav_state = RIDE_ELEVATOR;
    }
}

void ElevatorNavigation::getElevatorWaypoints(int elevator_id, int elevator_door_id)
{
    ropod_ros_msgs::GetElevatorWaypointsGoal elevator_waypoints_goal;
    elevator_waypoints_goal.elevator_id = elevator_id;
    elevator_waypoints_goal.door_id = elevator_door_id;

    this->elevator_waypoints_client->sendGoal(elevator_waypoints_goal);
    bool waypoint_client_completed = this->elevator_waypoints_client->waitForResult(ros::Duration(10.0));
    if (waypoint_client_completed)
    {
        auto elevator_waypoint_result = this->elevator_waypoints_client->getResult();
        this->waiting_pose = elevator_waypoint_result->wp_outside;
        this->inside_elevator_pose = elevator_waypoint_result->wp_inside;
        ROS_INFO("Waypoints for elevator %d and door %d assigned", elevator_id, elevator_door_id);
    }
    else
    {
        ROS_ERROR("Waypoints for elevator %d and door %d could not be retrieved", elevator_id, elevator_door_id);
        throw "Elevator waypoints could not be retrieved";
    }

    ropod_ros_msgs::GetTopologyNodeGoal topology_node_goal;
    topology_node_goal.id = elevator_door_id;
    topology_node_goal.type = "door";

    this->topology_node_client->sendGoal(topology_node_goal);
    bool topology_client_completed = this->topology_node_client->waitForResult(ros::Duration(10.0));
    if (topology_client_completed)
    {
        auto topology_client_result = this->topology_node_client->getResult();
        this->elevator_door_position = topology_client_result->position;
    }
    else
    {
        ROS_ERROR("Topology information for door %d could not be retrieved", elevator_door_id);
        throw "Door topology information could not be retrieved";
    }
}

/*--------------------------------------------------------*/
void ElevatorNavigation::setWaitingPose(maneuver_navigation::Goal &mn_goal, bool& send_goal)
{
    mn_goal.start.header.frame_id = "map";
    mn_goal.start.header.stamp = ros::Time::now();
    mn_goal.start.pose = base_position->pose;

    mn_goal.goal.header.frame_id = "map";
    mn_goal.goal.header.stamp = ros::Time::now();
    mn_goal.goal.pose.position = this->waiting_pose.position;
    mn_goal.goal.pose.orientation.w = this->waiting_pose.orientation.w;
    mn_goal.goal.pose.orientation.x = this->waiting_pose.orientation.x;
    mn_goal.goal.pose.orientation.y = this->waiting_pose.orientation.y;
    mn_goal.goal.pose.orientation.z = this->waiting_pose.orientation.z;

    goal.target_pose.pose = this->waiting_pose;
    send_goal = true;
}

/*--------------------------------------------------------*/
void ElevatorNavigation::setInsideElevatorPose(maneuver_navigation::Goal &mn_goal, bool& send_goal)
{
    mn_goal.start.header.frame_id = "map";
    mn_goal.start.header.stamp = ros::Time::now();
    mn_goal.start.pose = base_position->pose;

    mn_goal.goal.header.frame_id = "map";
    mn_goal.goal.header.stamp = ros::Time::now();
    mn_goal.goal.pose.position = this->inside_elevator_pose.position;
    mn_goal.goal.pose.orientation.w = this->inside_elevator_pose.orientation.w;
    mn_goal.goal.pose.orientation.x = this->inside_elevator_pose.orientation.x;
    mn_goal.goal.pose.orientation.y = this->inside_elevator_pose.orientation.y;
    mn_goal.goal.pose.orientation.z = this->inside_elevator_pose.orientation.z;

    goal.target_pose.pose = this->inside_elevator_pose;
    send_goal = true;
}

/*--------------------------------------------------------*/
void ElevatorNavigation::setOutsideElevatorPose(maneuver_navigation::Goal &mn_goal, bool& send_goal, std::string outside_area_id)
{
    // TODO: get the outside elevator coming with the EXIT_ELEVATOR message
    ropod_ros_msgs::GetTopologyNodeGoal topology_node_goal;
    topology_node_goal.id = std::stoi(outside_area_id);
    topology_node_goal.type = "local_area";

    this->topology_node_client->sendGoal(topology_node_goal);
    bool topology_client_completed = this->topology_node_client->waitForResult(ros::Duration(10.0));
    if (topology_client_completed)
    {
        auto topology_client_result = this->topology_node_client->getResult();
        ropod_ros_msgs::Position outside_position = topology_client_result->position;

        goal.target_pose.pose.position.x = outside_position.x;
        goal.target_pose.pose.position.y = outside_position.y;
        goal.target_pose.pose.position.z = 0.0;

        // TODO: handle the orientation better (e.g. what if we have a cart and we cannot easily get out)
        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z = 0.0;
        goal.target_pose.pose.orientation.w = 1.0;
    }
    else
    {
        ROS_ERROR("Topology information for area %s could not be retrieved", outside_area_id.c_str());
        throw "Area topology information could not be retrieved";
    }

    mn_goal.start.header.frame_id = "map";
    mn_goal.start.header.stamp = ros::Time::now();
    mn_goal.start.pose = base_position->pose;

    mn_goal.goal.header.frame_id = "map";
    mn_goal.goal.header.stamp = ros::Time::now();
    mn_goal.goal.pose.position = goal.target_pose.pose.position;
    mn_goal.goal.pose.orientation.w = goal.target_pose.pose.orientation.w;
    mn_goal.goal.pose.orientation.x = goal.target_pose.pose.orientation.x;
    mn_goal.goal.pose.orientation.y = goal.target_pose.pose.orientation.y;
    mn_goal.goal.pose.orientation.z = goal.target_pose.pose.orientation.z;
    send_goal = true;
}

/*--------------------------------------------------------*/
void ElevatorNavigation::setDestinationFloor(int destination_floor)
{
    this->destination_floor = destination_floor;
}

/*--------------------------------------------------------*/
void ElevatorNavigation::pauseNavigation()
{
    nav_paused_req = true;
    ROS_INFO("Navigation paused");
    return;
};

/*--------------------------------------------------------*/
void ElevatorNavigation::resumeNavigation()
{
    nav_paused_req = false;
    ROS_INFO("Navigation resumed");
    return;
}

/*--------------------------------------------------------*/
void ElevatorNavigation::resetNavigation()
{
    this->route_busy = false;
    this->nav_paused_req = false;
    this->nav_state = IDLE;
    this->elevator_id = -1;
    this->destination_floor = -1;
    this->inside_elevator = false;
    this->goal_sent = false;
}

/*--------------------------------------------------------*/
void ElevatorNavigation::stopNavigation()
{
    //base_position.reset();
    this->route_busy = false;
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
bool ElevatorNavigation::isDoorOpen()
{
    ropod_ros_msgs::GetDoorStatus door_status;
    door_status.request.robot_pose = this->base_position->pose;
    door_status.request.door_position = this->elevator_door_position;
    if (this->get_door_status_client.call(door_status))
    {
        if (door_status.response.status)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        ROS_ERROR("Could not check the door status");
        return false;
    }
}

/*--------------------------------------------------------*/
bool ElevatorNavigation::destinationFloorReached()
{
    floor_detection::DetectFloor detect_floor;
    if (this->get_floor_client.call(detect_floor))
    {
        if (detect_floor.response.floor == this->destination_floor)
        {
            map_switcher::SwitchMap switch_map;
            switch_map.request.entry_wormhole = "elevator_" + std::to_string(this->elevator_id);
            switch_map.request.new_map = "map_floor" + std::to_string(this->destination_floor);

            ROS_INFO("[elevator_navigation] Reached floor %d; switching map to %s", this->destination_floor, switch_map.request.new_map.c_str());
            if (this->switch_map_client.call(switch_map) && switch_map.response.success)
            {
                ROS_INFO("[elevator_navigation] Map switched");
                return true;
            }
            else
            {
                ROS_ERROR("[elevator_navigation] Map could not be switched");
                return false;
            }
        }
        else
        {
            return false;
        }
    }
    else
    {
        ROS_ERROR("[elevator_navigation] Could not get the current floor");
        return false;
    }
}

/*--------------------------------------------------------*/
TaskFeedbackCcu ElevatorNavigation::callNavigationStateMachine(maneuver_navigation::Goal &mn_goal, bool& send_goal, std::string outside_area_id, int destination_floor)
{
    TaskFeedbackCcu feedback_msg;
    feedback_msg.fb_nav = NAV_BUSY;
    send_goal = false;
    if (nav_state == IDLE)
    {
        feedback_msg.fb_nav = NAV_IDLE;
    }
    else if (nav_state == GOTO_WAITING_POINT)
    {
        if (!this->goal_sent)
        {
            this->setWaitingPose(mn_goal, send_goal);
            this->goal_sent = true;
        }

        if (this->isWaypointAchieved())
        {
            feedback_msg.fb_nav = NAV_DONE;
            this->nav_state = WAIT_FOR_ELEVATOR;
            this->goal_sent = false;
        }
    }
    else if (nav_state == WAIT_FOR_ELEVATOR)
    {
        if (!inside_elevator && this->isDoorOpen())
        {
            this->nav_state = ENTER_ELEVATOR;
        }
    }
    else if (nav_state == ENTER_ELEVATOR)
    {
        if (!this->goal_sent)
        {
            this->setInsideElevatorPose(mn_goal, send_goal);
            this->goal_sent = true;
        }

        if (this->isWaypointAchieved())
        {
            feedback_msg.fb_nav = NAV_DONE;
            this->nav_state = RIDE_ELEVATOR;
            this->inside_elevator = true;
            this->goal_sent = false;
        }
    }
    else if (nav_state == RIDE_ELEVATOR)
    {
        if (!this->goal_sent)
        {
            this->setDestinationFloor(destination_floor);
            this->goal_sent = true;
        }

        if (this->destinationFloorReached())
        {
            feedback_msg.fb_nav = NAV_DONE;
            this->nav_state = EXIT_ELEVATOR;
            this->goal_sent = false;
        }
    }
    else if (nav_state == EXIT_ELEVATOR)
    {
        if (!this->goal_sent)
        {
            this->setOutsideElevatorPose(mn_goal, send_goal, outside_area_id);
            this->goal_sent = true;
        }

        if (this->isWaypointAchieved())
        {
            feedback_msg.fb_nav = NAV_DONE;
            this->nav_state = IDLE;
            this->inside_elevator = false;
            this->goal_sent = false;
        }
    }

    return feedback_msg;
}

#include "maneuver_navigation.h"

namespace maneuver_n
{
ManeuverNavigation::ManeuverNavigation(tf::TransformListener& tf) :
tf_(tf)
{    
    initialized_ = false;
};


ManeuverNavigation::~ManeuverNavigation() {};

void ManeuverNavigation::init() 
{    

    costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    costmap_ = costmap_ros_->getCostmap();
    
    world_model_ = new base_local_planner::CostmapModel(*costmap_);
//     maneuver_planner = maneuver_planner::ManeuverPlanner("maneuver_planner",global_costmap_ros);    
    try{
//        local_planner.initialize("traj_planneer_ros", &tf, local_costmap_ros);
    } catch(...) {
      // 
        ROS_FATAL("Failed to initialize the global planner");
      exit(1);
    }
    
    initialized_ = true;

};






// bool ManeuverNavigation:: gotoGoal(const geometry_msgs::PoseStamped& goal) 
// {
//     
//     tf::Stamped<tf::Pose> global_pose;
//     geometry_msgs::PoseStamped start;
//      
//         if(!global_costmap_ros->getRobotPose(global_pose)){
//               ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
//           return false;
//         }
//     tf::poseStampedTFToMsg(global_pose, start);
//         
//       
//     maneuver_planner.makePlan(start,goal, plan);
//     return true; // TODO: implement
// 
// };

bool ManeuverNavigation::isGoalReachable() 
{
    return true; // TODO: implement

};


//we need to take the footprint of the robot into account when we calculate cost to obstacles
double ManeuverNavigation::footprintCost(double x_i, double y_i, double theta_i)
{
    if(!initialized_)
    {
        ROS_ERROR("The navigator has not been initialized");
        return -1.0;
    }

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();

    //if we have no footprint... do nothing
    if(footprint.size() < 3)
        return -1.0;

    //check if the footprint is legal
    double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
    return footprint_cost;
}


bool ManeuverNavigation::checkFootprintOnGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& plan, const double& max_ahead_dist, double& dist_before_obs)
{
    tf::Stamped<tf::Pose> global_pose;
    dist_before_obs = 0.0;
    if(!costmap_ros_->getRobotPose(global_pose))
        return false;    
    // First find the closes point from the robot pose to the path   
    double dist_to_path_min = 1e3;
    double dist_to_path; 
    tf::Pose pose_temp;
    tf::Quaternion quat_temp;
    int index_pose;
    double yaw, pitch, roll;
    double x = global_pose.getOrigin().getX();
    double y = global_pose.getOrigin().getY();
    int i;

    for (i =0; i < plan.size(); i++) 
    {
        dist_to_path = hypot(plan[i].pose.position.x-x,plan[i].pose.position.y-y);
        if(dist_to_path < dist_to_path_min)
        {
            dist_to_path_min = dist_to_path;
            index_pose = i; // TODO: Do this in a smarter way. Remebering lat position. Index needs to be resseted when ther is a replan
        }
        else
            break;
    }    
    // Now start checking poses in the future up to the desired distance
    double total_ahead_distance = 0.0;
    dist_before_obs = max_ahead_dist;
    double dist_next_point;
    double footprint_cost;
    bool is_traj_free = true;
    tf::Stamped<tf::Pose> pose_from_plan;
    
    for (i = index_pose; i < plan.size()-1; i++) 
    {
        dist_next_point = hypot(plan[i].pose.position.x-plan[i+1].pose.position.x,plan[i].pose.position.y-plan[i+1].pose.position.y);
        total_ahead_distance += dist_next_point;
        if( total_ahead_distance < max_ahead_dist)
        {
            tf::poseStampedMsgToTF(plan[i+1],pose_from_plan);             
            pose_from_plan.getBasis().getEulerYPR(yaw, pitch, roll);
            footprint_cost = footprintCost(pose_from_plan.getOrigin().getX(), pose_from_plan.getOrigin().getY(), yaw);
            if( footprint_cost < 0 )
            {
                is_traj_free = false;
                dist_before_obs = total_ahead_distance;
                break;
            }
        }
        else
            break;
    }    
    return is_traj_free;
    
    
}

 
}

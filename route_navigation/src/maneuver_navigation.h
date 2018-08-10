#ifndef MANEUVER_NAV_HH
#define MANEUVER_NAV_HH

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

// All this is needed to interface with teh costmap and thus check for obstacles
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>



namespace maneuver_n {
    
class ManeuverNavigation
{


public:

    

    ManeuverNavigation(tf::TransformListener& tf);

    ~ManeuverNavigation();

    void init();
//     void pauseNavigation();
//     void resumeNavigation();
//     void stopNavigation();
//     bool isPositionValid();    
//     bool isGoalReached();
    bool isGoalReachable();
//     bool executeCycle();
    
    double footprintCost(double x_i, double y_i, double theta_i);
    bool   checkFootprintOnGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& plan, const double& max_ahead_dist, double& dist_before_obs);
//     bool gotoGoal(const geometry_msgs::PoseStamped& goal);
    
//    maneuver_planner::ManeuverPlanner  maneuver_planner;   
//    base_local_planner::TrajectoryPlannerROS local_planner;
   std::vector<geometry_msgs::PoseStamped> plan;        
   
    costmap_2d::Costmap2DROS* costmap_ros_;
      costmap_2d::Costmap2D* costmap_;
      base_local_planner::WorldModel* world_model_; ///< @brief The world model that the controller will use  
      
private:      
      bool initialized_;
      tf::TransformListener& tf_;
      
      
//       bool checkFootprintOnGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& plan, const double max_ahead_dist, double dist_before_obs);

};

}


#endif

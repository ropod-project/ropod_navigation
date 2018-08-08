#include "simplified_world_model.h"


namespace wm
{

SimplifiedWorldModel::SimplifiedWorldModel() {
  
    // Corridor 1: Not used
    double l1C1[] = {4.22, -1.22, 3.57, -6.44};
    double wpl1C1[] = {4.5, -1.53, 3.92, -6.6};
    double l2C1[] = {5.4, -1.5, 5.04, -6.64};
    double wpl2C1[] = {5.05, -1.56, 4.63, -6.64};
    corridor1.init(l1C1,l2C1, wpl1C1, wpl2C1);
    
    
     
    // Elevator 1
    std::vector<PointWm> convex_area_elev;
    std::vector<PointWm> line_door;
    PoseWm wayp1_elevator;
    PoseWm wayp2_elevator;
    PoseWm  wayp_wait;
    PoseWm wayp_out; 
        
    PointWm p1;
    p1.x = 0.0;
    p1.y = 0.0;
    PointWm p2;
    p2.x = 0.0;
    p2.y = 0.0;
    PointWm p3;
    p3.x = 0.0;
    p3.y = 0.0;
    PointWm p4;
    p4.x = 0.0;
    p4.y = 0.0;
    convex_area_elev.push_back(p1);
    convex_area_elev.push_back(p2);
    convex_area_elev.push_back(p3);
    convex_area_elev.push_back(p4);

    p1.x = -8.5;
    p1.y = 3.3;
    p2.x = -8.5;
    p2.y = 2.35;
    line_door.push_back(p1);
    line_door.push_back(p2);
    

//     wayp1_elevator.position.x = -5.27;
//     wayp1_elevator.position.y = 0.025;
//     wayp1_elevator.position.z = 0.0;
//     wayp1_elevator.orientation.x = 0.0;
//     wayp1_elevator.orientation.y = 0.0;
//     wayp1_elevator.orientation.z = 0.012;
//     wayp1_elevator.orientation.w = 1.0;
    
    wayp1_elevator.position.x = -7.27;
    wayp1_elevator.position.y = 0.025;
    wayp1_elevator.position.z = 0.0;
    wayp1_elevator.orientation.x = 0.0;
    wayp1_elevator.orientation.y = 0.0;
    wayp1_elevator.orientation.z = 0.012;
    wayp1_elevator.orientation.w = 1.0;
    
    wayp2_elevator.position.x = -5.27;
    wayp2_elevator.position.y = 0.025;
    wayp2_elevator.position.z = 0.0;
    wayp2_elevator.orientation.x = 0.0;
    wayp2_elevator.orientation.y = 0.0;
    wayp2_elevator.orientation.z = 1.0;
    wayp2_elevator.orientation.w = 0.012;    
    

    wayp_wait.position.x = -8.1;
    wayp_wait.position.y = -1.87;
    wayp_wait.position.z = 0.0;
    wayp_wait.orientation.x = 0.0;
    wayp_wait.orientation.y = 0.0;
    wayp_wait.orientation.z = 0.71;
    wayp_wait.orientation.w = 0.71;


    wayp_out.position.x = -8.12;
    wayp_out.position.y = -1.53;
    wayp_out.position.z = 0.0;
    wayp_out.orientation.x = 0.0;
    wayp_out.orientation.y = 0.0;
    wayp_out.orientation.z = -0.72;
    wayp_out.orientation.w = 0.7;
    
    
    elevator1.init(convex_area_elev, line_door, wayp1_elevator, wayp2_elevator, wayp_wait, wayp_out);




};

SimplifiedWorldModel::~SimplifiedWorldModel() {
};

PoseWm getWMPose(const geometry_msgs::PoseStamped &pose)
{
    PoseWm p;
    p.position.x = pose.pose.position.x;
    p.position.y = pose.pose.position.y;
    p.position.z = pose.pose.position.z;
    p.orientation.x = pose.pose.orientation.x;
    p.orientation.y = pose.pose.orientation.y;
    p.orientation.z = pose.pose.orientation.z;
    p.orientation.w = pose.pose.orientation.w;
    return p;
};

}



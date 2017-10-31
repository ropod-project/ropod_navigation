#include "elevator.h"


namespace wm
{
Elevator::Elevator() {

};
void Elevator:: init(std::vector<point_wm> convex_area_elev_, std::vector<point_wm> line_door_,
                     pose_wm wayp_elevator_, pose_wm  wayp_wait_, pose_wm wayp_out_) {
  
    /* Elevator 1
     * In this simplified model, an elevator is using these parameters
     * convex_area_elev: Area inside the elevator. TODO: Use this to find a position iniside
     * line_door:        Line describing the door position. TODO: Use this to know when the robot has crossed the door
     * wayp_elevator:    Pose inside the elevator.
     * wayp_wait:        Pose to wait for elevator. TODO: use in demo?
     * wayp_out:         Pose to navbiate after going out of elevator. TODO: This can be depending on each floor?
     */  
    convex_area_elev = convex_area_elev_;
    line_door = line_door_;
    wayp_elevator = wayp_elevator_;
    wayp_wait = wayp_wait_;
    wayp_out = wayp_out_;
};

Elevator::~Elevator() {};



bool Elevator:: is_entrance_detectable() {
    return true; // TODO: for now we assume the robot is facing the door

};
bool Elevator:: is_entrance_accesible() {
    return true; // TODO:for now we assume the door is open, once the laser data is integrated this will be checked
};

void Elevator:: go_into() {};
void Elevator:: go_out() {};

}


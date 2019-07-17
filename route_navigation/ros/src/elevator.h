

#ifndef ELEVATOR_WM_HH
#define ELEVATOR_WM_HH

#include <vector>
#include "datatypes_wm.h"

#include "route_navigation_defines.h"

#define ELEV_WAIT_DIST_TH 0.5

namespace wm
{
  class Elevator
  {
    
    public: Elevator();		      

    public: ~Elevator();

    public: bool isEntranceDetectable();
    public: bool isEntranceAccesible();
    public: void goIn();
    public: void goOut();
    
    public: void init(std::vector<PointWm> convex_area_elev_, std::vector<PointWm> line_door_, 
		     PoseWm wayp1_elevator_, PoseWm wayp2_elevator_, PoseWm  wayp_wait_, PoseWm wayp_out_);
    private: std::vector<PointWm> convex_area_elev;
    public: std::vector<PointWm> line_door;
    public: PoseWm wayp1_elevator;
    public: PoseWm wayp2_elevator;
    public: PoseWm wayp_wait;
    public: PoseWm wayp_out;   
    
    public: void setInsideElevatorPose(PoseWm pose1, PoseWm pose2);
    public: void setOutsideElevatorPose(PoseWm pose);



  };
  
}
#endif

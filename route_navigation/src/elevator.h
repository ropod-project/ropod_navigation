

#ifndef ELEVATOR_WM_HH
#define ELEVATOR_WM_HH

#include <vector>
#include "datatypes_wm.h"

#define ELEV_WAIT_DIST_TH 0.5

namespace wm
{
  class Elevator
  {
    
    public: Elevator();		      

    public: ~Elevator();

    public: bool is_entrance_detectable();
    public: bool is_entrance_accesible();
    public: void go_into();
    public: void go_out();
    
    public: void init(std::vector<point_wm> convex_area_elev_, std::vector<point_wm> line_door_, 
		     pose_wm wayp_elevator_, pose_wm  wayp_wait_, pose_wm wayp_out_);
    private: std::vector<point_wm> convex_area_elev;
    public: std::vector<point_wm> line_door;
    public: pose_wm wayp_elevator;
    public: pose_wm wayp_wait;
    public: pose_wm wayp_out;   



  };
  
}
#endif
